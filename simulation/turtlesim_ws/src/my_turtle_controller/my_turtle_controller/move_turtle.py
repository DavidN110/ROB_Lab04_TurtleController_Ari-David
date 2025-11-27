
import sys
import select
import termios
import tty
import threading
import time

from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute

# Flechas (secuencias)
# Códigos ANSI de las flechas. Python los recibe así cuando el teclado está en modo raw.

UP = "\x1b[A"
DOWN = "\x1b[B"
LEFT = "\x1b[D"
RIGHT = "\x1b[C"

# Función para leer una tecla sin frenar el programa.
# Si presiono flechas, vienen en tres caracteres (por eso lo del "\x1b").
# timeout nos deja seguir moviendo a la tortuga aunque no haya teclas.
def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            c = sys.stdin.read(1)
            if c == "\x1b":
                c += sys.stdin.read(2)
            return c
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None


class TurtleController(Node):
    # Constructor del nodo. Aquí inicializo todo:
    # publisher, servicios, estados, letras, posiciones y el timer principal.
    def __init__(self):
        super().__init__("turtle_controller")

        # Publisher
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

       # Creo los servicios que la tortuga usa: lápiz, teletransporte y limpiar pantalla.
       # Toca esperar a que estén listos antes de usarlos.

        self.set_pen = self.create_client(SetPen, "/turtle1/set_pen")
        self.teleport = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.clear = self.create_client(Empty, "/clear")

        self.set_pen.wait_for_service()
        self.teleport.wait_for_service()
        self.clear.wait_for_service()

        print("\n====================================================")
        print("  Nodo de movimiento con flechas activado")
        print("\n  Controles:")
        print("     ↑  : avanzar")
        print("     ↓  : retroceder")
        print("     ←  : girar a la izquierda")
        print("     →  : girar a la derecha")
        print("\n  Letras disponibles: A, C, N, D, S, B")
        print("  Comando especial: L = limpiar pantalla")
        print("====================================================\n")

        # Estados internos del nodo.
        # drawing → si estoy dibujando una letra.
        # teleop_active → si estoy moviendo con flechas.
        # arrow_pressed → para bloquear letras mientras hay movimiento.
        # move_until → hasta cuándo debe seguir moviéndose la tortuga por cada flecha.

        self.drawing = False
        self.teleop_active = False
        self.arrow_pressed = False

        # NUEVO → tiempo hasta el cual debe seguir el movimiento
        self.move_until = 0.0

        # Trayectorias para cada letra.
        # Cada tupla es (p, nx, ny):
        # p = 0 → lápiz arriba, p = 1 → lápiz abajo.
        # nx, ny → coordenadas normalizadas que luego escalo y traslado.

        self.letters = {
            'A': [(0,0.5,2),(1,0,0),(0,0.5,2),(1,1,0),(0,0.25,1),(1,0.75,1)],
            'C': [(0,1,2),(1,0,2),(1,0,0),(1,1,0)],
            'N': [(0,0,2),(1,0,0),(0,0,2),(1,1,0),(1,1,2)],
            'D': [(0,0,2),(1,0,0),(0,0,2),(1,0.5,2),(1,1,1.4),(1,1,0.8),(1,0.5,0),(1,0,0)],
            'S': [(0,1,2),(1,0,2),(1,0,1),(1,1,1),(1,1,0),(1,0,0)],
            'B': [(0,0,2),(1,0,0),(0,0,2),(1,0.5,2),(1,0.5,1),(1,0,1),(1,1,1),(1,1,0),(1,0,0)]
        }

        # Posiciones donde arranca cada letra para que no se monten unas sobre otras.
        self.letter_origins = {
            'A': (2.5, 8.0),
            'C': (5.0, 8.0),
            'N': (7.5, 8.0),
            'D': (2.5, 5.0),
            'S': (5.0, 5.0),
            'B': (7.5, 5.0),
        }

        # Timer que llama a update() cada 50 ms.
        # Aquí es donde reviso teclas y muevo la tortuga.
        self.timer = self.create_timer(0.05, self.update)

#Servicios usados
    def pen_up(self):
        # Subo el lápiz (no dibuja). Básicamente es SetPen con 'off=True'.
        msg = SetPen.Request()
        msg.r = 255; msg.g = 255; msg.b = 255
        msg.width = 3
        msg.off = True
        self.set_pen.call_async(msg)
        
        # Bajo el lápiz (sí dibuja). Es igual al anterior pero con 'off=False'.
    def pen_down(self):
        msg = SetPen.Request()
        msg.r = 255; msg.g = 255; msg.b = 255
        msg.width = 3
        msg.off = False
        self.set_pen.call_async(msg)

        # Teletransporta la tortuga a coordenadas exactas.
        # Lo uso para dibujar las letras punto por punto sin hacer movimientos curvos.

    def teleport_to(self, x, y, theta=0.0):
        msg = TeleportAbsolute.Request()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self.teleport.call_async(msg)
        time.sleep(0.05)

    # Limpia el tablero del turtlesim.
    def clear_screen(self):
        req = Empty.Request()
        self.clear.call_async(req)
        time.sleep(0.05)

        # Función que dibuja una letra completa siguiendo su trayectoria.
        # Si estoy en modo flechas o ya estoy dibujando, no se entra aqui.
    def draw_letter(self, letter):

        if self.drawing or self.teleop_active:
            return

        self.drawing = True

        try:
            coords = self.letters[letter]
            base_x, base_y = self.letter_origins[letter]
            scale = 1.2

            p0, nx0, ny0 = coords[0]
            self.pen_up()
            self.teleport_to(base_x + nx0 * scale, base_y - ny0 * scale)

            # Convierto las coordenadas normalizadas en puntos reales del turtlesim.
            for p, nx, ny in coords:
                x = base_x + nx * scale
                y = base_y + ny * scale

                if p == 0:
                    self.pen_up()
                else:
                    self.pen_down()

                self.teleport_to(x, y)
                time.sleep(0.1)

            self.pen_up()

        finally:
            self.drawing = False


        #   Esta es la función principal que corre cada 0.05s.
        # Aquí leo las teclas, decido qué hacer y muevo la tortuga.
    def update(self):

        # bloqueo si está dibujando
        if self.drawing:
            return

        key = get_key(0.05)

        # Si no presioné nada, reviso si la tortuga todavía debe seguir moviéndose.
        # Esto hace que las flechas duren 0.5 segundos aunque toque la tecla rápido.

        if key is None:
            # Si hay movimiento activo, seguir moviendo
            if time.time() < self.move_until:
                self.teleop_active = True
                return
            else:
                # detener completamente
                self.arrow_pressed = False
                self.teleop_active = False
                stop = Twist()
                self.pub.publish(stop)
                return

        
        # Funcion para limpiar dibujo
        if key.upper() == "L":
            if not self.teleop_active:
                self.clear_screen()
            return

        # Si presiono una letra válida y no estoy en modo flechas, empiezo a dibujar.        
        if len(key) == 1:
            letter = key.upper()
            if letter in self.letters and not self.teleop_active:
                th = threading.Thread(target=self.draw_letter, args=(letter,), daemon=True)
                th.start()
            return

        # Movimiento manual con flechas.
        # Cada flecha activa medio segundo de movimiento continuo usando move_until.
        twist = Twist()

        if key == UP:
            twist.linear.x = 2.0
            self.move_until = time.time() + 0.5

        elif key == DOWN:
            twist.linear.x = -2.0
            self.move_until = time.time() + 0.5

        elif key == LEFT:
            twist.angular.z = 2.0
            self.move_until = time.time() + 0.5

        elif key == RIGHT:
            twist.angular.z = -2.0
            self.move_until = time.time() + 0.5

        else:
            return

        self.arrow_pressed = True
        self.teleop_active = True
        self.pen_down()
        self.pub.publish(twist)

 # Arranque del nodo ROS. 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
