# 🐢 Laboratorio No. 04 – Robótica de Desarrollo
### *Intro a ROS 2 Humble – Turtlesim*

**Integrantes:**  
- Ariadna Contreras Nossa
- David Santiago Nagles Barajas  

## 1. 🧠 Introducción

## Introducción

El presente laboratorio tiene como objetivo profundizar en el uso del simulador **Turtlesim** dentro del ecosistema **ROS 2 Humble**, aplicando conceptos fundamentales como nodos, tópicos, servicios y publicación de mensajes en un entorno de control distribuido.

A diferencia de los ejemplos básicos proporcionados por ROS 2 (como `turtle_teleop_key`), en este laboratorio se desarrolla un **nodo propio de teleoperación**, capaz de:

- Capturar entradas de teclado en **modo raw** sin necesidad de presionar Enter.
- Controlar la tortuga con flechas direccionales mediante un sistema de movimiento continuo basado en ventanas temporales.
- Gestionar servicios nativos de `turtlesim`, tales como:
  - `/turtle1/set_pen` para activar y desactivar el lápiz,
  - `/turtle1/teleport_absolute` para realizar movimientos instantáneos,
  - `/clear` para limpiar el canvas.
- Dibujar automáticamente letras predefinidas (**A, C, N, D, S y B**) a partir de trayectorias normalizadas y posiciones personalizadas.
- Evitar conflictos entre teleoperación y dibujo mediante un sistema interno de bloqueo que garantiza la ejecución segura de cada acción.

El desarrollo integra de manera completa los conceptos de **publicación de mensajes**, **uso de servicios**, **timers**, y manejo de **eventos de entrada**, lo que permite evidenciar el funcionamiento real del modelo de comunicación de ROS 2. El código final implementa un controlador interactivo robusto, modular y extensible, alineado con los requerimientos del laboratorio LabSIR.

## 2. 🧩 Descripción del desarrollo

Para resolver los requerimientos del laboratorio, se construyó un nodo en Python capaz de integrar simultáneamente cuatro elementos fundamentales de ROS 2: **publicadores**, **servicios**, **timers** y **lectura de teclado en modo raw**. El desarrollo se organizó en cuatro etapas principales:

---

### ✔ 2.1 Lectura de teclado sin bloqueo (modo raw)

Se implementó una función personalizada `get_key()` basada en las librerías del sistema `termios`, `tty` y `select`, permitiendo:

- Captura inmediata de teclas sin necesidad de presionar Enter.
- Soporte para flechas del teclado mediante secuencias ANSI (`\x1b[A`, `\x1b[B`, etc.).
- Compatibilidad con pulsación de letras para dibujar figuras.
- Lectura no bloquante, indispensable para no detener el ciclo del nodo ROS 2.

Esta función constituye la base del sistema de control teleoperado.

---

### ✔ 2.2 Control manual mediante flechas

El nodo publica mensajes `Twist` en el tópico `/turtle1/cmd_vel` para mover la tortuga.  
A diferencia de un control tradicional, se implementó un **movimiento continuo por ventanas temporales**:

- Cada pulsación de flecha activa un movimiento por **0.5 segundos**.
- Durante ese tiempo no se aceptan letras ni nuevos comandos de dibujo.
- El movimiento angular (giros) y linear (avance/retroceso) se gestionan de forma independiente.

Esto evita que la tortuga se detenga inmediatamente al no detectar teclas entre iteraciones del timer.

---

### ✔ 2.3 Uso de servicios para controlar acciones especiales

Se utilizaron tres servicios fundamentales:

| Servicio | Función |
|---------|---------|
| `/turtle1/set_pen` | Activa/ desactiva el lápiz, cambia color y grosor |
| `/turtle1/teleport_absolute` | Teletransporta instantáneamente a la tortuga |
| `/clear` | Limpia la pantalla del simulador |

Funciones implementadas:

- `pen_up()` para desactivar trazo  
- `pen_down()` para activarlo  
- `teleport_to(x, y, θ)` para mover instantáneamente  
- `clear_screen()` ligado a la tecla **L**  

Estos servicios permiten evitar trazos indeseados y posicionar la tortuga para dibujar letras.

---

### ✔ 2.4 Sistema para dibujar letras

Cada letra (A, C, N, D, S y B) se definió como una **trayectoria normalizada**:

## 3. 📐 Diagrama de flujo (Mermaid)

A continuación se presentan los principales diagramas de flujo del proyecto, que describen el funcionamiento del nodo, el sistema de dibujo y la lógica de teleoperación.

---

### 🟦 3.1 Diagrama general del nodo TurtleController

```mermaid
flowchart TD

A[Inicio del nodo] --> B[Crear publisher /turtle1/cmd_vel]
B --> C[Crear clientes de servicios: set_pen, teleport, clear]
C --> D[Definir trayectorias y posiciones base de letras]
D --> E[Inicializar estados internos: drawing, teleop_active]
E --> F[Crear timer update() cada 0.05 s]
F --> G[Leer teclado con get_key()]

G --> H{¿Letra válida?}
H -- Sí --> I[Iniciar hilo de dibujo draw_letter()]
H -- No --> J{¿Flecha presionada?}

J -- Sí --> K[Calcular Twist y activar movimiento continuo]
K --> F

J -- No --> L{¿Tecla L?}
L -- Sí --> M[Llamar servicio clear()]
M --> F

L -- No --> F

### 🟦 3.1 Diagrama general del nodo TurtleController

```mermaid
flowchart TD

A[Inicio del nodo] --> B[Crear publisher /turtle1/cmd_vel]
B --> C[Crear clientes de servicios: set_pen, teleport, clear]
C --> D[Definir trayectorias y posiciones base de letras]
D --> E[Inicializar estados internos: drawing, teleop_active]
E --> F[Crear timer update() cada 0.05 s]
F --> G[Leer teclado con get_key()]

G --> H{¿Letra válida?}
H -- Sí --> I[Iniciar hilo de dibujo draw_letter()]
H -- No --> J{¿Flecha presionada?}

J -- Sí --> K[Calcular Twist y activar movimiento continuo]
K --> F

J -- No --> L{¿Tecla L?}
L -- Sí --> M[Llamar servicio clear()]
M --> F

L -- No --> F
```


## 4. 🐍 Código principal

El código se encuentra en:

```
src/my_turtle_controller/move_turtle.py
```

Fragmento representativo:

A continuación se muestra un **fragmento representativo** que resume la estructura general del nodo, incluyendo la lectura del teclado, el uso de servicios y el ciclo principal `update()`:

```python
class TurtleController(Node):

    def __init__(self):
        super().__init__("turtle_controller")

        # Publicador para movimiento
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Clientes de servicios
        self.set_pen = self.create_client(SetPen, "/turtle1/set_pen")
        self.teleport = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.clear = self.create_client(Empty, "/clear")

        # Sincronización con los servicios
        self.set_pen.wait_for_service()
        self.teleport.wait_for_service()
        self.clear.wait_for_service()

        # Estados internos
        self.drawing = False
        self.teleop_active = False
        self.move_until = 0.0

        # Timer principal del nodo
        self.timer = self.create_timer(0.05, self.update)

    # Lógica principal del nodo
    def update(self):
        key = get_key(0.05)

        # Movimiento continuo por ventana de tiempo
        if key is None:
            if time.time() < self.move_until:
                self.teleop_active = True
                return
            else:
                self.pub.publish(Twist())  # detener
                self.teleop_active = False
                return

        # Limpiar pantalla
        if key.upper() == "L":
            self.clear_screen()
            return

        # Dibujar letras
        if len(key) == 1 and key.upper() in self.letters:
            threading.Thread(target=self.draw_letter, args=(key.upper(),), daemon=True).start()
            return

        # Teleoperación con flechas
        twist = Twist()
        if key == UP:
            twist.linear.x = 2.0
        elif key == DOWN:
            twist.linear.x = -2.0
        elif key == LEFT:
            twist.angular.z = 2.0
        elif key == RIGHT:
            twist.angular.z = -2.0
        else:
            return

        self.move_until = time.time() + 0.5
        self.teleop_active = True
        self.pen_down()
        self.pub.publish(twist)
```

## 5. ▶️ Ejecución

```bash
ros2 run turtlesim turtlesim_node
```

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run my_turtle_controller move_turtle
```

## 6. 👁️ Resultados

*(Insertar imágenes de las letras)*

## 7. 🎥 Video

👉 **[Insertar enlace del video]**

## 8. 📝 Conclusiones

- Se comprendió el funcionamiento básico de ROS 2 con Turtlesim.  
- Se implementó control manual y automático desde un único script.  
- Se aplicaron conceptos de cinemática, publicación y lectura de teclado.  
- Se cumplieron todas las restricciones del laboratorio.
