## 🐢 ¿Qué es Turtlesim?

Turtlesim es un simulador muy simple que viene con ROS 2.  
Sirve para aprender a usar topics, servicios, nodos y mensajes sin necesidad de un robot real.  
Básicamente es una tortuguita que se mueve por la pantalla obedeciendo comandos de velocidad
o teletransportes que le enviamos desde un nodo.

El archivo principal para desarrollar el proyecto es **move_turtle.py**, que es el nodo que creamos para controlar la tortuga.
Está dentro del paquete `my_turtle_controller`, que se encuentra en la carpeta src dentro del workspace turtlesim_ws

La estructura del workspace es más o menos así:

Turtlesim_ws/  </br>
└── src/  </br>
  └── my_turtle_controller/ </br>
       └── my_turtle_controller/ </br>
                ├── init.py </br>
                └── move_turtle.py ← AQUÍ está el nodo que controla la tortuga </br>

De todas formas se deja el archivo  **move_turtle.py** afuera en simulation para facil acceso pero debe posicionarse anteriormente
