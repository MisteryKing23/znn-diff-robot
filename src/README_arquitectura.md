# Arquitectura del Sistema de Control

Este directorio contiene los tres códigos principales que conforman la columna vertebral de nuestro proyecto modular para el control del robot diferencial. La arquitectura se divide en tres niveles de procesamiento interconectados para garantizar un control en tiempo real, seguro y eficiente.

## 1. Nivel de Alto Nivel: Laptop (Ubuntu + ROS 2)
**Archivo principal:** `laptop_ubuntu.py`

Este nodo actúa como el cerebro del sistema. Ejecutado en una laptop con Ubuntu y ROS 2, es el encargado de procesar la carga matemática pesada y la planificación de trayectorias.

*   **Generación de Trayectoria:** Utiliza Splines Cúbicos para trazar una ruta suave que respete los límites físicos del robot (velocidad y aceleración).
*   **Red Neuronal de Zhang (ZNN):** Calcula la cinemática inversa resolviendo la singularidad típica de los robots diferenciales al desplazar el punto de control.
*   **Modelo Dinámico (Euler-Lagrange):** Realiza un cálculo *feedforward* del par motor (torque) necesario, tomando en cuenta las inercias, las masas y un modelo de fricción (Viscosa + Coulomb).
*   **Monitoreo y Gráficas:** Recibe la telemetría, calcula el error de posición en tiempo real y genera gráficas de rendimiento al finalizar la prueba.

## 2. Nivel Intermedio: Raspberry Pi (Middleware)
**Archivo principal:** `raspberry_pi.py`

Ubicada físicamente en el robot, la Raspberry Pi actúa como el puente de comunicación entre la red WiFi (ROS 2) y el hardware a nivel local (Serial).

*   **Puente de Comunicación:** Se suscribe a los tópicos de control (`/cmd_vel`, `/cmd_current`) publicados por la Laptop y los traduce a cadenas de texto serial ultrarrápidas (3M baudios) para la tarjeta OpenCR.
*   **Conversión Torque a Corriente:** Escala matemáticamente el par (N·m) calculado por la Laptop a valores de corriente en miliamperios (mA) que los motores Dynamixel pueden interpretar.
*   **Gestor de Seguridad (Watchdog):** Incorpora un sistema de seguridad crítico. Si la Raspberry Pi pierde conexión WiFi con la Laptop, ejecuta un frenado suave de emergencia y detiene los motores para evitar accidentes.

## 3. Nivel de Bajo Nivel: OpenCR (Firmware C++)
**Archivo principal:** `opencr_firmware.ino` (o archivo `.cpp` equivalente)

La placa OpenCR está dedicada exclusivamente a operaciones de lectura y escritura síncronas en tiempo real, interactuando directamente con el hardware.

*   **Controladores de Motores:** Utiliza el DynamixelSDK para enviar los comandos de corriente (modo `CUR`) o velocidad (modo `VEL`) a los actuadores.
*   **Telemetría y Odometría:** Lee los *ticks* de los encoders de las ruedas y los datos de la IMU integrada a alta velocidad.
*   **Cinemática Directa Local:** Calcula la posición $(x, y, \theta)$ y las velocidades reales $(v, \omega)$ directamente en la placa, garantizando que el bucle de control en la Laptop reciba retroalimentación precisa y sin latencias de procesamiento del sistema operativo.

## Flujo de Datos

1. La **Laptop** publica `[Torque Izquierdo, Torque Derecho]` en `/cmd_current`.
2. La **Raspberry** lo convierte a corriente, revisa el enlace y envía por serial `Corriente_Izq,Corriente_Der\n`.
3. La **OpenCR** inyecta la corriente a los motores, lee sensores y responde por serial `x,y,theta,v,w\n`.
4. La **Raspberry** publica esto en `/odom_pose` y `/real_vel`.
5. La **Laptop** ajusta su siguiente cálculo en base a esta información.