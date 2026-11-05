# ZNN-Diff-Robot: Control Dinámico mediante Redes Neuronales de Zhang

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow.svg)](https://www.python.org/)
[![Hardware](https://img.shields.io/badge/Hardware-Raspberry_Pi_4_%7C_OpenCR-green.svg)]()

Este repositorio contiene el código fuente completo para el modelado, identificación y control de un robot móvil diferencial utilizando una **Red Neuronal de Zhang (ZNN)** acoplada a un modelo dinámico de **Euler-Lagrange**. 

El proyecto resuelve la singularidad cinemática de los robots diferenciales desplazando el punto de control, y garantiza un seguimiento de trayectoria de alta precisión mediante un control por torque (corriente) que compensa activamente las inercias, masas y fricciones no lineales del sistema.

---

## Arquitectura del Sistema

El proyecto está diseñado de forma modular para garantizar el procesamiento en tiempo real y la seguridad del robot, dividiendo la carga en tres capas:

1. **Alto Nivel (Laptop con Ubuntu + ROS 2):** Calcula la ZNN, las matrices de transformación, el feedforward dinámico (masas, inercias, fricción) y publica los torques requeridos.
2. **Nivel Intermedio (Raspberry Pi + ROS 2):** Actúa como middleware a bordo del robot. Escala los torques a corrientes (mA), gestiona la conexión WiFi y activa el watchdog de emergencia.
3. **Bajo Nivel (OpenCR en C++):** Ejecuta la inyección de corrientes síncronas a los motores Dynamixel a 3 Mbps y retorna telemetría y odometría de alta frecuencia.

>  **Para una explicación detallada de los códigos de control, revisa el [README de Arquitectura en la carpeta `/src`](src/README.md).**

---

## Estructura del Repositorio

```text
znn-diff-robot/
├── README.md                      <-- Este documento
├── /src                           <-- Códigos de ejecución principal del robot
│   ├── laptop_ubuntu.py           # Nodo ZNN y cálculo dinámico (Laptop)
│   ├── raspberry_pi.py            # Nodo puente y Watchdog (Raspberry Pi)
│   ├── OpenCR.ino                 # Firmware principal del microcontrolador
│   └── README_arquitectura.md     # Documentación de la capa de control
│
└── /identificacion_friccion       <-- Módulo de calibración experimental
    ├── openCR_friccion.ino        # Firmware temporal para pruebas a 500 Hz
    ├── main_identificacion.py     # Script orquestador de perfiles de velocidad
    ├── ajuste_friccion.py         # Motor matemático (Mínimos Cuadrados)
    └── README_friccion.md         # Documentación de la calibración
```

---

##  Requisitos y Dependencias

* **Sistema Operativo:** Ubuntu 22.04 LTS
* **Middleware:** ROS 2 (Humble)
* **Librerías Python:** `numpy`, `scipy`, `matplotlib`, `pyserial`
* **Hardware:** 
  * Laptop para estación base.
  * Raspberry Pi (3/4) montada en el robot.
  * Tarjeta OpenCR 1.0.
  * Motores DYNAMIXEL (Protocolo 2.0).

---

##  Guía Rápida de Uso

Para replicar o utilizar este proyecto, puedes clonarlo directamente en tu terminal:

```bash
git clone https://github.com/tu-usuario/znn-diff-robot.git
cd znn-diff-robot
```

### Flujo de Trabajo Recomendado:

El uso del sistema se divide en dos fases principales:

**Fase 1: Identificación de Parámetros (Una sola vez)**
Antes de poder aplicar un control de par preciso, el robot necesita conocer su resistencia mecánica.
1. Flashea `openCR_friccion.ino` en la OpenCR.
2. Sigue las instrucciones en [`/identificacion_friccion`](identificacion_friccion/README.md) para obtener las constantes $d$ (fricción viscosa) y $c$ (Coulomb).
3. Ingresa los valores obtenidos en el script principal de la laptop (`laptop_ubuntu.py`).

**Fase 2: Ejecución del Control (Operación Regular)**
1. Flashea el firmware principal `OpenCR.ino` (ubicado en `/src`) en la tarjeta.
2. En la Raspberry Pi, ejecuta el nodo puente:
   ```bash
   ros2 run mi_paquete raspberry_pi
   ```
3. En la Laptop, ejecuta el controlador ZNN:
   ```bash
   ros2 run mi_paquete laptop_ubuntu
   ```
4. Presiona `q` en la terminal de la laptop en cualquier momento para abortar de emergencia. Al llegar a la meta, se desplegarán automáticamente las gráficas de rendimiento.
