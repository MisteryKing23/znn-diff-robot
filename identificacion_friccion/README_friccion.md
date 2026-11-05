# Identificación Experimental de Fricción

Este módulo contiene las herramientas necesarias para obtener experimentalmente las constantes de fricción viscosa ($d$) y de Coulomb ($c$) de cada rueda del robot diferencial. 

Estos parámetros son indispensables para alimentar el modelo dinámico de Euler-Lagrange en el control principal, permitiendo que el robot calcule el torque exacto necesario para vencer la resistencia mecánica de sus propios motores y engranajes.

## Archivos del Módulo

El proceso se divide en 3 componentes principales:

### 1. El Firmware: `openCR_friccion.ino`
**Ubicación de ejecución:** Placa OpenCR (Flashear previamente).
Este es un firmware de diagnóstico temporal. Deshabilita el control cinemático estándar e implementa un controlador PI de velocidad determinista a 500 Hz en modo corriente. Su objetivo es mantener las ruedas a velocidades constantes ("mesetas") y reportar por puerto serial cuánta corriente (torque) fue necesaria para mantener esa velocidad en régimen permanente, donde la aceleración es nula.

### 2. El Orquestador: `main_identificacion_friccion.py`
**Ubicación de ejecución:** Raspberry Pi.
Es el script principal que dirige el experimento. Se comunica por serial con la OpenCR y automatiza una rutina de pruebas escalonadas.
* Acelera el robot a diferentes velocidades predefinidas (ej. 0.3, 0.5, 0.8... rad/s).
* Espera a que la velocidad se estabilice.
* Promedia las lecturas de corriente para filtrar el ruido.
* Guarda todos los resultados brutos en archivos `.csv`.

### 3. El Motor Matemático: `ajuste_friccion.py`
**Ubicación de ejecución:** Raspberry Pi (Llamado automáticamente por el Main).
Una vez que el script principal termina de recolectar datos, invoca este código para procesarlos. Utiliza el método de Mínimos Cuadrados para ajustar los datos experimentales al modelo de fricción:

$$ \tau = d \cdot \eta + c \cdot \text{sgn}(\eta) $$

Donde:
* $\tau$: Torque aplicado.
* $\eta$: Velocidad angular de la rueda.
* $d$: Coeficiente de fricción viscosa.
* $c$: Coeficiente de fricción de Coulomb.

Este script filtra vibraciones y genera automáticamente los archivos `.json` con los parámetros finales y gráficas `.png` para visualizar la calidad del ajuste ($R^2$).

## Instrucciones de Uso

Para recalibrar el robot, sigue estos pasos rigurosamente en este orden:

1. **Preparar el Hardware:** Conecta la OpenCR a tu computadora y flashea el archivo `openCR_friccion.ino`.
2. **Posicionar el Robot:** Coloca el robot en la superficie donde operará (suelo) teniendo en cuenta que avanzará en línea recta hasta 1.5 metros.
3. **Ejecutar la Prueba:** Ingresa por terminal a la Raspberry Pi, navega a esta carpeta y ejecuta:
   ```bash
   python3 main_identificacion_friccion.py --modo suelo
   ```
4. **Resultados:** Al finalizar, el script detendrá el robot y generará los archivos `friccion_params.json` y `friccion_ajuste.png`. **Nota:** ¡No olvides volver a flashear el firmware principal en la OpenCR cuando termines!