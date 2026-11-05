import sys
import math
import time
import rclpy
import threading
import termios
import tty
import select
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32MultiArray, String
from rclpy.node import Node

def euler_from_quaternion(x, y, z, w):
    """Convierte un cuaternión a ángulos de Euler. Devuelve el ángulo Yaw (rotación en Z)."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class ControlZNNLaptop(Node):
    def __init__(self):
        super().__init__('nodo_znn_laptop')
        # MODIFICAR AQUÍ: 'CUR' para control por corriente/torque, 'VEL' para control por velocidad cinemática.
        self.modo_control = 'CUR' 
        
        # --- 1. CONFIGURACIÓN DE ROS 2 (Publicadores y Suscriptores) ---
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_cur = self.create_publisher(Float32MultiArray, '/cmd_current', 10)
        self.pub_sys = self.create_publisher(String, '/sys_command', 10)
        self.sub_odom = self.create_subscription(Pose2D, '/odom_pose', self.odom_callback, 10)
        self.sub_real_vel = self.create_subscription(Twist, '/real_vel', self.vel_callback, 10)
        self.sub_rasp_status = self.create_subscription(String, '/rasp_status', self.status_callback, 10)

        self.v_real = 0.0
        self.w_real = 0.0
        
        # --- 2. PARÁMETROS FÍSICOS DEL ROBOT (Modelo de Lagrange) ---
        # MODIFICAR AQUÍ: Si cambias de robot, pesa las piezas y mide las distancias.
        self.mc = 1.570         # Masa del chasis sin ruedas (kg)
        self.mw = 0.116         # Masa de cada rueda (kg)
        self.Ic = 0.02258       # Inercia del chasis respecto a su centro de masa (kg*m^2)
        self.Iw = 0.00009474    # Inercia de cada rueda respecto a su eje (kg*m^2)
        self.d  = -0.064        # Distancia del eje de las ruedas al centro de masa del robot (metros)
        self.L  = 0.144         # Distancia del centro del eje a una de las ruedas (metros)
        self.R  = 0.033         # Radio de la rueda (metros)
         
        # Cálculo de masa (m) e inercia (I) equivalentes del sistema completo
        self.m = self.mc + 2 * self.mw
        self.I = self.Ic + self.mc * (self.d**2) + 2 * self.mw * (self.L**2) + 2 * self.Iw

        # --- 3. PARÁMETROS DE LA RED NEURONAL DE ZHANG (ZNN) ---
        # D: Punto de control desacoplado. Debe ser distinto de 0 para evitar la singularidad de los robots diferenciales.
        self.D = -0.064         
        # eta: Tasa de convergencia de la ZNN. 
        # MODIFICAR AQUÍ: Aumentar 'eta' hace que el error se reduzca más rápido, pero exige más aceleración.
        self.eta = 3.9          
        # K_znn: Ganancia proporcional del error de posición cartesiano.
        # MODIFICAR AQUÍ: Si el robot se desvía de la ruta, subir este valor (ej. 7.8) lo forzará a regresar al camino.
        self.K_znn = np.array([[1.0, 0.0], [0.0, 1.0]]) * 7.8

        # --- 4. CONTROLADOR PI DE VELOCIDAD (Lazo de compensación) ---
        self.K = 2.2            # Factor de escalado de tiempo (ajusta la agresividad de los splines)
        # Ganancias PI para corregir diferencias entre la velocidad comandada por la ZNN y la real del robot
        # MODIFICAR AQUÍ: Subir Kp_v y Kp_w si el robot no alcanza la velocidad deseada. Bajar si oscila.
        self.Kp_v = 0.4         # Proporcional para velocidad lineal
        self.Kp_w = 0.3         # Proporcional para velocidad angular
        self.Ki_v = 0.08        # Integral para velocidad lineal (elimina error en estado estacionario)
        self.Ki_w = 0.05        # Integral para angular
        self.integral_v = 0.0
        self.integral_w = 0.0
        self.max_integral = 1.5 # Límite anti-windup para evitar que la integral crezca al infinito

        # --- 5. LÍMITES FÍSICOS (Saturación) ---
        # MODIFICAR AQUÍ: Basado en las RPM máximas de tus motores (ej. Dynamixel XM430)
        self.V_MAX_ROBOT = 0.26      # Velocidad lineal máxima (m/s)
        self.OMEGA_MAX_ROBOT = 1.82  # Velocidad angular máxima (rad/s)

        # --- 6. DISEÑO DE TRAYECTORIA (Waypoints) ---
        # MODIFICAR AQUÍ: Cambia estos arreglos para dibujar la ruta que quieres que el robot siga.
        self.t_wp = np.array([0, 1, 2, 3, 4]) * 5                 # Tiempos de referencia (segundos)
        self.x_wp = np.array([0.064, 1.0, 1.0, 0.0, 0.0])         # Coordenadas X
        self.y_wp = np.array([0.0, 0.0, -1.0, -1.0, 0.0])         # Coordenadas Y
        self.cs_x, self.cs_y, self.tf_seguro = self.inicializar_splines(self.t_wp, self.x_wp, self.y_wp)
        
        # Variables de estado del robot
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.q = np.array([[0.0], [0.0]]) # Vector de velocidades calculadas [v, omega]^T
        
        # Filtro de suavizado para la odometría (evita saltos bruscos en el cálculo diferencial)
        self.alpha_filtro = 0.35
        self.x_filtrado = 0.0
        self.y_filtrado = 0.0
        self.primer_dato_odom = True
        
        # Historial para las gráficas
        self.conexion_establecida = False
        self.hist_t, self.hist_x, self.hist_y = [], [], []
        self.hist_v, self.hist_w = [], []
        self.hist_xd, self.hist_yd = [], []
        self.hist_e_x, self.hist_e_y = [], []
        self.hist_tau_r, self.hist_tau_l = [], []
        
        # Temporizador de control (100 Hz -> dt = 0.01 s)
        self.dt = 0.01
        self.start_time = None
        self.meta_alcanzada = False
        self.abortado_emergencia = False

        self.hilo_teclado = threading.Thread(target=self.escuchar_teclado)
        self.hilo_teclado.daemon = True
        self.hilo_teclado.start()

        self.timer = self.create_timer(self.dt, self.bucle_control)
        self.get_logger().info("Esperando conexion de la Raspberry y recepcion de odometria...")

    def inicializar_splines(self, t_wp, x_wp, y_wp):
            """Genera una trayectoria suave mediante Splines Cúbicos asegurando que no exceda las velocidades máximas."""
            cs_x_orig = CubicSpline(t_wp, x_wp, bc_type='natural')
            cs_y_orig = CubicSpline(t_wp, y_wp, bc_type='natural')
            t_smooth = np.linspace(t_wp[0], t_wp[-1], 200)
            
            dx, dy = cs_x_orig(t_smooth, 1), cs_y_orig(t_smooth, 1)
            ddx, ddy = cs_x_orig(t_smooth, 2), cs_y_orig(t_smooth, 2)
            v = np.sqrt(dx**2 + dy**2)
            denom = dx**2 + dy**2
            denom[denom == 0] = 1e-6 # Evita división por cero
            omega = (dx * ddy - dy * ddx) / denom
            
            factor_v = np.max(np.abs(v)) / self.V_MAX_ROBOT
            factor_omega = np.max(np.abs(omega)) / self.OMEGA_MAX_ROBOT
            
            # Escala el tiempo total para garantizar que el robot sea capaz de seguir la ruta físicamente
            t_seguro = t_wp * max(1.0, factor_v, factor_omega) * self.K
            cs_x_seg = CubicSpline(t_seguro, x_wp, bc_type='natural')
            cs_y_seg = CubicSpline(t_seguro, y_wp, bc_type='natural')
            return cs_x_seg, cs_y_seg, t_seguro[-1]

    def vel_callback(self, msg):
        """Filtro paso bajo simple para suavizar la velocidad real leída de los encoders."""
        alpha_vel = 0.1 
        self.v_real = alpha_vel * msg.linear.x + (1 - alpha_vel) * self.v_real
        self.w_real = alpha_vel * msg.angular.z + (1 - alpha_vel) * self.w_real

    def odom_callback(self, msg):
        """Actualiza la posición del robot filtrada desde la odometría."""
        if self.primer_dato_odom:
            self.x_filtrado = msg.x
            self.y_filtrado = msg.y
            self.primer_dato_odom = False
        else:
            self.x_filtrado = self.alpha_filtro * msg.x + (1 - self.alpha_filtro) * self.x_filtrado
            self.y_filtrado = self.alpha_filtro * msg.y + (1 - self.alpha_filtro) * self.y_filtrado

        self.x = self.x_filtrado
        self.y = self.y_filtrado
        theta_real = msg.theta
        # Desfase de 180 grados (pi) por la orientación del modelo en Gazebo/Físico
        self.theta = math.atan2(math.sin(theta_real + math.pi), math.cos(theta_real + math.pi))

        if not self.conexion_establecida:
            self.conexion_establecida = True
            self.start_time = time.time()
            self.get_logger().info(f"Conexion establecida con la Raspberry! Arrancando trayectoria ZNN en Modo {self.modo_control}...")

    def status_callback(self, msg):
        """Muestra en pantalla el estado del puente (bridge) en la Raspberry Pi."""
        codigos = {
            "S1": "[RASPBERRY] Conexion Serial con OpenCR establecida (3M baudios).",
            "S2": "[RASPBERRY] ERROR CRITICO: No se pudo abrir el puerto Serial.",
            "M_VEL": "[RASPBERRY] Modo estatico bloqueado en: VELOCIDAD.",
            "M_CUR": "[RASPBERRY] Modo estatico bloqueado en: CORRIENTE.",
            "W1": "[RASPBERRY] Enlace confirmado por hardware. Watchdog activado.",
            "W2": "[RASPBERRY] ADVERTENCIA: Frenado suave por perdida de señal local.",
            "W3": "[RASPBERRY] WATCHDOG CRITICO: 5s sin datos. Apagando puente.",
            "F1": "[RASPBERRY] Cierre seguro (Trayectoria completada o abortada)."
        }
        
        mensaje = codigos.get(msg.data, f"[RASPBERRY] Codigo desconocido: {msg.data}")
        
        if "ERROR" in mensaje or "CRITICO" in mensaje:
            self.get_logger().error(mensaje)
        elif "ADVERTENCIA" in mensaje:
            self.get_logger().warn(mensaje)
        else:
            self.get_logger().info(mensaje)

    def escuchar_teclado(self):
        """Permite abortar la ejecución de emergencia presionando 'q' en la terminal."""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not self.meta_alcanzada and not self.abortado_emergencia:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    tecla = sys.stdin.read(1)
                    if tecla.lower() == 'q':
                        self.abortado_emergencia = True
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def bucle_control(self):
        """Lazo principal de control ejecutado a 100 Hz. Calcula Cinemática (ZNN) y Dinámica (Lagrange)."""
        if self.abortado_emergencia:
            self.stop_robot(emergencia=True)
            self.timer.cancel()
            raise SystemExit
            
        if not self.conexion_establecida:
            return

        t = time.time() - self.start_time

        # 1. EVALUAR TRAYECTORIA DESEADA
        if t <= self.tf_seguro:
            X_d = np.array([[self.cs_x(t)], [self.cs_y(t)]])
            X_d_dot = np.array([[self.cs_x(t, 1)], [self.cs_y(t, 1)]])
            X_d_ddot = np.array([[self.cs_x(t, 2)], [self.cs_y(t, 2)]])
        else:
            # Si el tiempo terminó, mantener la posición final
            X_d = np.array([[self.cs_x(self.tf_seguro)], [self.cs_y(self.tf_seguro)]])
            X_d_dot = X_d_ddot = np.array([[0.0], [0.0]])

        # 2. CINEMÁTICA ZNN (Ecuaciones de Transformación)
        # Matriz T de transformación cinemática
        T = np.array([
            [math.cos(self.theta), -self.D * math.sin(self.theta)],
            [math.sin(self.theta),  self.D * math.cos(self.theta)]
        ])
        # Inversa analítica de T
        T_inv = np.array([
            [math.cos(self.theta), math.sin(self.theta)],
            [-(1/self.D) * math.sin(self.theta), (1/self.D) * math.cos(self.theta)]
        ])
        omega_current = self.q[1, 0]
        # Derivada temporal de la matriz T
        T_dot = np.array([
            [-math.sin(self.theta) * omega_current, -self.D * math.cos(self.theta) * omega_current],
            [ math.cos(self.theta) * omega_current, -self.D * math.sin(self.theta) * omega_current]
        ])
        
        # Posición actual del punto de control desacoplado (X_h)
        X_h = np.array([
            [self.x + self.D * math.cos(self.theta)],
            [self.y + self.D * math.sin(self.theta)]
        ])

        distancia_a_meta = math.hypot(self.x_wp[-1] - X_h[0, 0], self.y_wp[-1] - X_h[1, 0])

        # Parar si hemos llegado al final
        if t > self.tf_seguro and distancia_a_meta < 0.01:
            self.stop_robot(emergencia=False)
            self.timer.cancel()
            self.meta_alcanzada = True
            raise SystemExit
            
        # 3. LEY DE ZHANG
        X_h_dot = np.dot(T, self.q)
        U = X_d_dot + np.dot(self.K_znn, (X_d - X_h)) # Velocidad deseada
        e = X_h_dot - U                               # Error ZNN
        
        # Aceleración calculada por la ZNN (q_dot = [v_dot, omega_dot]^T)
        q_dot = np.dot(T_inv, (X_d_ddot - np.dot(T_dot, self.q) - self.eta * e))
        
        # Integración numérica de Euler para obtener las velocidades a enviar (q)
        self.q = self.q + q_dot * self.dt
        
        # Saturación anti-windup: si supera la velocidad máxima, anular la aceleración respectiva
        if abs(self.q[0, 0]) >= self.V_MAX_ROBOT:
            self.q[0, 0] = np.clip(self.q[0, 0], -self.V_MAX_ROBOT, self.V_MAX_ROBOT)
            v_dot = 0
        else:
            v_dot = q_dot[0, 0]

        if abs(self.q[1, 0]) >= self.OMEGA_MAX_ROBOT:
            self.q[1, 0] = np.clip(self.q[1, 0], -self.OMEGA_MAX_ROBOT, self.OMEGA_MAX_ROBOT)
            w_dot = 0
        else:
            w_dot = q_dot[1, 0]

        v, omega = self.q[0, 0], self.q[1, 0]
        
        # Inversión física debido al eje de referencia
        v_phys = -v
        v_dot_phys = -v_dot
        
        # 4. COMPENSACIÓN PI (Lazo de velocidad interna)
        error_v = v_phys - self.v_real
        error_w = omega - self.w_real
        
        self.integral_v = np.clip(self.integral_v + error_v * self.dt, -self.max_integral, self.max_integral)
        self.integral_w = np.clip(self.integral_w + error_w * self.dt, -self.max_integral, self.max_integral)
        
        # 5. MODELO DINÁMICO DE LAGRANGE (Feedforward de Par)
        # T1_inercia: Componente traslacional (masa efectiva)
        T1_inercia = (self.m + (2 * self.Iw) / (self.R**2)) * v_dot_phys - self.mc * self.d * (omega**2)
        # T2_inercia: Componente rotacional (inercia efectiva)
        T2_inercia = (self.I + (2 * (self.L**2) * self.Iw) / (self.R**2)) * w_dot + self.mc * self.d * v_phys * omega
        
        # Se suma la prealimentación de Lagrange con la compensación PI
        T1 = T1_inercia + (self.Kp_v * error_v) + (self.Ki_v * self.integral_v)
        T2 = T2_inercia + (self.Kp_w * error_w) + (self.Ki_w * self.integral_w)
        
        # Desacoplamiento de torques (inversa de la matriz Q)
        tauR_base = ((self.R / 2.0) * (T1 + (1.0 / self.L) * T2))
        tauL_base = ((self.R / 2.0) * (T1 - (1.0 / self.L) * T2))
        
        # 6. MODELO DE FRICCIÓN (Viscosa + Coulomb)
        omega_rueda_r = (v_phys + (self.L) * omega) / self.R
        omega_rueda_l = (v_phys - (self.L) * omega) / self.R

        # MODIFICAR AQUÍ: Estas constantes (0.01546, 0.01681, etc.) deben identificarse experimentalmente 
        # para tu robot en particular. Representan cuánta fuerza se necesita solo para vencer el rozamiento.
        k_tanh = 100.0
        friccion_r = ((0.00153 + 0.00010 * 0) * omega_rueda_r) + ((0.01546 + 0.00100 * 1) * np.tanh(k_tanh * omega_rueda_r))
        friccion_l = ((0.00149 + 0.00010 * 0) * omega_rueda_l) + ((0.01681 + 0.00100 * 1) * np.tanh(k_tanh * omega_rueda_l))
        
        # Torque Final Solicitado
        tauR = tauR_base + friccion_r
        tauL = tauL_base + friccion_l

        # ¡ADVERTENCIA PARA DESARROLLADORES!: 
        # La instrucción actual trunca los torques negativos a casi cero (UMBRAL).
        # Esto impide que el robot frene activamente o gire sobre su propio eje.
        # Si se requiere marcha atrás o giros cerrados, reemplazar `max(tau, UMBRAL)` por:
        # tauR = np.clip(tauR, -TAU_MAX_MOTOR, TAU_MAX_MOTOR)
        UMBRAL = 0.00000000001  
        tauR = max(tauR, UMBRAL)
        tauL = max(tauL, UMBRAL)
        
        # 7. PUBLICACIÓN A ROS 2
        if self.modo_control == 'VEL':
            msg = Twist()
            msg.linear.x = -float(v)
            msg.angular.z = float(omega)
            self.pub_vel.publish(msg)
        elif self.modo_control == 'CUR':
            msg = Float32MultiArray()
            msg.data = [float(tauL), float(tauR)]
            self.pub_cur.publish(msg)

        # 8. GUARDAR HISTORIAL PARA GRÁFICAS
        self.hist_t.append(t)
        self.hist_x.append(X_h[0, 0])
        self.hist_y.append(X_h[1, 0])
        self.hist_v.append(v)
        self.hist_w.append(omega)
        self.hist_xd.append(X_d[0, 0])
        self.hist_yd.append(X_d[1, 0])
        self.hist_tau_r.append(tauR)
        self.hist_tau_l.append(tauL)
        
        error_pos = X_d - X_h
        self.hist_e_x.append(error_pos[0, 0])
        self.hist_e_y.append(error_pos[1, 0])

    def stop_robot(self, emergencia=True):
        """Detiene el robot enviando comandos de cero velocidad/torque."""
        if self.modo_control == 'VEL':
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.pub_vel.publish(twist_msg)
        elif self.modo_control == 'CUR':
            msg_cur = Float32MultiArray()
            msg_cur.data = [0.0, 0.0]
            self.pub_cur.publish(msg_cur)
            
        msg_sys = String()
        if not emergencia:
            msg_sys.data = "FINISH"
            self.pub_sys.publish(msg_sys)
        else:
            msg_sys.data = "MANUAL_STOP"
            self.pub_sys.publish(msg_sys)

    def graficar_resultados(self):
        """Genera las 4 gráficas interactivas al terminar la prueba."""
        print("\n[INFO] Generando graficas interactivas...")
        fig = plt.figure(figsize=(14, 10))
        
        plt.subplot(2, 2, 1)
        plt.plot(self.hist_x, self.hist_y, 'b-', linewidth=1.5, label='Ruta Robot')
        plt.plot(self.hist_xd, self.hist_yd, 'k--', linewidth=1, label='Referencia')
        plt.plot(self.x_wp, self.y_wp, 'ro', label='Waypoints')
        plt.title('Recorrido: CUADRADO MODIFICADO')
        plt.xlabel('X [m]'); plt.ylabel('Y [m]')
        plt.grid(True); plt.legend()
        
        plt.subplot(2, 2, 2)
        plt.plot(self.hist_t, self.hist_v, 'g-', label='v (m/s)')
        plt.plot(self.hist_t, self.hist_w, 'm-', label='w (rad/s)')
        plt.title('Perfil de Velocidades ZNN')
        plt.xlabel('Tiempo [s]'); plt.ylabel('Velocidad')
        plt.grid(True); plt.legend()
        
        plt.subplot(2, 2, 3)
        plt.plot(self.hist_t, self.hist_e_x, label='Error X')
        plt.plot(self.hist_t, self.hist_e_y, label='Error Y')
        plt.title('Error de Posicion')
        plt.xlabel('Tiempo [s]'); plt.ylabel('Error [m]')
        plt.grid(True); plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(self.hist_t, self.hist_tau_r, 'r-', label='Tau Der')
        plt.plot(self.hist_t, self.hist_tau_l, 'b-', label='Tau Izq')
        plt.title('Torques Calculados')
        plt.xlabel('Tiempo [s]'); plt.ylabel('Torque [N·m]')
        plt.grid(True); plt.legend(loc='upper right')
        
        plt.tight_layout()
        plt.show() 

def main(args=None):
    rclpy.init(args=args)
    node = ControlZNNLaptop()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    
    if node.meta_alcanzada or node.abortado_emergencia:
        node.graficar_resultados()
        
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()