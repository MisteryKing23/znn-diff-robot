import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32MultiArray, String
import serial
import time

def clip(valor, min_val, max_val):
    return max(min_val, min(valor, max_val))

class BridgeRaspberryOpenCR(Node):
    def __init__(self):
        super().__init__('nodo_bridge_raspberry')
        
        # --- MODIFICACIÓN 1: RECEPCIÓN DINÁMICA DE MODO DESDE LA LAPTOP ---
        self.modo_operacion = None # Se inicializa vacío. El OpenCR esperará órdenes.
        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 3000000, timeout=0.01)
            time.sleep(1.0)
            self.get_logger().info("Conexion Serial con OpenCR establecida a 3M baudios.")
            
            # Solo se reinicias la odometría del OpenCR. No fuerza ningún modo aún.
            self.ser.write(b"RESET\n")
            time.sleep(0.1)
            
        except Exception as e:
            self.get_logger().error(f"Error al abrir puerto Serial: {e}")
            self.ser = None

        self.pub_odom = self.create_publisher(Pose2D, '/odom_pose', 10)
        self.pub_vel_real = self.create_publisher(Twist, '/real_vel', 10)

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 10)
        self.sub_cmd_cur = self.create_subscription(Float32MultiArray, '/cmd_current', self.callback_cmd_current, 10)
        self.sub_sys = self.create_subscription(String, '/sys_command', self.callback_sys, 10)
        
        self.val1_cmd = 0.0
        self.val2_cmd = 0.0
        self.conexion_establecida = False
        self.ha_conectado_alguna_vez = False
        self.fin_intencional = False
        self.razon_fin = ""
        self.ultimo_tiempo_recibido = time.time()
        
        self.limite_watchdog = 0.5
        self.limite_desconexion_total = 5.0
        
        self.timer = self.create_timer(0.01, self.ciclo_comunicacion)
        self.get_logger().info("Bridge iniciado. Esperando publicación de ROS de la Laptop para definir el modo...")

    def cambiar_modo_unica_vez(self, nuevo_modo):
        # Esta función sella el modo en cuanto la laptop envía el primer paquete.
        if self.modo_operacion is None and self.ser is not None:
            self.modo_operacion = nuevo_modo
            comando_inicio = f"MODE,{self.modo_operacion}\n"
            self.ser.write(comando_inicio.encode('utf-8'))
            self.get_logger().info(f"Modo estatico bloqueado en: {self.modo_operacion} (Dictado por Laptop)")
            time.sleep(0.05) # Pausa obligatoria para que el OpenCR procese el cambio

    def callback_sys(self, msg):
        if msg.data == "FINISH":
            self.fin_intencional = True
            self.razon_fin = "Trayectoria finalizada correctamente."
        elif msg.data == "MANUAL_STOP":
            self.fin_intencional = True
            self.razon_fin = "Parada manual por el usuario."

    def callback_cmd_vel(self, msg):
        # Si es el primer mensaje de la historia, ancla el modo a Velocidad
        if self.modo_operacion is None:
            self.cambiar_modo_unica_vez('VEL')
            
        # Filtro de seguridad: Solo procesa si el modo anclado coincide
        if self.modo_operacion == 'VEL':
            self.val1_cmd = msg.linear.x
            self.val2_cmd = msg.angular.z
            self.registrar_actividad()

    def callback_cmd_current(self, msg):
        # Si es el primer mensaje de la historia, ancla el modo a Corrientes
        if self.modo_operacion is None:
            self.cambiar_modo_unica_vez('CUR')
            
        # Filtro de seguridad: Solo procesa si el modo anclado coincide
        if self.modo_operacion == 'CUR' and len(msg.data) >= 2:
            K_t = 1.05
            mA_por_unidad = 2.69 
            corriente_izq = (msg.data[0] / K_t) * 1000.0 / mA_por_unidad
            corriente_der = (msg.data[1] / K_t) * 1000.0 / mA_por_unidad
            self.val1_cmd = float(clip(corriente_izq, -300, 300))
            self.val2_cmd = float(clip(corriente_der, -300, 300))
            self.registrar_actividad()

    def registrar_actividad(self):
        self.ultimo_tiempo_recibido = time.time()
        self.ha_conectado_alguna_vez = True
        if not self.conexion_establecida:
            self.conexion_establecida = True
            self.get_logger().info("Control ZNN de Laptop enlazado. Watchdog activado.")

    def ciclo_comunicacion(self):
        if self.ser is None or not self.ser.is_open:
            return

        tiempo_actual = time.time()
        factor_rampa = 1.0 

        # Si aún no recibimos nada de la laptop, enviamos ceros para mantener la telemetría viva
        if not self.ha_conectado_alguna_vez:
            cadena_envio = "0.0000,0.0000\n"
            self.ser.write(cadena_envio.encode('utf-8'))
        else:
            tiempo_sin_datos = tiempo_actual - self.ultimo_tiempo_recibido
            
            if tiempo_sin_datos > self.limite_desconexion_total:
                if self.fin_intencional:
                    self.get_logger().info(f"Cierre seguro: {self.razon_fin} Cerrando nodo.")
                else:
                    self.get_logger().error("Watchdog Critico: 5 segundos sin conexion. Terminando ejecucion.")
                raise SystemExit
            
            elif tiempo_sin_datos > self.limite_watchdog:
                tiempo_en_rampa = tiempo_sin_datos - self.limite_watchdog
                duracion_rampa = 0.5 
                
                if tiempo_en_rampa >= duracion_rampa:
                    factor_rampa = 0.0 
                else:
                    factor_rampa = 1.0 - (tiempo_en_rampa / duracion_rampa)
                
                if self.conexion_establecida and not self.fin_intencional and factor_rampa == 0.0:
                    self.get_logger().warn("Watchdog: Frenado suave completado por perdida de señal.", throttle_duration_sec=2.0)
                    self.conexion_establecida = False

            envio_val1 = self.val1_cmd * factor_rampa
            envio_val2 = self.val2_cmd * factor_rampa

            cadena_envio = f"{envio_val1:.4f},{envio_val2:.4f}\n"
            self.ser.write(cadena_envio.encode('utf-8'))

        # LECTURA DE TELEMETRÍA (Siempre activa)
        while self.ser.in_waiting > 0:
            try:
                linea = self.ser.readline().decode('utf-8').strip()
                if linea.startswith("OK:"):
                    continue
                partes = linea.split(',')
                if len(partes) == 5:
                    msg_odom = Pose2D()
                    msg_odom.x = float(partes[0])
                    msg_odom.y = float(partes[1])
                    msg_odom.theta = float(partes[2])
                    self.pub_odom.publish(msg_odom)

                    msg_vel = Twist()
                    msg_vel.linear.x = float(partes[3])
                    msg_vel.angular.z = float(partes[4])
                    self.pub_vel_real.publish(msg_vel)
            except (ValueError, UnicodeDecodeError):
                pass  

def main(args=None):
    rclpy.init(args=args)
    nodo = BridgeRaspberryOpenCR()
    try:
        rclpy.spin(nodo)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if nodo.ser and nodo.ser.is_open:
            nodo.ser.write(b"0.0000,0.0000\n")
            nodo.ser.close()
        nodo.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()