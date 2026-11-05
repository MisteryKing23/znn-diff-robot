#include <DynamixelSDK.h>
#include <IMU.h>
#include <math.h>

const uint8_t leftID = 1;
const uint8_t rightID = 2;

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// --- DIRECCIONES DYNAMIXEL ---
constexpr uint16_t TorqueEnableAddr = 64;
constexpr uint16_t OperatingModeAddr = 11;
constexpr uint16_t GoalCurAddr = 102; 
constexpr uint16_t GoalVelAddr = 104; 
constexpr uint16_t PresentVelAddr = 128;
constexpr uint16_t PresentPosAddr = 132;

cIMU imu; 

// Variables globales de odometría y calibración
float pos_x = 0.0;
float pos_y = 0.0;
int32_t last_tick_left = 0;
int32_t last_tick_right = 0;
bool first_read = true; 
float initial_yaw_offset = 0.0;

// Constantes físicas del robot
const float wheel_radius = 0.033; 
const float wheel_separation = 0.287;
const float ticks_per_rev = 4096.0; 

// --- ESTRUCTURA DE CONTROL DUAL ---
enum ControlModes : uint8_t {
  CUR_CTRL = 0,
  VEL_CTRL = 1
};
ControlModes current_mode = VEL_CTRL; 

void setControlMode(ControlModes new_mode) {
  if (current_mode == new_mode) return;

  uint8_t dxl_error = 0;
  packetHandler->write1ByteTxRx(portHandler, leftID, TorqueEnableAddr, 0, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, rightID, TorqueEnableAddr, 0, &dxl_error);

  packetHandler->write1ByteTxRx(portHandler, leftID, OperatingModeAddr, new_mode, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, rightID, OperatingModeAddr, new_mode, &dxl_error);

  packetHandler->write1ByteTxRx(portHandler, leftID, TorqueEnableAddr, 1, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, rightID, TorqueEnableAddr, 1, &dxl_error);

  current_mode = new_mode;
}

void setup() {
  Serial.begin(3000000);  //3Mbps
  imu.begin();     
  portHandler = dynamixel::PortHandler::getPortHandler("OpenCR_DXL_Port");
  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
  portHandler->openPort();  
  
  uint8_t dxl_error = 0;
  packetHandler->write1ByteTxRx(portHandler, leftID, 8, 5, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, rightID, 8, 5, &dxl_error);
  portHandler->setBaudRate(3000000);

  packetHandler->write1ByteTxRx(portHandler, leftID, TorqueEnableAddr, 0, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, rightID, TorqueEnableAddr, 0, &dxl_error);  
  packetHandler->write1ByteTxRx(portHandler, leftID, OperatingModeAddr, VEL_CTRL, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, rightID, OperatingModeAddr, VEL_CTRL, &dxl_error);  
  packetHandler->write1ByteTxRx(portHandler, leftID, TorqueEnableAddr, 1, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, rightID, TorqueEnableAddr, 1, &dxl_error);
}

void loop() {
  imu.update(); 

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); 

    if (data.equals("RESET")) {
      pos_x = 0.0;
      pos_y = 0.0;
      first_read = true;
       Serial.println("0.0000,0.0000,0.0000,0.0000,0.0000"); // NUEVO: 5 valores
       return;  
     }

    if (data.startsWith("MODE,")) {
      String mode_str = data.substring(5);
      if (mode_str.equals("VEL")) {
        setControlMode(VEL_CTRL);
        Serial.println("OK: MODO_VELOCIDAD");
      } else if (mode_str.equals("CUR")) {
        setControlMode(CUR_CTRL);
        Serial.println("OK: MODO_CORRIENTE");
      }
      return;  
     }

    int commaIndex = data.indexOf(',');      
    if (commaIndex > 0) {
      float val1 = data.substring(0, commaIndex).toFloat();
      float val2 = data.substring(commaIndex + 1).toFloat();      
      uint8_t dxl_error = 0;

      if (current_mode == VEL_CTRL) {
        float left_wheel_vel = (val1 - (val2 * wheel_separation / 2.0)) / wheel_radius;
        float right_wheel_vel = (val1 + (val2 * wheel_separation / 2.0)) / wheel_radius;

        int32_t goal_velocity_left = (int32_t)(left_wheel_vel * 41.69);
        int32_t goal_velocity_right = (int32_t)(right_wheel_vel * 41.69);

        packetHandler->write4ByteTxRx(portHandler, leftID, GoalVelAddr, (uint32_t)goal_velocity_left, &dxl_error);
        packetHandler->write4ByteTxRx(portHandler, rightID, GoalVelAddr, (uint32_t)goal_velocity_right, &dxl_error);      
      } else if (current_mode == CUR_CTRL) {
        int16_t goal_current_left = (int16_t)val1;
        int16_t goal_current_right = (int16_t)val2;        
        packetHandler->write2ByteTxRx(portHandler, leftID, GoalCurAddr, (uint16_t)goal_current_left, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, rightID, GoalCurAddr, (uint16_t)goal_current_right, &dxl_error);
      }

      // --- ODOMETRÍA Y LECTURA DE VELOCIDAD REAL ---
      int32_t current_tick_left = 0;
      int32_t current_tick_right = 0;
      int32_t current_vel_left = 0; // NUEVO
      int32_t current_vel_right = 0; // NUEVO
      
      packetHandler->read4ByteTxRx(portHandler, leftID, PresentPosAddr, (uint32_t*)&current_tick_left, &dxl_error);
      packetHandler->read4ByteTxRx(portHandler, rightID, PresentPosAddr, (uint32_t*)&current_tick_right, &dxl_error);
      // NUEVO: Leer registros de velocidad (dirección 128)
      packetHandler->read4ByteTxRx(portHandler, leftID, PresentVelAddr, (uint32_t*)&current_vel_left, &dxl_error);
      packetHandler->read4ByteTxRx(portHandler, rightID, PresentVelAddr, (uint32_t*)&current_vel_right, &dxl_error);

      if (first_read) {
        last_tick_left = current_tick_left;
        last_tick_right = current_tick_right;
        initial_yaw_offset = imu.rpy[2] * (PI / 180.0);
        first_read = false;
      }

      int32_t delta_tick_left = current_tick_left - last_tick_left;
      int32_t delta_tick_right = current_tick_right - last_tick_right;

      last_tick_left = current_tick_left;
      last_tick_right = current_tick_right;

      float dist_left = ((float)delta_tick_left / ticks_per_rev) * (2.0 * PI * wheel_radius);
      float dist_right = ((float)delta_tick_right / ticks_per_rev) * (2.0 * PI * wheel_radius);

      float delta_s = (dist_right + dist_left) / 2.0;

      float raw_theta = imu.rpy[2] * (PI / 180.0);
      float theta = raw_theta - initial_yaw_offset; 

      pos_x += delta_s * cos(theta);
      pos_y += delta_s * sin(theta);

      // NUEVO: Cinemática directa para calcular velocidad real
      float v_wheel_left = ((float)current_vel_left / 41.69) * wheel_radius;
      float v_wheel_right = ((float)current_vel_right / 41.69) * wheel_radius;
      float v_real = (v_wheel_right + v_wheel_left) / 2.0;
      float w_real = (v_wheel_right - v_wheel_left) / wheel_separation;

      // ENVIAR PAQUETE EXTENDIDO (5 Valores)
      Serial.print(pos_x, 4);
      Serial.print(",");
      Serial.print(pos_y, 4);
      Serial.print(",");
      Serial.print(theta, 4);
      Serial.print(",");
      Serial.print(v_real, 4);
      Serial.print(",");
      Serial.println(w_real, 4);
    } 
  }
}
