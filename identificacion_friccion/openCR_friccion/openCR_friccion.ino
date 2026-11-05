/*
 * openCR_friccion.ino
 * Firmware de PRUEBA para identificar la fricción viscosa (d) y de Coulomb (c) de cada rueda.
 *
 * Idea: se sostiene cada rueda a velocidad constante con un PI de velocidad que corre AQUÍ
 * (lazo determinista a 500 Hz, en modo corriente). En régimen permanente la aceleración es
 * cero, así que la corriente promedio que hace falta es exactamente la fricción:
 *     ktau * i_ss = d * eta + c * sgn(eta)
 * El PI "tiembla" entre unidades vecinas de corriente; al promediar se obtiene una resolución
 * mucho más fina que 1 unidad (2.69 mA), cosa que un escalón de corriente fija no permite.
 *
 * Comandos por USB (una línea, '\n'):
 *   "C,<izq>,<der>"      corriente objetivo en UNIDADES Dynamixel (1 u = 2.69 mA)  -> modo CORRIENTE
 *   "V,<izq>,<der>"      velocidad objetivo en rad/s (con el signo físico del motor) -> modo VELOCIDAD
 *   "G,<kp>,<ki>"        ganancias del PI: kp [u/(rad/s)], ki [u/(rad/s)/s]
 *   "A,<acel>"           rampa de la referencia de velocidad [rad/s^2] (0 = sin rampa)
 *   "TEL,EXT" | "TEL,STD"  telemetría extendida (promedios) o la estándar de siempre
 *   "STOP" | "RESET"     corriente 0 y modo corriente
 *   "MODE,CUR"           (re)fuerza modo corriente en los XM430
 *   "DIAG" | "REBOOT"    igual que openCR_caracterizacion.ino
 *
 * Telemetría cada 10 ms:
 *   STD: "T,<t_us>,<goal_izq>,<goal_der>,<cur_izq>,<cur_der>,<vel_izq>,<vel_der>,<pos_izq>,<pos_der>"
 *        (idéntica a openCR_caracterizacion.ino, valores instantáneos)
 *   EXT: "X,<t_us>,<modo>,<ref_izq>,<ref_der>,<goal_izq>,<goal_der>,<cur_izq>,<cur_der>,
 *          <vel_izq>,<vel_der>,<pos_izq>,<pos_der>,<n>,<lat_us>"
 *        modo 0 = corriente, 1 = velocidad; ref en rad/s (la de la rampa);
 *        goal/cur/vel = PROMEDIO de los <n> ciclos de control de la ventana, en unidades crudas
 *        (goal y cur en 2.69 mA/u, vel en 0.229 rpm/u); pos = instantánea (ticks);
 *        lat_us = duración máxima de un ciclo de control en la ventana (lectura+cálculo+escritura).
 *   Todo con el signo físico de cada motor (la Raspberry corrige el sentido).
 *
 * Seguridad: corriente limitada a +/-LIMITE_CORRIENTE, velocidad a +/-LIMITE_VEL y todo a 0 si pasan
 * WATCHDOG_MS sin recibir un comando "C,..." o "V,...".
 */

#include <DynamixelSDK.h>
#include <stdlib.h>
#include <algorithm>

const uint8_t leftID = 1;
const uint8_t rightID = 2;

// --- DIRECCIONES XM430 (Protocolo 2.0) ---
constexpr uint16_t ADDR_BAUD_RATE        = 8;
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_GOAL_CURRENT     = 102;  // 2 bytes, con signo
constexpr uint16_t ADDR_PRESENT_CURRENT  = 126;  // 2 bytes, con signo
constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;  // 4 bytes, con signo
constexpr uint16_t ADDR_PRESENT_POSITION = 132;  // 4 bytes, con signo
constexpr uint16_t LEN_BLOQUE_ESTADO     = 10;   // 126..135 en una sola lectura

constexpr uint8_t MODO_CORRIENTE = 0;

constexpr float RADPS_POR_UNIDAD = 0.229f * 2.0f * PI / 60.0f;  // 0.02398 rad/s por unidad de velocidad

// --- PARÁMETROS DE LA PRUEBA ---
const int16_t  LIMITE_CORRIENTE = 400;     // unidades (~1.08 A)
const float    LIMITE_VEL       = 8.0f;    // rad/s (XM430-W210 en vacío ~8 rad/s a 12 V)
const uint32_t PERIODO_CTRL_US  = 2000;    // 500 Hz de control
const uint8_t  CICLOS_POR_TEL   = 5;       // 5 x 2 ms = telemetría cada 10 ms
const uint32_t WATCHDOG_MS      = 300;

// Ganancias por defecto del PI (en unidades de corriente). Se cambian con "G,kp,ki".
float kp = 8.0f;     // u / (rad/s)      -> 8 u = 21.5 mA por cada rad/s de error
float ki = 60.0f;    // u / (rad/s) / s
float acel = 4.0f;   // rad/s^2 de la rampa de referencia
float tau_filt = 0.0f;  // s, constante de tiempo del EMA de velocidad (0 = sin filtro)

enum Modo : uint8_t { M_CORRIENTE = 0, M_VELOCIDAD = 1 };

dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;
dynamixel::GroupSyncWrite *syncWriteCur;
dynamixel::GroupSyncRead  *syncReadEstado;

struct Rueda {
  int16_t goal = 0;        // corriente que se manda [u]
  float   ref_obj = 0.0f;  // referencia pedida [rad/s]
  float   ref = 0.0f;      // referencia tras la rampa [rad/s]
  float   integ = 0.0f;    // integrador del PI [u]
  float   vel_filt = 0.0f; // velocidad filtrada [rad/s] (EMA)
  int16_t cur = 0;         // última lectura
  int32_t vel = 0, pos = 0;
  // acumuladores de la ventana de telemetría
  float s_goal = 0, s_cur = 0, s_vel = 0;
};

Rueda izq, der;
Modo modo = M_CORRIENTE;
bool tel_ext = false;
uint32_t ultimo_cmd_ms = 0;
uint32_t ultimo_ciclo_us = 0;
uint8_t  ciclos = 0;
uint32_t lat_max_us = 0;

char    buf_rx[64];
uint8_t idx_rx = 0;

// ---------------------------------------------------------------
int16_t limitar(long v) {
  if (v >  LIMITE_CORRIENTE) return  LIMITE_CORRIENTE;
  if (v < -LIMITE_CORRIENTE) return -LIMITE_CORRIENTE;
  return (int16_t)v;
}

float limitarf(float v, float lim) {
  if (v >  lim) return  lim;
  if (v < -lim) return -lim;
  return v;
}

void escribir1(uint8_t id, uint16_t addr, uint8_t valor) {
  uint8_t dxl_error = 0;
  packetHandler->write1ByteTxRx(portHandler, id, addr, valor, &dxl_error);
}

void reiniciarPI() {
  izq.integ = der.integ = 0.0f;
  izq.ref = der.ref = 0.0f;
  izq.ref_obj = der.ref_obj = 0.0f;
}

void modoCorrienteCero() {
  modo = M_CORRIENTE;
  izq.goal = der.goal = 0;
  reiniciarPI();
}

void forzarModoCorriente() {
  escribir1(leftID,  ADDR_TORQUE_ENABLE, 0);
  escribir1(rightID, ADDR_TORQUE_ENABLE, 0);
  escribir1(leftID,  ADDR_OPERATING_MODE, MODO_CORRIENTE);
  escribir1(rightID, ADDR_OPERATING_MODE, MODO_CORRIENTE);
  escribir1(leftID,  ADDR_TORQUE_ENABLE, 1);
  escribir1(rightID, ADDR_TORQUE_ENABLE, 1);
  modoCorrienteCero();
}

bool escribirCorrientes(int16_t i_izq, int16_t i_der) {
  uint8_t p_izq[2] = { DXL_LOBYTE((uint16_t)i_izq), DXL_HIBYTE((uint16_t)i_izq) };
  uint8_t p_der[2] = { DXL_LOBYTE((uint16_t)i_der), DXL_HIBYTE((uint16_t)i_der) };
  syncWriteCur->addParam(leftID,  p_izq);
  syncWriteCur->addParam(rightID, p_der);
  int res = syncWriteCur->txPacket();
  syncWriteCur->clearParam();
  return res == COMM_SUCCESS;
}

bool leerEstado() {
  if (syncReadEstado->txRxPacket() != COMM_SUCCESS) return false;
  if (!syncReadEstado->isAvailable(leftID,  ADDR_PRESENT_CURRENT, LEN_BLOQUE_ESTADO) ||
      !syncReadEstado->isAvailable(rightID, ADDR_PRESENT_CURRENT, LEN_BLOQUE_ESTADO)) return false;
  izq.cur = (int16_t)syncReadEstado->getData(leftID,  ADDR_PRESENT_CURRENT, 2);
  der.cur = (int16_t)syncReadEstado->getData(rightID, ADDR_PRESENT_CURRENT, 2);
  izq.vel = (int32_t)syncReadEstado->getData(leftID,  ADDR_PRESENT_VELOCITY, 4);
  der.vel = (int32_t)syncReadEstado->getData(rightID, ADDR_PRESENT_VELOCITY, 4);
  izq.pos = (int32_t)syncReadEstado->getData(leftID,  ADDR_PRESENT_POSITION, 4);
  der.pos = (int32_t)syncReadEstado->getData(rightID, ADDR_PRESENT_POSITION, 4);
  return true;
}

// PI de velocidad con rampa de referencia y anti-windup por integración condicional.
void controlVelocidad(Rueda &r, float dt) {
  float paso = acel * dt;
  if (acel <= 0.0f) r.ref = r.ref_obj;
  else if (r.ref < r.ref_obj) r.ref = std::min(r.ref + paso, r.ref_obj);
  else if (r.ref > r.ref_obj) r.ref = std::max(r.ref - paso, r.ref_obj);

  float e = r.ref - r.vel_filt;
  float integ_nuevo = limitarf(r.integ + ki * e * dt, LIMITE_CORRIENTE);
  float u = kp * e + integ_nuevo;
  if (u > LIMITE_CORRIENTE || u < -LIMITE_CORRIENTE) {
    // saturado: solo se integra si el error ayuda a salir de la saturación
    if ((u > 0) != (e > 0)) r.integ = integ_nuevo;
  } else {
    r.integ = integ_nuevo;
  }
  float out = kp * e + r.integ;
  r.goal = limitar((long)(out + (out >= 0 ? 0.5f : -0.5f)));
}

// Diagnóstico: comunicación, modelo, modo, torque, límite, errores y voltaje.
void diagnostico() {
  uint8_t ids[2] = { leftID, rightID };
  for (int k = 0; k < 2; k++) {
    uint8_t id = ids[k], err = 0, modo_dxl = 255, torque = 255, hw = 255;
    uint16_t modelo = 0, lim = 0, volt = 0;
    int com = packetHandler->read2ByteTxRx(portHandler, id, 0, &modelo, &err);   // Model Number
    packetHandler->read1ByteTxRx(portHandler, id, 11, &modo_dxl, &err);          // Operating Mode
    packetHandler->read1ByteTxRx(portHandler, id, 64, &torque, &err);            // Torque Enable
    packetHandler->read2ByteTxRx(portHandler, id, 38, &lim, &err);               // Current Limit
    packetHandler->read1ByteTxRx(portHandler, id, 70, &hw, &err);                // Hardware Error Status
    packetHandler->read2ByteTxRx(portHandler, id, 144, &volt, &err);             // Present Input Voltage (0.1 V)
    char tx[160];
    snprintf(tx, sizeof(tx),
             "DIAG,id=%d,com=%s,modelo=%u,modo=%u,torque=%u,cur_limit=%u,hw_error=%u,volt=%.1f",
             id, com == COMM_SUCCESS ? "OK" : "FALLA", modelo, modo_dxl, torque, lim, hw, volt / 10.0);
    Serial.println(tx);
  }
  char tx[96];
  snprintf(tx, sizeof(tx), "DIAG,pi,kp_x100=%ld,ki_x100=%ld,acel_x100=%ld,tau_ms=%ld,periodo_us=%lu",
           (long)(kp * 100), (long)(ki * 100), (long)(acel * 100), (long)(tau_filt * 1000), (unsigned long)PERIODO_CTRL_US);
  Serial.println(tx);
}

void reiniciarMotores() {
  uint8_t err = 0;
  packetHandler->reboot(portHandler, leftID, &err);
  packetHandler->reboot(portHandler, rightID, &err);
  delay(500);
  forzarModoCorriente();
}

// Lee dos números separados por coma después del prefijo "X,". Devuelve false si no hay dos.
bool dosNumeros(char *p, float &a, float &b) {
  char *fin;
  a = strtod(p, &fin);
  if (fin == p || *fin != ',') return false;
  p = fin + 1;
  b = strtod(p, &fin);
  return fin != p;
}

void procesarLinea(char *linea) {
  if (strcmp(linea, "DIAG") == 0) {
    diagnostico();
    return;
  }
  if (strcmp(linea, "REBOOT") == 0) {
    reiniciarMotores();
    Serial.println("OK:REBOOT");
    diagnostico();
    return;
  }
  float a, b;
  if (linea[0] == 'C' && linea[1] == ',') {
    char *p = linea + 2;
    long ci = strtol(p, &p, 10);
    if (*p == ',') {
      long cd = strtol(p + 1, NULL, 10);
      if (modo != M_CORRIENTE) reiniciarPI();
      modo = M_CORRIENTE;
      izq.goal = limitar(ci);
      der.goal = limitar(cd);
      ultimo_cmd_ms = millis();
    }
  } else if (linea[0] == 'V' && linea[1] == ',') {
    if (dosNumeros(linea + 2, a, b)) {
      if (modo != M_VELOCIDAD) {
        // arranque sin salto: la rampa parte de la velocidad medida
        reiniciarPI();
        izq.vel_filt = izq.vel * RADPS_POR_UNIDAD;
        der.vel_filt = der.vel * RADPS_POR_UNIDAD;
        izq.ref = izq.vel_filt;
        der.ref = der.vel_filt;
      }
      modo = M_VELOCIDAD;
      izq.ref_obj = limitarf(a, LIMITE_VEL);
      der.ref_obj = limitarf(b, LIMITE_VEL);
      ultimo_cmd_ms = millis();
    }
  } else if (linea[0] == 'G' && linea[1] == ',') {
    if (dosNumeros(linea + 2, a, b) && a >= 0 && b >= 0) {
      kp = a;
      ki = b;
      char gx[48];
      snprintf(gx, sizeof(gx), "OK:G,%ld,%ld", (long)(kp * 100), (long)(ki * 100));
      Serial.println(gx);
    } else {
      Serial.println("ERR:G");
    }
  } else if (linea[0] == 'F' && linea[1] == ',') {
    tau_filt = std::max(0.0f, (float)strtod(linea + 2, NULL));
    char fx[48];
    snprintf(fx, sizeof(fx), "OK:F,%ld", (long)(tau_filt * 1000));
    Serial.println(fx);
  } else if (linea[0] == 'A' && linea[1] == ',') {
    acel = std::max(0.0f, (float)strtod(linea + 2, NULL));
    Serial.println("OK:A");
  } else if (strcmp(linea, "TEL,EXT") == 0) {
    tel_ext = true;
    Serial.println("OK:TEL,EXT");
  } else if (strcmp(linea, "TEL,STD") == 0) {
    tel_ext = false;
    Serial.println("OK:TEL,STD");
  } else if (strcmp(linea, "STOP") == 0 || strcmp(linea, "RESET") == 0) {
    modoCorrienteCero();
    Serial.println("OK:STOP");
  } else if (strcmp(linea, "MODE,CUR") == 0) {
    forzarModoCorriente();
    Serial.println("OK:MODE,CUR");
  } else if (strncmp(linea, "MODE,", 5) == 0) {
    Serial.println("ERR:este firmware solo usa MODE,CUR");
  }
}

void leerSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf_rx[idx_rx] = '\0';
      if (idx_rx > 0) procesarLinea(buf_rx);
      idx_rx = 0;
    } else if (idx_rx < sizeof(buf_rx) - 1) {
      buf_rx[idx_rx++] = c;
    } else {
      idx_rx = 0;  // línea demasiado larga: descartar
    }
  }
}

void acumular(Rueda &r) {
  r.s_goal += r.goal;
  r.s_cur  += r.cur;
  r.s_vel  += r.vel;
}

void enviarTelemetria(uint32_t t_us) {
  if (tel_ext) {
    float n = ciclos > 0 ? ciclos : 1;
    String s = "X," + String(t_us) + "," + String((int)modo) + "," +
               String(izq.ref, 4) + "," + String(der.ref, 4) + "," +
               String(izq.s_goal / n, 3) + "," + String(der.s_goal / n, 3) + "," +
               String(izq.s_cur / n, 3) + "," + String(der.s_cur / n, 3) + "," +
               String(izq.s_vel / n, 3) + "," + String(der.s_vel / n, 3) + "," +
               String(izq.pos) + "," + String(der.pos) + "," +
               String((int)ciclos) + "," + String(lat_max_us);
    Serial.println(s);
  } else {
    char tx[120];
    snprintf(tx, sizeof(tx), "T,%lu,%d,%d,%d,%d,%ld,%ld,%ld,%ld",
             (unsigned long)t_us, izq.goal, der.goal, izq.cur, der.cur,
             (long)izq.vel, (long)der.vel, (long)izq.pos, (long)der.pos);
    Serial.println(tx);
  }
  izq.s_goal = izq.s_cur = izq.s_vel = 0;
  der.s_goal = der.s_cur = der.s_vel = 0;
  ciclos = 0;
  lat_max_us = 0;
}
// ---------------------------------------------------------------
void setup() {
  Serial.begin(3000000);

  portHandler   = dynamixel::PortHandler::getPortHandler("OpenCR_DXL_Port");
  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
  portHandler->openPort();

  // Igual que el firmware actual: pedir 3 Mbps (si ya están a 3M, esta escritura falla sin efecto)
  escribir1(leftID,  ADDR_BAUD_RATE, 5);
  escribir1(rightID, ADDR_BAUD_RATE, 5);
  portHandler->setBaudRate(3000000);

  syncWriteCur   = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, 2);
  syncReadEstado = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LEN_BLOQUE_ESTADO);
  syncReadEstado->addParam(leftID);
  syncReadEstado->addParam(rightID);

  forzarModoCorriente();
  escribirCorrientes(0, 0);
  Serial.println("OK:READY");
}

void loop() {
  leerSerial();

  uint32_t ahora_us = micros();
  uint32_t dt_us = ahora_us - ultimo_ciclo_us;
  if (dt_us < PERIODO_CTRL_US) return;
  ultimo_ciclo_us = ahora_us;

  // Watchdog
  if ((uint32_t)(millis() - ultimo_cmd_ms) > WATCHDOG_MS && (modo != M_CORRIENTE || izq.goal || der.goal))
    modoCorrienteCero();

  // 1) leer  2) calcular  3) escribir: en modo velocidad el PI usa la medición de este ciclo
  if (!leerEstado()) {
    Serial.println("ERR:SYNCREAD");
    escribirCorrientes(izq.goal, der.goal);
    return;
  }
  // EMA de velocidad (si tau_filt > 0)
  if (tau_filt > 0.0f) {
    float alpha = dt_us * 1e-6f / tau_filt;
    if (alpha > 1.0f) alpha = 1.0f;
    izq.vel_filt += alpha * (izq.vel * RADPS_POR_UNIDAD - izq.vel_filt);
    der.vel_filt += alpha * (der.vel * RADPS_POR_UNIDAD - der.vel_filt);
  } else {
    izq.vel_filt = izq.vel * RADPS_POR_UNIDAD;
    der.vel_filt = der.vel * RADPS_POR_UNIDAD;
  }
  if (modo == M_VELOCIDAD) {
    float dt = std::min(dt_us, 3 * PERIODO_CTRL_US) * 1e-6f;   // acota dt tras una pausa larga
    controlVelocidad(izq, dt);
    controlVelocidad(der, dt);
  }
  escribirCorrientes(izq.goal, der.goal);

  uint32_t lat = micros() - ahora_us;
  if (lat > lat_max_us) lat_max_us = lat;
  acumular(izq);
  acumular(der);
  if (++ciclos >= CICLOS_POR_TEL) enviarTelemetria(ahora_us);
}
