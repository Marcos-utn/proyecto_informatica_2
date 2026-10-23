/*
 * MONITOR DE VIBRACIONES PARA MANTENIMIENTO PREDICTIVO DE MOTORES ELECTRICOS
 * Placa:   Arduino UNO R4 WiFi
 * Sensor:  MPU-6050 (GY-521) por I2C
 *
 * Funcionamiento
 *  - Muestrea la aceleracion a 1 kHz en ventanas de 1024 muestras (~1,02 s).
 *  - Calcula por eje: velocidad RMS (mm/s, banda 10-250 Hz, estilo ISO 10816/20816)
 *    mediante FFT con ventana Hann e integracion en frecuencia.
 *  - Del eje dominante: aceleracion RMS, valor pico, factor de cresta y
 *    frecuencia dominante, ademas del espectro de velocidad.
 *  - Estados: 0 NORMAL (verde), 1 ALERTA (amarillo), 2 ALARMA (rojo + zumbador),
 *    con histeresis y numero minimo de lecturas consecutivas.
 *  - Funciona de forma autonoma (LEDs/zumbador/rele) y se controla desde Processing.
 *  - Maniobra del motor: un modulo de rele (activo en BAJO) comanda la bobina de un
 *    contactor de 4 polos (3F+N). Arranque solo por orden expresa, disparo automatico
 *    opcional por ALARMA con ENCLAVAMIENTO (no rearranca solo) y arranque siempre en OFF.
 *
 * Protocolo serie (115200 baud, lineas ASCII terminadas en \n)
 *  Arduino -> PC
 *    D,ms,estado,vmax,vx,vy,vz,aRMS_g,aPico_g,cresta,fdom_Hz,acked,errores
 *    S,binHz,n,v0,v1,...        (espectro de velocidad mm/s RMS, bins 0..n-1)
 *    C,alerta,alarma,lecturas,running,disparoAuto,acked,ruidoX,ruidoY,ruidoZ,motor,disparado
 *    I,texto    (informacion)      E,texto   (error)
 *  PC -> Arduino
 *    START | STOP | GET | ACK | RESET | ZERO | ZEROCLR
 *    TH,alerta,alarma,lecturas      RELAY,0|1 (disparo automatico)      MOTOR,0|1
 */

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <ctype.h>

// ======================= PINES =======================
const uint8_t PIN_LED_VERDE = 2;
const uint8_t PIN_LED_AMAR  = 3;
const uint8_t PIN_LED_ROJO  = 4;
const uint8_t PIN_BUZZER    = 5;   // a la base de un NPN (via 2,2 kOhm) o al pin IO de un modulo buzzer activo
const uint8_t PIN_BTN_ACK   = 6;   // pulsador a GND (INPUT_PULLUP)
// Modulo de rele de 4 canales con optoacopladores: se ACTIVA con 0 V (nivel BAJO).
const uint8_t PINES_RELE[4] = {7, 8, 9, 10};   // IN1..IN4
const uint8_t N_RELES = 1;     // 1 = IN1 comanda la bobina de un contactor (RECOMENDADO)
                               // 4 = los 4 canales conmutan a la vez (solo para cargas dentro de la capacidad del rele)
const bool    RELE_ACTIVO_ALTO = false;
const uint32_t GRACIA_ARRANQUE_MS = 15000;   // tras arrancar no se dispara por vibracion (transitorio de arranque)

// ======================= SENSOR ======================
const uint8_t MPU_ADDR = 0x68;                 // AD0 a GND (0x69 si AD0 a 3,3 V)
const float   ACC_SCALE = 9.80665f / 8192.0f;  // m/s2 por LSB (rango +-4 g)
const float   G_PER_LSB = 1.0f / 8192.0f;      // g por LSB

// ======================= ANALISIS ====================
constexpr uint16_t N        = 1024;            // potencia de 2 (si falta RAM: 512)
constexpr uint32_t FS_HZ    = 1000;            // frecuencia de muestreo
constexpr uint32_t F_LO_HZ  = 10;              // banda de velocidad ISO: 10 Hz ...
constexpr uint32_t F_HI_HZ  = 250;             // ... hasta el limite del filtro del MPU6050
constexpr uint16_t K_LO     = (F_LO_HZ * N + FS_HZ - 1) / FS_HZ;  // primer bin de la banda
constexpr uint16_t K_HI     = (F_HI_HZ * N) / FS_HZ;              // ultimo bin de la banda
constexpr float    BIN_HZ   = (float)FS_HZ / N;
const uint32_t PERIODO_US   = 1000000UL / FS_HZ;
const float SUM_W  = N * 0.5f;                 // suma de la ventana Hann
const float SUM_W2 = N * 0.375f;               // suma de la ventana Hann al cuadrado
const float HIST   = 0.10f;                    // histeresis 10 %

// ======================= CONFIGURACION (EEPROM) ======
struct Config {
  uint32_t magic;
  float    warn;      // umbral de alerta  (mm/s)
  float    alarm;     // umbral de alarma  (mm/s)
  uint8_t  cnt;       // lecturas consecutivas
  uint8_t  relay;     // rele habilitado
  float    noise[3];  // ruido propio del sensor por eje (mm/s)
};
const uint32_t CFG_MAGIC = 0x56494233;
Config cfg;

// ======================= VARIABLES ===================
int16_t raw[3][N];                  // 6 KB
float   re[N], im[N];               // 8 KB
float   cosT[N / 2 + 1];            // 2 KB: tabla de coseno para la FFT, el seno y la ventana Hann
float   spec[K_HI + 1], specTmp[K_HI + 1];

float vAxis[3], vRawAxis[3];
float vMax = 0, aRms = 0, aPeak = 0, crest = 0, fDom = 0;

bool     running = true;
bool     sensorFault = false;
bool     acked = false;
bool     zeroReq = false;
bool     motorOn = false;     // contactor conectado
bool     tripped = false;     // disparado por vibracion (enclavado hasta RESET)
uint32_t motorOnAt = 0;
uint8_t  state = 0, prevState = 0, upCnt = 0, dnCnt = 0;
uint16_t winErrors = 0;
uint8_t  windowsSinceCfg = 0;

char    lineBuf[48];
uint8_t lineLen = 0;

// ======================= I2C / MPU ===================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

bool initMPU() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x01);   // sale de reposo, reloj PLL
  delay(50);
  mpuWrite(0x1A, 0x00);   // DLPF desactivado -> ancho de banda del acelerometro 260 Hz
  mpuWrite(0x19, 0x07);   // 8 kHz / (1+7) = 1 kHz
  mpuWrite(0x1C, 0x08);   // rango +-4 g
  delay(100);
  return true;
}

bool readAccel(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6) return false;
  x = (int16_t)((Wire.read() << 8) | Wire.read());
  y = (int16_t)((Wire.read() << 8) | Wire.read());
  z = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

// ======================= CONFIG ======================
void defaultConfig() {
  cfg.magic = CFG_MAGIC;
  cfg.warn  = 1.8f;      // ISO 10816-1 clase I (hasta 15 kW): limite B/C
  cfg.alarm = 4.5f;      // limite C/D
  cfg.cnt   = 3;
  cfg.relay = 0;
  cfg.noise[0] = cfg.noise[1] = cfg.noise[2] = 0;
}

void saveConfig() { EEPROM.put(0, cfg); }

void loadConfig() {
  EEPROM.get(0, cfg);
  bool ok = (cfg.magic == CFG_MAGIC) && (cfg.warn > 0) && (cfg.alarm > cfg.warn) &&
            (cfg.cnt >= 1) && (cfg.cnt <= 20) &&
            (cfg.noise[0] >= 0) && (cfg.noise[1] >= 0) && (cfg.noise[2] >= 0);
  if (!ok) { defaultConfig(); saveConfig(); }
}

// ======================= FFT =========================
inline float sinTab(uint16_t idx) {          // sin(2*pi*idx/N), idx < N/2
  int d = (int)idx - (int)(N / 4);
  if (d < 0) d = -d;
  return cosT[d];
}

void fft() {
  uint16_t j = 0;
  for (uint16_t i = 0; i < N - 1; i++) {      // reordenacion bit-reversal
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
    uint16_t k = N >> 1;
    while (k <= j) { j -= k; k >>= 1; }
    j += k;
  }
  for (uint16_t len = 2; len <= N; len <<= 1) {
    uint16_t half = len >> 1, step = N / len;
    for (uint16_t i = 0; i < N; i += len) {
      for (uint16_t k = 0; k < half; k++) {
        float wr = cosT[k * step];
        float wi = -sinTab(k * step);
        uint16_t a = i + k, b = a + half;
        float tr = re[b] * wr - im[b] * wi;
        float ti = re[b] * wi + im[b] * wr;
        re[b] = re[a] - tr; im[b] = im[a] - ti;
        re[a] += tr;        im[a] += ti;
      }
    }
  }
}

// ======================= ADQUISICION =================
bool acquireWindow() {
  uint16_t fails = 0, late = 0;
  uint32_t next = micros() + PERIODO_US;
  for (uint16_t i = 0; i < N; i++) {
    while ((int32_t)(micros() - next) < 0) { updateOutputs(); }
    if ((int32_t)(micros() - next) > (int32_t)PERIODO_US) late++;
    next += PERIODO_US;
    int16_t x, y, z;
    if (readAccel(x, y, z)) {
      raw[0][i] = x; raw[1][i] = y; raw[2][i] = z;
    } else {
      fails++;
      for (uint8_t a = 0; a < 3; a++) raw[a][i] = (i > 0) ? raw[a][i - 1] : 0;
    }
    if ((i & 63) == 0) {
      handleSerial();
      if (!running) return false;
    }
  }
  winErrors = fails + late;
  return fails < 20;
}

// ======================= PROCESADO ===================
void processWindow() {
  float best = -1.0f;
  for (uint8_t a = 0; a < 3; a++) {
    // 1) eliminar continua (gravedad + offset)
    int32_t sum = 0;
    for (uint16_t i = 0; i < N; i++) sum += raw[a][i];
    float mean = (float)sum / N;

    // 2) indicadores en el tiempo
    float s2 = 0, pk = 0;
    for (uint16_t i = 0; i < N; i++) {
      float d = ((float)raw[a][i] - mean) * G_PER_LSB;
      s2 += d * d;
      if (fabsf(d) > pk) pk = fabsf(d);
    }
    float armsA = sqrtf(s2 / N);

    // 3) ventana Hann + FFT (aceleracion en m/s2)
    for (uint16_t i = 0; i < N; i++) {
      float w = 0.5f * (1.0f - cosT[(i <= N / 2) ? i : (N - i)]);   // Hann desde la tabla (sin cosf)
      re[i] = ((float)raw[a][i] - mean) * ACC_SCALE * w;
      im[i] = 0;
    }
    fft();

    // 4) velocidad por integracion en frecuencia: V = A / (2*pi*f)
    float sumV = 0, maxAmp = 0;
    uint16_t kMax = K_LO;
    for (uint16_t k = 0; k <= K_HI; k++) {
      if (k < K_LO) { specTmp[k] = 0; continue; }
      float p = re[k] * re[k] + im[k] * im[k];
      float w = TWO_PI * k * BIN_HZ;
      sumV += p / (w * w);
      float vpk = (2.0f * sqrtf(p) / SUM_W) / w * 1000.0f;   // mm/s pico por linea
      specTmp[k] = vpk * 0.70711f;                           // -> RMS
      if (specTmp[k] > maxAmp) { maxAmp = specTmp[k]; kMax = k; }
    }
    float vrms = sqrtf(2.0f * sumV / ((float)N * SUM_W2)) * 1000.0f;   // mm/s
    vRawAxis[a] = vrms;

    // 5) descontar ruido propio del sensor (calibracion ZERO), suma cuadratica
    float vc = vrms * vrms - cfg.noise[a] * cfg.noise[a];
    vc = (vc > 0) ? sqrtf(vc) : 0;
    vAxis[a] = vc;

    // 6) eje dominante -> espectro y demas indicadores
    if (vc > best) {
      best = vc;
      memcpy(spec, specTmp, sizeof(spec));
      aRms = armsA; aPeak = pk;
      crest = (armsA > 1e-6f) ? pk / armsA : 0;
      fDom = kMax * BIN_HZ;
    }
  }
  vMax = best;
}

// ======================= ESTADO ======================
void updateState(float v) {
  uint8_t r = 0;
  if (v >= cfg.alarm) r = 2;
  else if (state == 2 && v >= cfg.alarm * (1.0f - HIST)) r = 2;
  else if (v >= cfg.warn) r = 1;
  else if (state >= 1 && v >= cfg.warn * (1.0f - HIST)) r = 1;

  if (r > state) {
    upCnt++; dnCnt = 0;
    if (upCnt >= cfg.cnt) { state = r; upCnt = 0; }
  } else if (r < state) {
    dnCnt++; upCnt = 0;
    if (dnCnt >= cfg.cnt) { state = r; dnCnt = 0; }
  } else {
    upCnt = dnCnt = 0;
  }
  if (state != prevState) { acked = false; prevState = state; }
}

// ======================= SALIDAS =====================
// Escribe el pin solo si cambia el valor (menos carga y menos jitter en el muestreo)
inline void outWrite(uint8_t pin, bool v, int8_t &cache) {
  if (cache != (int8_t)v) { digitalWrite(pin, v); cache = v; }
}

void updateOutputs() {
  uint32_t now = millis();
  static uint32_t lastRun = 0;
  if (now - lastRun < 5) return;           // 200 Hz sobran para LEDs, zumbador y pulsador
  lastRun = now;

  bool g = false, y = false, r = false, bz = false;
  if (sensorFault) {
    bool b = ((now / 250) % 2) == 0;       // fallo de sensor: los 3 LEDs parpadean
    g = y = r = b;
  } else if (!running) {
    g = ((now / 1000) % 2) == 0;           // parado: verde parpadeo lento
  } else {
    if (state == 0) g = true;
    if (state == 1) y = true;
    if (state == 2) {
      r = true;
      bz = !acked && (((now / 400) % 2) == 0);
    }
  }

  static int8_t cG = -1, cY = -1, cR = -1, cB = -1;
  static int8_t cRel[4] = {-1, -1, -1, -1};
  outWrite(PIN_LED_VERDE, g, cG);
  outWrite(PIN_LED_AMAR,  y, cY);
  outWrite(PIN_LED_ROJO,  r, cR);
  outWrite(PIN_BUZZER,    bz, cB);

  // Todos los canales usados cambian en la misma pasada (microsegundos de diferencia)
  for (uint8_t i = 0; i < N_RELES; i++)
    outWrite(PINES_RELE[i], RELE_ACTIVO_ALTO ? motorOn : !motorOn, cRel[i]);

  static uint32_t lastBtn = 0;
  if (digitalRead(PIN_BTN_ACK) == LOW && now - lastBtn > 300) {
    lastBtn = now;
    acked = true;
  }
}

// ======================= SERIE =======================
void sendConfig() {
  Serial.print("C,");
  Serial.print(cfg.warn, 2);   Serial.print(',');
  Serial.print(cfg.alarm, 2);  Serial.print(',');
  Serial.print(cfg.cnt);       Serial.print(',');
  Serial.print(running ? 1 : 0); Serial.print(',');
  Serial.print(cfg.relay);     Serial.print(',');
  Serial.print(acked ? 1 : 0);
  for (uint8_t a = 0; a < 3; a++) { Serial.print(','); Serial.print(cfg.noise[a], 3); }
  Serial.print(','); Serial.print(motorOn ? 1 : 0);
  Serial.print(','); Serial.print(tripped ? 1 : 0);
  Serial.println();
}

void sendInfo(const char *t) { Serial.print("I,"); Serial.println(t); }

void sendResults() {
  Serial.print("D,");
  Serial.print(millis());      Serial.print(',');
  Serial.print(state);         Serial.print(',');
  Serial.print(vMax, 3);       Serial.print(',');
  Serial.print(vAxis[0], 3);   Serial.print(',');
  Serial.print(vAxis[1], 3);   Serial.print(',');
  Serial.print(vAxis[2], 3);   Serial.print(',');
  Serial.print(aRms, 4);       Serial.print(',');
  Serial.print(aPeak, 4);      Serial.print(',');
  Serial.print(crest, 2);      Serial.print(',');
  Serial.print(fDom, 1);       Serial.print(',');
  Serial.print(acked ? 1 : 0); Serial.print(',');
  Serial.println(winErrors);

  Serial.print("S,");
  Serial.print(BIN_HZ, 4);     Serial.print(',');
  Serial.print(K_HI + 1);
  for (uint16_t k = 0; k <= K_HI; k++) { Serial.print(','); Serial.print(spec[k], 3); }
  Serial.println();
}

void setMotor(bool on, const char *msg) {
  if (on && !motorOn) {
    motorOnAt = millis();
    state = prevState = 0; upCnt = dnCnt = 0; acked = false;   // evalua el motor desde cero
  }
  motorOn = on;
  sendInfo(msg);
}

void processCommand(char *s) {
  for (char *p = s; *p; p++) *p = toupper(*p);

  if (strcmp(s, "START") == 0)        { running = true; sendConfig(); }
  else if (strcmp(s, "STOP") == 0)    { running = false; sendConfig(); }
  else if (strcmp(s, "GET") == 0)     { sendConfig(); }
  else if (strcmp(s, "ACK") == 0)     { acked = true; sendConfig(); }
  else if (strcmp(s, "RESET") == 0)   { state = prevState = 0; upCnt = dnCnt = 0; acked = false; tripped = false;
                                        sendInfo("Estado reiniciado"); sendConfig(); }
  else if (strcmp(s, "ZERO") == 0)    { zeroReq = true;
                                        sendInfo("Calibrando ruido del sensor (motor PARADO)..."); }
  else if (strcmp(s, "ZEROCLR") == 0) { cfg.noise[0] = cfg.noise[1] = cfg.noise[2] = 0;
                                        saveConfig(); sendInfo("Ruido de referencia borrado"); sendConfig(); }
  else if (strncmp(s, "MOTOR,", 6) == 0) {
    if (atoi(s + 6) == 0) {
      setMotor(false, "Motor DESCONECTADO (orden remota)");
    } else if (tripped) {
      Serial.println("E,Motor disparado: haga RESET antes de arrancar");
    } else if (cfg.relay && (sensorFault || !running)) {
      Serial.println("E,Con disparo automatico activo la medicion debe estar en marcha y el sensor sin fallo");
    } else {
      setMotor(true, "Motor CONECTADO (orden remota)");
    }
    sendConfig();
  }
  else if (strncmp(s, "RELAY,", 6) == 0) {
    cfg.relay = (atoi(s + 6) != 0); saveConfig(); sendConfig();
  }
  else if (strncmp(s, "TH,", 3) == 0) {
    char *p = s + 3;
    float w = atof(p);
    char *q = strchr(p, ',');
    if (q) {
      float a = atof(q + 1);
      char *r = strchr(q + 1, ',');
      int n = r ? atoi(r + 1) : cfg.cnt;
      if (w > 0 && a > w && n >= 1 && n <= 20) {
        cfg.warn = w; cfg.alarm = a; cfg.cnt = n; saveConfig();
        sendInfo("Umbrales actualizados");
      } else {
        Serial.println("E,Umbrales no validos");
      }
    }
    sendConfig();
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; processCommand(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ======================= SETUP / LOOP ================
void setup() {
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_AMAR, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  for (uint8_t i = 0; i < N_RELES; i++) {          // reles en reposo (motor desconectado) lo antes posible
    pinMode(PINES_RELE[i], OUTPUT);
    digitalWrite(PINES_RELE[i], RELE_ACTIVO_ALTO ? LOW : HIGH);
  }
  pinMode(PIN_BTN_ACK, INPUT_PULLUP);

  Serial.begin(115200);
  delay(300);

  loadConfig();
  for (uint16_t k = 0; k <= N / 2; k++) cosT[k] = cosf(TWO_PI * k / N);

  // prueba de LEDs
  digitalWrite(PIN_LED_VERDE, HIGH); delay(200);
  digitalWrite(PIN_LED_AMAR, HIGH);  delay(200);
  digitalWrite(PIN_LED_ROJO, HIGH);  delay(200);
  digitalWrite(PIN_LED_VERDE, LOW); digitalWrite(PIN_LED_AMAR, LOW); digitalWrite(PIN_LED_ROJO, LOW);

  sensorFault = !initMPU();
  if (sensorFault) {
    Serial.println("E,MPU6050 no responde (revisar SDA/SCL/alimentacion)");
  } else {
    int id = mpuReadReg(0x75);
    if (id != 0x68) sendInfo("WHO_AM_I distinto de 0x68 (posible clon); se continua");
  }
  sendInfo("Monitor de vibraciones listo");
  sendConfig();
}

void loop() {
  handleSerial();
  updateOutputs();
  if (!running) return;

  if (sensorFault) {                       // intento de recuperacion
    if (initMPU()) { sensorFault = false; sendInfo("Sensor recuperado"); }
    else { Serial.println("E,Sensor MPU6050 sin respuesta"); delay(1000); return; }
  }

  if (!acquireWindow()) {
    if (!running) return;
    sensorFault = true;
    Serial.println("E,Fallo de lectura I2C del sensor");
    return;
  }

  processWindow();

  if (zeroReq) {                           // calibracion de ruido con motor parado
    for (uint8_t a = 0; a < 3; a++) cfg.noise[a] = vRawAxis[a];
    saveConfig();
    zeroReq = false;
    sendInfo("Calibracion de ruido guardada");
    sendConfig();
    return;
  }

  updateState(vMax);

  // Disparo automatico: ALARMA confirmada (N lecturas) con el motor ya fuera del transitorio de arranque.
  // Queda ENCLAVADO: el motor NO se reconecta solo aunque la vibracion baje; hace falta RESET + orden de arranque.
  if (cfg.relay && motorOn && state == 2 && (millis() - motorOnAt) > GRACIA_ARRANQUE_MS) {
    tripped = true;
    motorOn = false;
    sendInfo("DISPARO por vibracion: motor DESCONECTADO (enclavado hasta RESET)");
    sendConfig();
  }
  sendResults();
  if (++windowsSinceCfg >= 10) { windowsSinceCfg = 0; sendConfig(); }
}
