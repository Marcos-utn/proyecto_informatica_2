/*
 * MONITOR DE VIBRACIONES - Interfaz Processing 4
 * Controla y supervisa el Arduino UNO R4 WiFi (Monitor_Vibraciones_Arduino.ino)
 *
 *  - Estado (Normal / Alerta / Alarma), valores numericos
 *  - Grafica en tiempo real, espectro de velocidad (con marcas 1x 2x 3x) y tendencia
 *  - Control: iniciar/parar, calibrar ruido, reconocer alarma, reset, rele, umbrales
 *  - Registro CSV en la carpeta "registros" del sketch
 *
 * No necesita librerias externas (solo processing.serial, incluida).
 */

import processing.serial.*;
import java.util.concurrent.ConcurrentLinkedQueue;

final int BAUD       = 115200;
final int LIVE_MAX   = 300;     // valores en la grafica en tiempo real
final int TREND_SEC  = 60;      // segundos por punto de tendencia
final int TREND_MAX  = 2880;    // 48 h de tendencia

// ---- comunicacion ----
Serial port;
String[] ports = new String[0];
int portIdx = 0;
boolean connected = false;
int lastRx = 0;
ConcurrentLinkedQueue<String> rxQueue = new ConcurrentLinkedQueue<String>();

// ---- datos ----
int state = 0, prevState = -1;
float vmax, vx, vy, vz, arms, apeak, crest, fdom;
int errs = 0;
boolean haveData = false, running = true, acked = false, relayEn = false;
float warnT = 1.8, alarmT = 4.5;
int cntN = 3;
int rpm = 1500;
float[] spec = new float[0];
float binHz = 0.9766;

FloatList live = new FloatList();
FloatList trendAvg = new FloatList();
FloatList trendMax = new FloatList();
float accSum = 0, accMax = 0;
int accN = 0, accStart;

ArrayList<String> events = new ArrayList<String>();

// ---- CSV ----
PrintWriter csv = null;
boolean csvOn = true;

// ---- interfaz ----
String[] stTxt = {"NORMAL", "ALERTA", "ALARMA"};
color[] stCol = new color[3];
ArrayList<Btn> btns = new ArrayList<Btn>();
Btn bPrev, bNext, bRefresh, bConnect;
Btn bStart, bZero, bAck, bRelay, bReset, bCsv;
Btn bWarnM, bWarnP, bAlarmM, bAlarmP, bCntM, bCntP, bRpmM, bRpmP;
Btn[] hdrBtns, leftBtns;   // listas fijas: evitan crear arrays en cada fotograma

class Btn {
  float x, y, w, h;
  String label;
  boolean on = false;
  Btn(float x, float y, float w, float h, String label) {
    this.x = x; this.y = y; this.w = w; this.h = h; this.label = label;
    btns.add(this);
  }
  boolean over() { return mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h; }
  void show() {
    stroke(110);
    if (on) fill(41, 128, 185);
    else if (over()) fill(88, 101, 128);
    else fill(55, 65, 85);
    rect(x, y, w, h, 6);
    fill(255); noStroke();
    textAlign(CENTER, CENTER); textSize(13);
    text(label, x + w / 2, y + h / 2);
  }
}

void setup() {
  size(1280, 800);
  surface.setTitle("Monitor de vibraciones - Mantenimiento predictivo");
  frameRate(30);   // suficiente para esta interfaz y reduce el uso de CPU
  textFont(createFont("SansSerif", 14));
  stCol[0] = color(46, 204, 113);
  stCol[1] = color(241, 196, 15);
  stCol[2] = color(231, 76, 60);
  accStart = millis();

  bPrev    = new Btn(700, 14, 28, 28, "<");
  bNext    = new Btn(930, 14, 28, 28, ">");
  bRefresh = new Btn(964, 14, 34, 28, "R");
  bConnect = new Btn(1005, 14, 110, 28, "CONECTAR");

  bStart = new Btn(20, 384, 132, 32, "PARAR");
  bZero  = new Btn(160, 384, 132, 32, "CALIB. RUIDO");
  bAck   = new Btn(20, 422, 132, 32, "ACK ALARMA");
  bRelay = new Btn(160, 422, 132, 32, "RELE: OFF");
  bReset = new Btn(20, 460, 132, 32, "RESET ESTADO");
  bCsv   = new Btn(160, 460, 132, 32, "CSV: ON");

  bWarnM  = new Btn(160, 516, 30, 28, "-");  bWarnP  = new Btn(262, 516, 30, 28, "+");
  bAlarmM = new Btn(160, 550, 30, 28, "-");  bAlarmP = new Btn(262, 550, 30, 28, "+");
  bCntM   = new Btn(160, 584, 30, 28, "-");  bCntP   = new Btn(262, 584, 30, 28, "+");
  bRpmM   = new Btn(160, 618, 30, 28, "-");  bRpmP   = new Btn(262, 618, 30, 28, "+");

  hdrBtns  = new Btn[] {bPrev, bNext, bRefresh, bConnect};
  leftBtns = new Btn[] {bStart, bZero, bAck, bRelay, bReset, bCsv, bWarnM, bWarnP, bAlarmM, bAlarmP, bCntM, bCntP, bRpmM, bRpmP};

  refreshPorts();
}

// =====================================================
//  BUCLE PRINCIPAL
// =====================================================
void draw() {
  background(15, 19, 28);
  String l;
  while ((l = rxQueue.poll()) != null) parseLine(l);

  drawHeader();
  drawLeft();

  float X = 312, W = width - X - 10;
  linePlot(X, 66, W, 230, "Velocidad de vibracion RMS (mm/s) - tiempo real", live, null, LIVE_MAX, "-5 min", "ahora");
  spectrumPlot(X, 304, W, 230);
  String tl = "-" + nf(trendAvg.size() * TREND_SEC / 3600.0f, 1, 1) + " h";
  linePlot(X, 542, W, 250, "Tendencia: media (azul) y maximo (naranja) por minuto", trendAvg, trendMax, 0, tl, "ahora");
}

// =====================================================
//  SERIE
// =====================================================
void refreshPorts() {
  ports = Serial.list();
  portIdx = constrain(portIdx, 0, max(0, ports.length - 1));
}

void connectPort() {
  if (connected) { disconnectPort(); return; }
  if (ports.length == 0) { addEvent("No hay puertos serie"); return; }
  try {
    port = new Serial(this, ports[portIdx], BAUD);
    port.bufferUntil('\n');
    connected = true;
    lastRx = millis();
    haveData = false;
    if (csvOn) openCsv();
    addEvent("Conectado a " + ports[portIdx]);
    sendCmd("GET");
  } catch (Exception e) {
    connected = false;
    addEvent("No se pudo abrir el puerto");
  }
}

void disconnectPort() {
  try { port.stop(); } catch (Exception e) {}
  connected = false;
  haveData = false;
  closeCsv();
  addEvent("Desconectado");
}

void serialEvent(Serial p) {
  String s = p.readStringUntil('\n');
  if (s != null) rxQueue.add(s.trim());
}

void sendCmd(String c) {
  if (!connected) { addEvent("No conectado"); return; }
  try { port.write(c + "\n"); } catch (Exception e) { addEvent("Error de escritura"); }
}

void sendTH(float w, float a, int n) {
  w = round(w * 10) / 10.0f;
  a = round(a * 10) / 10.0f;
  if (w < 0.1f || a <= w || n < 1 || n > 20) { addEvent("Umbral no valido"); return; }
  sendCmd("TH," + nf(w, 1, 1).replace(',', '.') + "," + nf(a, 1, 1).replace(',', '.') + "," + n);
}

void parseLine(String s) {
  if (s.length() < 2) return;
  lastRx = millis();
  String[] t = split(s, ',');
  char c = s.charAt(0);

  if (c == 'D' && t.length >= 13) {
    haveData = true;
    state = int(t[2]);
    vmax = fnum(t[3]); vx = fnum(t[4]); vy = fnum(t[5]); vz = fnum(t[6]);
    arms = fnum(t[7]); apeak = fnum(t[8]); crest = fnum(t[9]); fdom = fnum(t[10]);
    acked = int(t[11]) == 1;
    errs = int(t[12]);

    live.append(vmax);
    while (live.size() > LIVE_MAX) live.remove(0);

    accSum += vmax; accN++; accMax = max(accMax, vmax);
    if (millis() - accStart >= TREND_SEC * 1000) {
      trendAvg.append(accSum / accN);
      trendMax.append(accMax);
      while (trendAvg.size() > TREND_MAX) { trendAvg.remove(0); trendMax.remove(0); }
      accSum = 0; accN = 0; accMax = 0; accStart = millis();
    }

    if (state != prevState) {
      if (prevState >= 0) addEvent(stTxt[state] + " (" + nf(vmax, 1, 2) + " mm/s)");
      prevState = state;
    }
    writeCsv();
  }
  else if (c == 'S' && t.length >= 4) {
    binHz = fnum(t[1]);
    int n = int(t[2]);
    if (t.length >= 3 + n) {
      float[] sp = new float[n];
      for (int i = 0; i < n; i++) sp[i] = fnum(t[3 + i]);
      spec = sp;
    }
  }
  else if (c == 'C' && t.length >= 7) {
    warnT = fnum(t[1]); alarmT = fnum(t[2]); cntN = int(t[3]);
    running = int(t[4]) == 1; relayEn = int(t[5]) == 1; acked = int(t[6]) == 1;
  }
  else if (c == 'I' && t.length >= 2) addEvent(s.substring(2));
  else if (c == 'E' && t.length >= 2) addEvent("ERROR: " + s.substring(2));
}

float fnum(String s) {
  float v = float(s);
  return Float.isNaN(v) ? 0 : v;
}

// =====================================================
//  CSV
// =====================================================
String stamp(boolean forFile) {
  if (forFile) return nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  return nf(year(), 4) + "-" + nf(month(), 2) + "-" + nf(day(), 2) + " " + nf(hour(), 2) + ":" + nf(minute(), 2) + ":" + nf(second(), 2);
}

String f3(float v, int d) { return String.format(java.util.Locale.US, "%." + d + "f", v).replace('.', ','); }

void openCsv() {
  if (csv != null) return;
  String name = "registros/vibraciones_" + stamp(true) + ".csv";
  csv = createWriter(name);
  csv.println("fecha_hora;estado;v_max_mm_s;vx_mm_s;vy_mm_s;vz_mm_s;acel_rms_g;acel_pico_g;factor_cresta;f_dominante_Hz;alerta_mm_s;alarma_mm_s");
  csv.flush();
  addEvent("CSV: " + name);
}

void closeCsv() {
  if (csv != null) { csv.flush(); csv.close(); csv = null; }
}

void writeCsv() {
  if (csv == null || !csvOn) return;
  csv.println(stamp(false) + ";" + stTxt[constrain(state, 0, 2)] + ";" + f3(vmax, 3) + ";" + f3(vx, 3) + ";" + f3(vy, 3) + ";" + f3(vz, 3)
    + ";" + f3(arms, 4) + ";" + f3(apeak, 4) + ";" + f3(crest, 2) + ";" + f3(fdom, 1) + ";" + f3(warnT, 1) + ";" + f3(alarmT, 1));
  csv.flush();
}

void exit() {
  closeCsv();
  super.exit();
}

void addEvent(String s) {
  events.add(0, nf(hour(), 2) + ":" + nf(minute(), 2) + ":" + nf(second(), 2) + " " + s);
  while (events.size() > 8) events.remove(events.size() - 1);
}

// =====================================================
//  INTERFAZ
// =====================================================
void drawHeader() {
  noStroke(); fill(28, 34, 46); rect(0, 0, width, 56);
  fill(255); textAlign(LEFT, CENTER); textSize(20);
  text("MONITOR DE VIBRACIONES  -  Motor electrico", 14, 28);

  textSize(13); fill(200); textAlign(RIGHT, CENTER);
  text("Puerto:", 694, 28);
  String pn = ports.length > 0 ? ports[portIdx] : "(sin puertos)";
  if (pn.length() > 24) pn = "..." + pn.substring(pn.length() - 21);
  fill(255); textAlign(CENTER, CENTER);
  text(pn, 829, 28);

  bConnect.label = connected ? "DESCONECTAR" : "CONECTAR";
  bConnect.on = connected;
  for (Btn b : hdrBtns) b.show();

  boolean stale = connected && running && (millis() - lastRx > 5000);
  fill(!connected ? color(150) : (stale ? stCol[1] : stCol[0]));
  textAlign(LEFT, CENTER);
  text(!connected ? "OFFLINE" : (stale ? "SIN DATOS" : "ONLINE"), 1128, 28);
}

void drawLeft() {
  fill(22, 27, 38); stroke(70); rect(10, 66, 292, 726, 6);
  noStroke();

  boolean stale = connected && running && (millis() - lastRx > 5000);
  String txt; color c;
  if (!connected)      { txt = "DESCONECTADO"; c = color(120); }
  else if (!haveData || stale) { txt = running ? "SIN DATOS" : "PARADO"; c = color(120); }
  else if (!running)   { txt = "PARADO"; c = color(120); }
  else { txt = stTxt[constrain(state, 0, 2)]; c = stCol[constrain(state, 0, 2)]; }

  fill(c); ellipse(62, 122, 78, 78);
  fill(255); textAlign(LEFT, CENTER); textSize(22);
  text(txt, 112, 112);
  textSize(12); fill(170);
  text(state == 2 && acked ? "Zumbador silenciado" : "Estado del motor", 112, 138);

  float y = 186;
  color vc = (haveData && running) ? c : color(230);
  kv("Vel. RMS maxima", f1(vmax, 2) + " mm/s", y, vc);          y += 22;
  kv("   Eje X", f1(vx, 2) + " mm/s", y, color(210));            y += 22;
  kv("   Eje Y", f1(vy, 2) + " mm/s", y, color(210));            y += 22;
  kv("   Eje Z", f1(vz, 2) + " mm/s", y, color(210));            y += 22;
  kv("Aceleracion RMS", f1(arms, 3) + " g", y, color(210));      y += 22;
  kv("Aceleracion pico", f1(apeak, 3) + " g", y, color(210));    y += 22;
  kv("Factor de cresta", f1(crest, 1), y, crest > 6 ? stCol[1] : color(210)); y += 22;
  float f1x = rpm / 60.0f;
  kv("Frec. dominante", f1(fdom, 1) + " Hz (" + f1(fdom / f1x, 1) + "x)", y, color(210)); y += 22;

  bStart.label = running ? "PARAR" : "INICIAR"; bStart.on = running;
  bAck.on = acked;
  bRelay.label = "RELE: " + (relayEn ? "ON" : "OFF"); bRelay.on = relayEn;
  bCsv.label = "CSV: " + (csvOn ? "ON" : "OFF"); bCsv.on = csvOn;
  for (Btn b : leftBtns) b.show();

  param("Alerta (mm/s)", f1(warnT, 1), 516);
  param("Alarma (mm/s)", f1(alarmT, 1), 550);
  param("Lecturas conf.", str(cntN), 584);
  param("Motor (rpm)", str(rpm), 618);

  fill(200); textAlign(LEFT, TOP); textSize(13);
  text("Eventos", 20, 660);
  textSize(11); fill(170);
  for (int i = 0; i < events.size(); i++) {
    String e = events.get(i);
    if (e.length() > 46) e = e.substring(0, 46);
    text(e, 20, 678 + i * 13);
  }
  if (errs > 0 && connected) { fill(stCol[1]); textSize(11); text("Muestras perdidas en ventana: " + errs, 120, 660); }
}

String f1(float v, int d) { return nf(v, 1, d); }

void kv(String k, String v, float y, color c) {
  textSize(13); textAlign(LEFT, CENTER);
  fill(150); text(k, 20, y);
  fill(c); textAlign(RIGHT, CENTER); text(v, 292, y);
}

void param(String k, String v, float y) {
  fill(200); textAlign(LEFT, CENTER); textSize(13);
  text(k, 20, y + 14);
  textAlign(CENTER, CENTER); fill(255);
  text(v, 226, y + 14);
}

void plotFrame(float x, float y, float w, float h, String title) {
  fill(22, 27, 38); stroke(70); rect(x, y, w, h, 6);
  fill(220); noStroke(); textAlign(LEFT, TOP); textSize(13);
  text(title, x + 10, y + 7);
}

void thrLine(float px, float py, float pw, float ph, float ymax, float thr, color c, String name) {
  float yy = py + ph - ph * constrain(thr / ymax, 0, 1);
  stroke(c); line(px, yy, px + pw, yy);
  fill(c); noStroke(); textSize(11); textAlign(RIGHT, BOTTOM);
  text(name + " " + nf(thr, 1, 1), px + pw - 4, yy - 2);
}

void drawSeries(FloatList d, float px, float py, float pw, float ph, float ymax, int sp, color c, float wgt) {
  noFill(); stroke(c); strokeWeight(wgt);
  beginShape();
  int n = d.size();
  for (int i = 0; i < n; i++) {
    float tx = px + pw * (i + (sp - n)) / (float)(sp - 1);
    float ty = py + ph - ph * constrain(d.get(i) / ymax, 0, 1);
    vertex(tx, ty);
  }
  endShape();
  strokeWeight(1);
}

void linePlot(float x, float y, float w, float h, String title, FloatList a, FloatList b, int span, String xl, String xr) {
  plotFrame(x, y, w, h, title);
  float px = x + 50, py = y + 30, pw = w - 66, ph = h - 56;
  float ymax = alarmT * 1.25f;
  for (int i = 0; i < a.size(); i++) ymax = max(ymax, a.get(i) * 1.1f);
  if (b != null) for (int i = 0; i < b.size(); i++) ymax = max(ymax, b.get(i) * 1.1f);

  textSize(11);
  for (int i = 0; i <= 4; i++) {
    float yy = py + ph - ph * i / 4.0f;
    stroke(40, 47, 62); line(px, yy, px + pw, yy);
    noStroke(); fill(150); textAlign(RIGHT, CENTER);
    text(nf(ymax * i / 4.0f, 1, 1), px - 6, yy);
  }
  thrLine(px, py, pw, ph, ymax, warnT, stCol[1], "Alerta");
  thrLine(px, py, pw, ph, ymax, alarmT, stCol[2], "Alarma");

  int n = a.size();
  int sp = (span > 0) ? span : max(n, 2);
  if (b != null && b.size() > 1) drawSeries(b, px, py, pw, ph, ymax, sp, color(230, 126, 34), 1);
  if (n > 1) drawSeries(a, px, py, pw, ph, ymax, sp, color(52, 152, 219), 2);
  else {
    fill(120); noStroke(); textAlign(CENTER, CENTER); textSize(13);
    text("Acumulando datos...", px + pw / 2, py + ph / 2);
  }
  noStroke(); fill(150); textSize(11);
  textAlign(LEFT, TOP);  text(xl, px, py + ph + 6);
  textAlign(RIGHT, TOP); text(xr, px + pw, py + ph + 6);
}

void spectrumPlot(float x, float y, float w, float h) {
  plotFrame(x, y, w, h, "Espectro de velocidad (mm/s RMS) - eje dominante   [marcas: 1x 2x 3x velocidad de giro]");
  float px = x + 50, py = y + 30, pw = w - 66, ph = h - 56;
  int n = spec.length;
  float fmax = max(1, (n - 1) * binHz);
  float ymax = 0.5f;
  for (int i = 0; i < n; i++) ymax = max(ymax, spec[i] * 1.2f);

  textSize(11);
  for (int i = 0; i <= 4; i++) {
    float yy = py + ph - ph * i / 4.0f;
    stroke(40, 47, 62); line(px, yy, px + pw, yy);
    noStroke(); fill(150); textAlign(RIGHT, CENTER);
    text(nf(ymax * i / 4.0f, 1, 2), px - 6, yy);
  }
  for (int f = 0; f <= fmax; f += 25) {
    float xx = px + pw * f / fmax;
    stroke(40, 47, 62); line(xx, py, xx, py + ph);
    noStroke(); fill(150); textAlign(CENTER, TOP);
    text(f + " Hz", xx, py + ph + 6);
  }
  noStroke(); fill(46, 204, 113);
  for (int i = 0; i < n; i++) {
    float bx = px + pw * (i * binHz) / fmax;
    float bh = ph * constrain(spec[i] / ymax, 0, 1);
    rect(bx, py + ph - bh, max(pw / n, 1), bh);
  }
  float f1x = rpm / 60.0f;
  for (int m = 1; m <= 3; m++) {
    float xx = px + pw * (f1x * m) / fmax;
    if (xx > px + pw) break;
    stroke(241, 196, 15); line(xx, py, xx, py + ph);
    noStroke(); fill(241, 196, 15); textAlign(LEFT, TOP);
    text(m + "x", xx + 3, py + 2);
  }
}

// =====================================================
//  RATON
// =====================================================
void mousePressed() {
  for (Btn b : btns) if (b.over()) { onBtn(b); break; }
}

void onBtn(Btn b) {
  if (b == bPrev)         { if (!connected && ports.length > 0) portIdx = (portIdx - 1 + ports.length) % ports.length; }
  else if (b == bNext)    { if (!connected && ports.length > 0) portIdx = (portIdx + 1) % ports.length; }
  else if (b == bRefresh) { if (!connected) { refreshPorts(); addEvent(ports.length + " puerto(s) encontrado(s)"); } }
  else if (b == bConnect) connectPort();
  else if (b == bStart)   sendCmd(running ? "STOP" : "START");
  else if (b == bZero)    sendCmd("ZERO");
  else if (b == bAck)     sendCmd("ACK");
  else if (b == bReset)   sendCmd("RESET");
  else if (b == bRelay)   sendCmd(relayEn ? "RELAY,0" : "RELAY,1");
  else if (b == bCsv)     { csvOn = !csvOn; if (connected) { if (csvOn) openCsv(); else closeCsv(); } }
  else if (b == bWarnM)   sendTH(warnT - 0.1f, alarmT, cntN);
  else if (b == bWarnP)   sendTH(warnT + 0.1f, alarmT, cntN);
  else if (b == bAlarmM)  sendTH(warnT, alarmT - 0.1f, cntN);
  else if (b == bAlarmP)  sendTH(warnT, alarmT + 0.1f, cntN);
  else if (b == bCntM)    sendTH(warnT, alarmT, cntN - 1);
  else if (b == bCntP)    sendTH(warnT, alarmT, cntN + 1);
  else if (b == bRpmM)    rpm = max(100, rpm - 50);
  else if (b == bRpmP)    rpm = min(6000, rpm + 50);
}
