#include <SCServo.h>

// Servo setup and test over USB serial (115200). IDs are set with set-id.cpp.
// Keys:
//   s  scan IDs 0-20 with position, voltage and temperature
//   q  bus quality: ping success rate per servo
//   t  torque off on all servos and the Z stepper, so they can be moved by hand
//   c  calibrate centre: current arm pose becomes 2048 on IDs 1 and 2
//   l  angle limits: shoulder +/-95 deg, elbow +/-90 deg, hand-taught open/close on ID 3
//   j  jog each joint +15 deg and back, to check rotation direction
//   w  sweep one servo +/-20 deg
//   h  spin one servo 180 deg
//   p  print positions of all servos
//   d  bus diagnostics (line levels, passive listen, raw ping, edges, motion)

// Servo driver is on Serial2 (TX2 = pin 16 = PH1, RX2 = pin 17 = PH0).
// TX2 must stay driven: the driver board pulls an undriven TX line low,
// which corrupts the bus. Serial2.end() releases TX2, so only call it with
// TX2 already driven as a GPIO output.
static const unsigned long BUS_BAUD = 1000000;
static const byte TX_PIN = 16;
static const byte RX_PIN = 17;
static const byte RX_BIT = _BV(PH0);

static const byte SHOULDER_ID = 1;
static const byte ELBOW_ID = 2;
static const byte GRIPPER_ID = 3;
static const byte JOINT_IDS[] = {SHOULDER_ID, ELBOW_ID};
static const byte ALL_IDS[] = {SHOULDER_ID, ELBOW_ID, GRIPPER_ID};

static const int CENTER = 2048;
static const int COUNTS_PER_REV = 4096;
static const int SHOULDER_LIMIT_COUNTS = 1081; // 95 deg
static const int ELBOW_LIMIT_COUNTS = 1024;    // 90 deg
static const int JOG_COUNTS = 171;    // 15 deg
static const int JOG_SPEED = 400;
static const byte JOG_ACC = 20;
static const int MIN_GRIP_TRAVEL = 20;
static const int SWEEP_COUNTS = 228; // 20 deg
static const byte SWEEP_TIMES = 2;
static const int SWEEP_SPEED = 300;
static const byte SWEEP_ACC = 50;
// Full swing at SPEED, plus ramp time SPEED / (ACC * 100) s, plus margin.
static const unsigned long SWEEP_MOVE_MS =
    2UL * SWEEP_COUNTS * 1000 / SWEEP_SPEED + SWEEP_SPEED * 1000UL / (SWEEP_ACC * 100UL) + 100;
static const int SPIN_SPEED = 1000;
static const byte SPIN_ACC = 50;
// One-way swing of half a revolution, plus ramp time, plus margin.
static const unsigned long SPIN_MOVE_MS =
    (COUNTS_PER_REV / 2) * 1000UL / SPIN_SPEED + SPIN_SPEED * 1000UL / (SPIN_ACC * 100UL) + 300;

static const unsigned long DIAG_BAUDS[] = {1000000, 500000, 250000, 128000,
                                           115200,  76800,  57600,  38400};
// Z stepper driver ENABLE input, active low. Placeholder pin until wired;
// must match Z_EN_PIN in manual.cpp.
static const byte Z_EN_PIN = 4;
static const byte Z_ENABLED_LEVEL = LOW;

static const byte BROADCAST_ID = 0xFE;
static const int SCAN_MAX_ID = 20;
static const unsigned int QUALITY_PINGS = 200;

SMS_STS st;

// ---------------------------------------------------------------- servo helpers

// The bus occasionally drops a reply, so reads are retried.
static const byte BUS_TRIES = 3;
static const byte RETRY_DELAY_MS = 2;

static bool ping(byte id)
{
  for (byte i = 0; i < BUS_TRIES; i++) {
    if (st.Ping(id) == id) {
      return true;
    }
    delay(RETRY_DELAY_MS);
  }
  return false;
}

static bool readU16(byte id, byte addr, unsigned int &value)
{
  byte b[2];
  for (byte i = 0; i < BUS_TRIES; i++) {
    if (st.Read(id, addr, b, 2) == 2) {
      value = b[0] | (b[1] << 8);
      return true;
    }
    delay(RETRY_DELAY_MS);
  }
  return false;
}

static bool readPosition(byte id, int &pos)
{
  unsigned int raw;
  if (!readU16(id, SMS_STS_PRESENT_POSITION_L, raw)) {
    return false;
  }
  pos = (raw & 0x8000) ? -(int)(raw & 0x7FFF) : (int)raw;
  return true;
}

static void calibrateCentre(byte id)
{
  st.unLockEprom(id);
  delay(10);
  st.CalibrationOfs(id);
  delay(50);
  st.LockEprom(id);
  delay(10);
}

static void writeLimits(byte id, int lo, int hi)
{
  st.unLockEprom(id);
  delay(10);
  st.writeWord(id, SMS_STS_MIN_ANGLE_LIMIT_L, lo);
  delay(10);
  st.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, hi);
  delay(10);
  st.LockEprom(id);
  delay(10);
}

static void printLimits(byte id)
{
  unsigned int lo, hi;
  if (readU16(id, SMS_STS_MIN_ANGLE_LIMIT_L, lo) &&
      readU16(id, SMS_STS_MAX_ANGLE_LIMIT_L, hi)) {
    Serial.print(F(" limits "));
    Serial.print(lo);
    Serial.print(F(" .. "));
    Serial.println(hi);
  } else {
    Serial.println(F(" limits written, readback failed"));
  }
}

// ---------------------------------------------------------------- USB serial helpers

static void flushInput()
{
  delay(20);
  while (Serial.read() != -1) {
  }
}

// Waits for a key press and returns it, discarding the rest of the line.
static char waitForKey()
{
  flushInput();
  while (!Serial.available()) {
  }
  char c = Serial.read();
  flushInput();
  return c;
}

static long readNumber()
{
  flushInput();
  while (!Serial.available()) {
  }
  Serial.setTimeout(10000);
  long v = Serial.parseInt();
  flushInput();
  return v;
}

static bool confirm()
{
  Serial.println(F("Type y to continue, anything else to cancel."));
  char c = waitForKey();
  if (c != 'y' && c != 'Y') {
    Serial.println(F("Cancelled."));
    return false;
  }
  return true;
}

static void promptEnter(const __FlashStringHelper *msg)
{
  Serial.print(msg);
  Serial.println(F(", then press Enter."));
  waitForKey();
}

static void printId(byte id)
{
  Serial.print(F("ID "));
  Serial.print(id);
  switch (id) {
  case SHOULDER_ID: Serial.print(F(" (shoulder)")); break;
  case ELBOW_ID: Serial.print(F(" (elbow)")); break;
  case GRIPPER_ID: Serial.print(F(" (gripper)")); break;
  }
}

static void printDeg(int pos)
{
  Serial.print((pos - CENTER) * 360.0 / COUNTS_PER_REV, 1);
  Serial.print(F(" deg"));
}

static bool requirePresent(const byte *ids, byte n)
{
  bool ok = true;
  for (byte i = 0; i < n; i++) {
    if (!ping(ids[i])) {
      printId(ids[i]);
      Serial.println(F(" not responding."));
      ok = false;
    }
  }
  return ok;
}

// ---------------------------------------------------------------- commands

static void cmdScan()
{
  Serial.println(F("Scanning IDs 0-20..."));
  byte found = 0;
  for (int id = 0; id <= SCAN_MAX_ID; id++) {
    if (!ping(id)) {
      continue;
    }
    found++;
    Serial.print(F("  found "));
    printId(id);
    int pos;
    if (readPosition(id, pos)) {
      Serial.print(F("  pos="));
      Serial.print(pos);
    }
    int volt = st.ReadVoltage(id);
    if (volt >= 0) {
      Serial.print(F("  volt="));
      Serial.print(volt / 10.0, 1);
      Serial.print('V');
    }
    int temp = st.ReadTemper(id);
    if (temp >= 0) {
      Serial.print(F("  temp="));
      Serial.print(temp);
      Serial.print('C');
    }
    Serial.println();
  }
  Serial.print(found);
  Serial.println(F(" servo(s) found."));
}

// Single pings without retries, to measure how often replies are lost.
static void cmdQuality()
{
  Serial.print(F("Bus quality: "));
  Serial.print(QUALITY_PINGS);
  Serial.println(F(" pings per ID, no retries"));
  for (byte id : ALL_IDS) {
    unsigned int ok = 0;
    for (unsigned int i = 0; i < QUALITY_PINGS; i++) {
      if (st.Ping(id) == id) {
        ok++;
      }
    }
    printId(id);
    Serial.print(F(": "));
    Serial.print(ok);
    Serial.print('/');
    Serial.println(QUALITY_PINGS);
  }
  Serial.println(F("  anything under 100% on a connected servo = flaky bus"));
}

static void cmdTorqueOff()
{
  for (byte id : ALL_IDS) {
    st.EnableTorque(id, 0);
  }
  digitalWrite(Z_EN_PIN, !Z_ENABLED_LEVEL);
  Serial.println(F("Torque off on IDs 1-3 and Z stepper. Arm can be moved by hand."));
}

static void cmdCalibrate()
{
  if (!requirePresent(JOINT_IDS, sizeof(JOINT_IDS))) {
    return;
  }
  for (byte id : JOINT_IDS) {
    st.EnableTorque(id, 0);
  }
  Serial.println(F("Calibrate centre: hold the arm STRAIGHT (both links in line)."));
  if (!confirm()) {
    return;
  }
  for (byte id : JOINT_IDS) {
    calibrateCentre(id);
    int pos;
    printId(id);
    if (readPosition(id, pos)) {
      Serial.print(F(" now reads "));
      Serial.println(pos);
    } else {
      Serial.println(F(" calibrated, but position read failed"));
    }
  }
  Serial.println(F("Both should read about 2048."));
}

static void jointLimits()
{
  for (byte id : JOINT_IDS) {
    if (!ping(id)) {
      printId(id);
      Serial.println(F(" not responding, skipped."));
      continue;
    }
    int limit = id == SHOULDER_ID ? SHOULDER_LIMIT_COUNTS : ELBOW_LIMIT_COUNTS;
    writeLimits(id, CENTER - limit, CENTER + limit);
    printId(id);
    printLimits(id);
  }
}

// Teaches the gripper's travel by hand. If open..close crosses the servo's
// 0/4095 seam the limits can't be expressed as min..max, so the gripper is
// recentred at mid-travel and taught again.
static void gripperLimits()
{
  if (!ping(GRIPPER_ID)) {
    printId(GRIPPER_ID);
    Serial.println(F(" not responding, skipped."));
    return;
  }
  st.EnableTorque(GRIPPER_ID, 0);
  Serial.println(F("Gripper limits: torque is off, move the gripper by hand."));

  for (byte attempt = 0; attempt < 2; attempt++) {
    int open, close;
    promptEnter(F("Move gripper to MAX OPEN"));
    if (!readPosition(GRIPPER_ID, open)) {
      Serial.println(F("Position read failed. Cancelled."));
      return;
    }
    Serial.print(F("  open = "));
    Serial.println(open);

    promptEnter(F("Move gripper to MAX CLOSE"));
    if (!readPosition(GRIPPER_ID, close)) {
      Serial.println(F("Position read failed. Cancelled."));
      return;
    }
    Serial.print(F("  close = "));
    Serial.println(close);

    int travel = abs(open - close);
    if (travel < MIN_GRIP_TRAVEL) {
      Serial.println(F("Open and close are the same position. Cancelled."));
      return;
    }
    if (travel <= COUNTS_PER_REV / 2) {
      writeLimits(GRIPPER_ID, min(open, close), max(open, close));
      printId(GRIPPER_ID);
      printLimits(GRIPPER_ID);
      Serial.print(F("For firmware: GRIPPER_OPEN = "));
      Serial.print(open);
      Serial.print(F(", GRIPPER_CLOSE = "));
      Serial.println(close);
      return;
    }

    if (attempt > 0) {
      break;
    }
    Serial.println(F("Travel crosses the servo's zero point; recentring."));
    promptEnter(F("Move gripper HALF WAY between open and close"));
    calibrateCentre(GRIPPER_ID);
    Serial.println(F("Recentred. Teach open and close again."));
  }
  Serial.println(F("Travel still crosses the zero point. Cancelled."));
}

static void cmdLimits()
{
  jointLimits();
  gripperLimits();
}

static void cmdJog()
{
  if (!requirePresent(JOINT_IDS, sizeof(JOINT_IDS))) {
    return;
  }
  Serial.println(F("Jog: each joint turns +15 deg then back. Note which way it turns"));
  Serial.println(F("viewed from ABOVE. Counter-clockwise -> SIGN = +1, clockwise -> SIGN = -1."));
  if (!confirm()) {
    return;
  }
  const char *names[] = {"SHOULDER_SIGN", "ELBOW_SIGN"};
  for (byte i = 0; i < sizeof(JOINT_IDS); i++) {
    byte id = JOINT_IDS[i];
    int start;
    if (!readPosition(id, start)) {
      Serial.println(F("Position read failed. Cancelled."));
      return;
    }
    st.EnableTorque(id, 1);
    printId(id);
    Serial.print(F(" (sets "));
    Serial.print(names[i]);
    Serial.println(F(") moving +15 deg now..."));
    st.WritePosEx(id, start + JOG_COUNTS, JOG_SPEED, JOG_ACC);
    delay(1500);
    st.WritePosEx(id, start, JOG_SPEED, JOG_ACC);
    delay(1500);
  }
  Serial.println(F("Done. Torque left on; press t to release."));
}

static void cmdPositions()
{
  for (byte id : ALL_IDS) {
    int pos;
    printId(id);
    if (!readPosition(id, pos)) {
      Serial.println(F(": no reply"));
      continue;
    }
    Serial.print(F(": "));
    Serial.print(pos);
    if (id != GRIPPER_ID) {
      Serial.print(F(" ("));
      printDeg(pos);
      Serial.print(F(" from centre)"));
    }
    Serial.println();
  }
}

static void cmdSweep()
{
  Serial.println(F("Sweep: ID to move (1 = shoulder, 2 = elbow, 3 = gripper)?"));
  long id = readNumber();
  if (id < 0 || id > 253 || !ping(id)) {
    Serial.println(F("No servo at that ID. Cancelled."));
    return;
  }
  int start;
  if (!readPosition(id, start)) {
    Serial.println(F("Position read failed. Cancelled."));
    return;
  }
  const int plus = constrain(start + SWEEP_COUNTS, 0, COUNTS_PER_REV - 1);
  const int minus = constrain(start - SWEEP_COUNTS, 0, COUNTS_PER_REV - 1);
  printId(id);
  Serial.print(F(" at "));
  Serial.print(start);
  Serial.println(F(", sweeping +/-20 deg"));

  st.EnableTorque(id, 1);
  for (byte i = 0; i < SWEEP_TIMES; i++) {
    Serial.println(F("  +20"));
    st.WritePosEx(id, plus, SWEEP_SPEED, SWEEP_ACC);
    delay(SWEEP_MOVE_MS);
    Serial.println(F("  -20"));
    st.WritePosEx(id, minus, SWEEP_SPEED, SWEEP_ACC);
    delay(SWEEP_MOVE_MS);
  }
  st.WritePosEx(id, start, SWEEP_SPEED, SWEEP_ACC);
  delay(SWEEP_MOVE_MS);
  int end;
  Serial.print(F("Back at start"));
  if (readPosition(id, end)) {
    Serial.print(F(", reads "));
    Serial.print(end);
  }
  Serial.println(F(". Torque left on; press t to release."));
}

// Turns half a revolution from the current position. The servo clamps goals
// to its angle limits, so a target outside them would stop at the limit;
// in that case the limits can be cleared first (re-teach them with l after).
static void cmdSpin()
{
  Serial.println(F("Spin 180: ID to move (1 = shoulder, 2 = elbow, 3 = gripper)?"));
  long id = readNumber();
  if (id < 0 || id > 253 || !ping(id)) {
    Serial.println(F("No servo at that ID. Cancelled."));
    return;
  }
  int start;
  unsigned int lo, hi;
  if (!readPosition(id, start)) {
    Serial.println(F("Position read failed. Cancelled."));
    return;
  }
  if (!readU16(id, SMS_STS_MIN_ANGLE_LIMIT_L, lo) ||
      !readU16(id, SMS_STS_MAX_ANGLE_LIMIT_L, hi)) {
    Serial.println(F("Angle limit read failed. Cancelled."));
    return;
  }
  const int half = COUNTS_PER_REV / 2;
  const int target = start + half < COUNTS_PER_REV ? start + half : start - half;
  printId(id);
  Serial.print(F(" at "));
  Serial.print(start);
  Serial.print(F(", spinning 180 deg to "));
  Serial.println(target);

  if (target < (int)lo || target > (int)hi) {
    Serial.print(F("Target is outside the angle limits "));
    Serial.print(lo);
    Serial.print(F(" .. "));
    Serial.print(hi);
    Serial.println(F(", so the servo would stop at the limit."));
    Serial.println(F("Clear limits to 0 .. 4095? Only if the full 180 deg is safe for the mechanism."));
    if (!confirm()) {
      return;
    }
    writeLimits(id, 0, COUNTS_PER_REV - 1);
    printId(id);
    printLimits(id);
    Serial.println(F("Run l afterwards to teach the limits again."));
  } else if (!confirm()) {
    return;
  }

  st.EnableTorque(id, 1);
  st.WritePosEx(id, target, SPIN_SPEED, SPIN_ACC);
  delay(SPIN_MOVE_MS);
  int end;
  Serial.print(F("Done"));
  if (readPosition(id, end)) {
    Serial.print(F(", reads "));
    Serial.print(end);
  }
  Serial.println(F(". Torque left on; press t to release."));
}

// ---------------------------------------------------------------- diagnostics
// These don't rely on the library decoding a reply, so they still work when
// the bus is half broken. Each one leaves Serial2 at BUS_BAUD with TX driven.

static void printHex(byte b)
{
  if (b < 0x10) {
    Serial.print('0');
  }
  Serial.print(b, HEX);
}

static void drainBus()
{
  while (Serial2.read() != -1) {
  }
}

static void restoreBus()
{
  Serial2.begin(BUS_BAUD);
  drainBus();
}

static byte buildPing(byte id, byte pkt[6])
{
  pkt[0] = 0xFF;
  pkt[1] = 0xFF;
  pkt[2] = id;
  pkt[3] = 2;
  pkt[4] = INST_PING;
  pkt[5] = ~(byte)(id + 2 + INST_PING);
  return 6;
}

static byte sampleHighCount(byte pin)
{
  byte highs = 0;
  for (byte i = 0; i < 100; i++) {
    highs += digitalRead(pin);
    delayMicroseconds(50);
  }
  return highs;
}

static void diagLines()
{
  Serial.println(F("== Line levels, UART off, TX held high =="));
  // Drive TX2 high as a GPIO before the UART lets go of it, so it is never undriven.
  digitalWrite(TX_PIN, HIGH);
  pinMode(TX_PIN, OUTPUT);
  Serial2.end();

  pinMode(RX_PIN, INPUT);
  delay(5);
  Serial.print(F("No pullup:    RX high "));
  Serial.print(sampleHighCount(RX_PIN));
  Serial.println(F("/100  (100 = driver board holds idle high, in between = floating)"));

  pinMode(RX_PIN, INPUT_PULLUP);
  delay(5);
  Serial.print(F("With pullup:  RX high "));
  Serial.print(sampleHighCount(RX_PIN));
  Serial.println(F("/100  (0 = something actively pulls RX low)"));

  digitalWrite(TX_PIN, LOW);
  delay(1);
  byte lowRead = sampleHighCount(RX_PIN);
  digitalWrite(TX_PIN, HIGH);
  delay(1);
  byte highRead = sampleHighCount(RX_PIN);
  Serial.print(F("Drive TX LOW: RX high "));
  Serial.print(lowRead);
  Serial.print(F("/100   Drive TX HIGH: RX high "));
  Serial.print(highRead);
  Serial.println(F("/100"));
  if (lowRead == 0 && highRead == 100) {
    Serial.println(F("  RX follows TX: TX and RX are tied, expect an echo of every packet"));
  } else {
    Serial.println(F("  RX does not follow TX: separate lines (normal for the driver board)"));
  }
  restoreBus();
}

static void diagPassive()
{
  Serial.println(F("== Passive listen, 200 ms per baud, nothing transmitted =="));
  for (unsigned long baud : DIAG_BAUDS) {
    Serial2.begin(baud);
    delay(2);
    drainBus();
    unsigned int count = 0;
    byte first[8];
    unsigned long start = millis();
    while (millis() - start < 200) {
      int c = Serial2.read();
      if (c >= 0) {
        if (count < sizeof(first)) {
          first[count] = c;
        }
        count++;
      }
    }
    Serial.print(baud);
    Serial.print(F(": "));
    Serial.print(count);
    Serial.print(F(" bytes"));
    for (byte i = 0; i < count && i < sizeof(first); i++) {
      Serial.print(' ');
      printHex(first[i]);
    }
    Serial.println();
  }
  Serial.println(F("  any bytes here = something else is transmitting on the bus"));
  restoreBus();
}

static void diagRawPing()
{
  Serial.println(F("== Raw bytes for 5 ms after ping (E = matches our echo) =="));
  for (unsigned long baud : DIAG_BAUDS) {
    Serial2.begin(baud);
    for (byte id : ALL_IDS) {
      drainBus();
      byte pkt[6];
      buildPing(id, pkt);
      Serial2.write(pkt, sizeof(pkt));
      Serial2.flush();

      byte buf[32];
      byte n = 0;
      unsigned long start = micros();
      while (micros() - start < 5000 && n < sizeof(buf)) {
        int c = Serial2.read();
        if (c >= 0) {
          buf[n++] = c;
        }
      }

      bool echo = n >= sizeof(pkt) && memcmp(buf, pkt, sizeof(pkt)) == 0;
      Serial.print(baud);
      Serial.print(F(" ID"));
      Serial.print(id);
      Serial.print(F(": "));
      Serial.print(n);
      Serial.print(F(" bytes"));
      if (echo) {
        Serial.print(F(" [E]"));
      }
      for (byte i = echo ? sizeof(pkt) : 0; i < n; i++) {
        Serial.print(' ');
        printHex(buf[i]);
      }
      Serial.println();
    }
  }
  Serial.println(F("  a good reply looks like FF FF id 02 00 chk"));
  restoreBus();
}

static const uint16_t EDGE_SAMPLES = 4000;

struct EdgeResult {
  uint16_t edges;
  uint16_t minRun;
};

static EdgeResult captureEdges()
{
  EdgeResult r = {0, 0xFFFF};
  byte last = PINH & RX_BIT;
  uint16_t run = 0;
  for (uint16_t i = 0; i < EDGE_SAMPLES; i++) {
    byte v = PINH & RX_BIT;
    if (v != last) {
      if (r.edges > 0 && run < r.minRun) {
        r.minRun = run;
      }
      r.edges++;
      run = 0;
      last = v;
    } else {
      run++;
    }
  }
  return r;
}

static void diagEdges()
{
  Serial.println(F("== Edges on RX after ping, UART receiver bypassed =="));
  unsigned long t0 = micros();
  captureEdges();
  float usPerSample = (float)(micros() - t0) / EDGE_SAMPLES;
  Serial.print(F("Sample period ~"));
  Serial.print(usPerSample, 2);
  Serial.print(F(" us, window ~"));
  Serial.print(usPerSample * EDGE_SAMPLES / 1000.0, 1);
  Serial.println(F(" ms"));

  for (unsigned long baud : DIAG_BAUDS) {
    Serial2.begin(baud);
    for (byte id : ALL_IDS) {
      byte pkt[6];
      buildPing(id, pkt);
      Serial2.write(pkt, sizeof(pkt));
      Serial2.flush();
      // Only the receiver is switched off; the transmitter keeps TX2 driven high.
      UCSR2B &= ~_BV(RXEN2);
      noInterrupts();
      EdgeResult r = captureEdges();
      interrupts();
      UCSR2B |= _BV(RXEN2);

      Serial.print(baud);
      Serial.print(F(" ID"));
      Serial.print(id);
      Serial.print(F(": "));
      Serial.print(r.edges);
      Serial.print(F(" edges"));
      if (r.edges > 1) {
        float bitUs = (r.minRun + 1) * usPerSample;
        Serial.print(F(", shortest pulse ~"));
        Serial.print(bitUs, 1);
        Serial.print(F(" us (~"));
        Serial.print((unsigned long)(1000000.0 / bitUs));
        Serial.print(F(" baud)"));
      }
      Serial.println();
    }
  }
  Serial.println(F("  0 edges everywhere = nothing drives RX after our ping"));
  Serial.println(F("  edges but no bytes in raw ping = wrong baud or bad logic levels"));
  restoreBus();
}

static void diagMotion()
{
  Serial.println(F("== Motion: broadcast torque on + move +/-45 deg, every baud =="));
  Serial.println(F("Moves EVERY servo, ignoring replies. Watch the horns and note"));
  Serial.println(F("which baud line was printed when they moved."));
  if (!confirm()) {
    return;
  }
  for (unsigned long baud : DIAG_BAUDS) {
    Serial.print(F("Baud "));
    Serial.print(baud);
    Serial.println(F(": moving now"));
    Serial2.begin(baud);
    st.EnableTorque(BROADCAST_ID, 1);
    delay(10);
    st.WritePosEx(BROADCAST_ID, CENTER - 512, 1500, 50);
    delay(1200);
    st.WritePosEx(BROADCAST_ID, CENTER + 512, 1500, 50);
    delay(1200);
    st.WritePosEx(BROADCAST_ID, CENTER, 1500, 50);
    delay(1000);
  }
  Serial.println(F("  moved at 1000000 = TX path, power and baud are fine; fault is the reply path"));
  Serial.println(F("  moved at another baud = servo baud is not 1 Mbps"));
  Serial.println(F("  never moved = commands are not reaching the servos (wiring, power)"));
  restoreBus();
}

static void cmdDiagnose()
{
  Serial.println(F("Diagnostics: l lines | p passive | r raw ping | e edges | "
                   "m motion | a all but motion"));
  switch (waitForKey()) {
  case 'l': diagLines(); break;
  case 'p': diagPassive(); break;
  case 'r': diagRawPing(); break;
  case 'e': diagEdges(); break;
  case 'm': diagMotion(); break;
  case 'a':
    diagLines();
    diagPassive();
    diagRawPing();
    diagEdges();
    break;
  default: Serial.println(F("Cancelled.")); break;
  }
}

// ----------------------------------------------------------------

static void printMenu()
{
  Serial.println();
  Serial.println(F("s scan | q bus quality | t torque off | c calibrate centre | "
                   "l angle limits | j jog | w sweep | h spin 180 | p positions | "
                   "d diagnostics"));
}

void setup()
{
  Serial.begin(115200);
  digitalWrite(Z_EN_PIN, Z_ENABLED_LEVEL);
  pinMode(Z_EN_PIN, OUTPUT);
  Serial2.begin(BUS_BAUD);
  st.pSerial = &Serial2;
  st.IOTimeOut = 5;
  delay(1000);
  Serial.println(F("\n*** SCARA servo config ***"));
  Serial.println(F("Set IDs first with set-id.cpp (shoulder=1, elbow=2, gripper=3),"));
  Serial.println(F("then with everything connected: s, c, l, p, j"));
  printMenu();
}

void loop()
{
  switch (Serial.read()) {
  case 's': cmdScan(); break;
  case 'q': cmdQuality(); break;
  case 't': cmdTorqueOff(); break;
  case 'c': cmdCalibrate(); break;
  case 'l': cmdLimits(); break;
  case 'j': cmdJog(); break;
  case 'w': cmdSweep(); break;
  case 'h': cmdSpin(); break;
  case 'p': cmdPositions(); break;
  case 'd': cmdDiagnose(); break;
  default: return;
  }
  printMenu();
}
