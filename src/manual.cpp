#include <Arduino.h>
#include <SCServo.h>
#include <limits.h>
#include <math.h>
#include <util/atomic.h>

// SCARA Cartesian jog. Driven by host/jog.py over USB serial (115200).
//
// Table frame (used for V and P): origin at the shoulder axis, +Y is the
// base plate's forward edge direction, +X is to its right, viewed from above.
// The robot frame has +Y along the straight arm (shoulder at 0) and is turned
// BASE_YAW_DEG clockwise from the table frame.
// Joint angles are 0 when straight and positive CCW viewed from above; the
// elbow angle is relative to link 1. Straight = 2048 on each servo, set with
// config.cpp 'c', so XY needs no homing: the servos are absolute.
//
// XY jogging is resolved-rate (differential) IK: each tick solves a damped
// least squares step with the joint limits as box constraints. When the jog
// direction pushes a joint into its limit, the tool slides along the
// direction the other joint can still move instead of stopping.
//
// Z is 0 at the top limit switch and negative going down (carriage travel,
// plus Z_TOOL_OFFSET in reports).
//
// Commands (one per line):
//   V vx vy vz   jog velocity, mm/s, table frame. Resend at least every WATCHDOG_MS.
//   S            stop now
//   H            home Z (without a switch: current Z becomes the top)
//   F            swing the elbow to the other side, tool stays at the same XY
//   O            open the gripper
//   C            close the gripper
// Replies:
//   P x y z t1 t2   table-frame position (mm) and joint angles (deg), every REPORT_MS
//   I <text>        info
//   E <text>        error or limit hit

// ---------------------------------------------------------------- arm

static const float L1 = 90.72;  // shoulder axis to elbow axis, mm
static const float L2 = 111.01; // elbow axis to grip centre, mm

static const byte SHOULDER_ID = 1;
static const byte ELBOW_ID = 2;

// From the config.cpp jog test: +1 if "+" turns counter-clockwise viewed from above.
static const int SHOULDER_SIGN = 1;
static const int ELBOW_SIGN = 1;

static const float SHOULDER_LIMIT_DEG = 90.0;
static const float ELBOW_LIMIT_DEG = 85.0;
// Servo EEPROM limits written by config.cpp 'l' (95 and 90 deg).
static const int SHOULDER_EEPROM_COUNTS = 1081;
static const int ELBOW_EEPROM_COUNTS = 1024;

static const int CENTER = 2048;
static const float COUNTS_PER_DEG = 4096.0 / 360.0;

// Robot base yaw: clockwise from the table frame, viewed from above.
static const float BASE_YAW_DEG = 23.541508;

// Elbow side used when the arm starts within ELBOW_SIDE_BAND_DEG of straight
// (-1 = clockwise viewed from above). Starting inside the band swings to that
// side, bent by ELBOW_SIDE_BAND_DEG, with the F flip motion.
static const int PREFERRED_ELBOW_DIR = -1;
static const float ELBOW_SIDE_BAND_DEG = 5.0;
// Jogging keeps the elbow at least this bent. Dead straight, the arm can't
// move radially at all (singular), and could cross to the other side.
static const float ELBOW_MIN_BEND_DEG = 2.0;

// Per-joint speed cap for jogging; the whole step is scaled down to keep the
// tool's direction. Below the servos' ~300 deg/s.
static const float MAX_JOINT_SPEED_DEG = 240.0;
// Damped least squares: damping in mm (Jacobian units, mm/rad). Only bites
// near the straight-arm singularity, where it trades accuracy for bounded
// joint motion.
static const float DLS_LAMBDA = 4.0;
// Gauss-Newton passes per tick; the tool lands within IK_TOL_MM of the target.
static const byte IK_ITERATIONS = 3;
static const float IK_TOL_MM = 0.001;
// Elbow flip swing speed: 1.2 deg per 20 ms tick = 60 deg/s.
static const float FLIP_DEG_PER_TICK = 1.2;
// Speed 0 means full speed to the servo, so never send it.
static const u16 MIN_SERVO_SPEED = 50;   // counts/s
static const u16 MAX_SERVO_SPEED = 3400; // counts/s
static const float SERVO_SPEED_MARGIN = 1.25;

static const byte READ_TRIES = 5;

// ---------------------------------------------------------------- gripper

static const byte GRIPPER_ID = 3;
// Copy from the "For firmware:" line printed by config.cpp 'l'. While they
// are equal the gripper stays disabled.
static const int GRIPPER_OPEN = 3834;
static const int GRIPPER_CLOSE = 3409;
static const u16 GRIPPER_SPEED = 1000; // counts/s
static const byte GRIPPER_ACC = 50;

// ---------------------------------------------------------------- Z

static const byte Z_STEP_PIN = 2;
static const byte Z_DIR_PIN = 3;
// Flip if "up" moves the carriage down.
static const byte Z_UP_LEVEL = HIGH;

// Top limit switch, normally closed to GND, so a broken wire reads as pressed.
static const byte Z_HOME_PIN = 8;
static const bool Z_HOME_ENABLED = false;
static const byte Z_SWITCH_PRESSED = HIGH;

static const float LEAD_MM = 8.0;
static const float PULSES_PER_REV = 1600;
static const float STEPS_PER_MM = PULSES_PER_REV / LEAD_MM;
static const float Z_TRAVEL_MM = 150.0;
static const float Z_MAX_MM = 0.0;
static const float Z_MIN_MM = -Z_TRAVEL_MM;
// Table to grip point with the carriage at the top switch. Not measured yet.
static const float Z_TOOL_OFFSET = 0.0;

static const unsigned int STEP_PULSE_US = 5;
static const unsigned int DIR_SETUP_US = 10;

static const float Z_HOME_FAST = 3.0;    // mm/s
static const float Z_HOME_SLOW = 0.5;    // mm/s
static const float Z_HOME_BACKOFF = 2.0; // mm

// Timer1 at /8 prescale: 2 MHz, so 65535 ticks (~31 steps/s) is the slowest rate.
static const float TIMER_HZ = 2000000.0;
static const float MIN_STEP_RATE = 31.0;
static const uint16_t MIN_STEP_TICKS = 100;

// ---------------------------------------------------------------- jog

static const float XY_MAX_SPEED = 100.0; // mm/s
static const float XY_ACC = 600.0;       // mm/s^2
static const float Z_MAX_SPEED = 40.0;  // mm/s
static const float Z_ACC = 100.0;        // mm/s^2

static const unsigned long TICK_MS = 20;
static const unsigned long REPORT_MS = 100;
static const unsigned long WATCHDOG_MS = 300;

// ---------------------------------------------------------------- bus

// TX2 = pin 16, RX2 = pin 17. Don't call Serial2.end(): the driver board pulls
// an undriven TX line low and corrupts the bus.
static const unsigned long BUS_BAUD = 1000000;

SMS_STS st;

static u8 jointIds[2] = {SHOULDER_ID, ELBOW_ID};

// ---------------------------------------------------------------- state

struct Point {
  float x, y;
};

static bool ready = false;
static int elbowDir = 1;
static float curT1 = 0, curT2 = 0; // commanded joint angles, deg
static s16 curCounts[2] = {CENTER, CENTER};
static Point pos = {0, L1 + L2};

static float cmdVx = 0, cmdVy = 0, cmdVz = 0; // from the host, table frame
static float vx = 0, vy = 0, vz = 0;          // ramped
static unsigned long lastCmdMs = 0;
static bool xyBlocked = false;
static bool swinging = false;
static float swingT1 = 0, swingT2 = 0;
static int swingDir = 1;
static bool zBlocked = false;
static bool gripperReady = false;

static volatile long zSteps = 0;
static volatile long zMinSteps = lround(Z_MIN_MM * STEPS_PER_MM);
static volatile long zMaxSteps = lround(Z_MAX_MM * STEPS_PER_MM);
static volatile int8_t zDir = 0;
static int8_t zDirSet = 0;

// ---------------------------------------------------------------- kinematics

static float wrapPi(float a)
{
  while (a > PI) {
    a -= TWO_PI;
  }
  while (a < -PI) {
    a += TWO_PI;
  }
  return a;
}

// dir +1 / -1 picks which side the elbow bends to. No joint limit check.
static bool ikSolve(Point p, int dir, float &t1, float &t2)
{
  float c2 = (p.x * p.x + p.y * p.y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (c2 > 1.0 || c2 < -1.0) {
    return false;
  }
  float a2 = dir * acos(c2);
  float a1 = atan2(p.y, p.x) - HALF_PI - atan2(L2 * sin(a2), L1 + L2 * cos(a2));
  t1 = degrees(wrapPi(a1));
  t2 = degrees(a2);
  return true;
}

static Point fk(float t1, float t2)
{
  float a1 = radians(t1) + HALF_PI;
  float a12 = a1 + radians(t2);
  return {L1 * cos(a1) + L2 * cos(a12), L1 * sin(a1) + L2 * sin(a12)};
}

static const float BASE_COS = cos(radians(BASE_YAW_DEG));
static const float BASE_SIN = sin(radians(BASE_YAW_DEG));

static Point tableToRobot(Point p)
{
  return {BASE_COS * p.x - BASE_SIN * p.y, BASE_SIN * p.x + BASE_COS * p.y};
}

static Point robotToTable(Point p)
{
  return {BASE_COS * p.x + BASE_SIN * p.y, -BASE_SIN * p.x + BASE_COS * p.y};
}

// Tool distance from the shoulder axis for an elbow angle.
static float reachAt(float t2)
{
  return sqrt(L1 * L1 + L2 * L2 + 2 * L1 * L2 * cos(radians(t2)));
}

// Joint range for jogging, deg. The elbow stays on its locked side.
static void jogBounds(float lo[2], float hi[2])
{
  lo[0] = -SHOULDER_LIMIT_DEG;
  hi[0] = SHOULDER_LIMIT_DEG;
  if (elbowDir > 0) {
    lo[1] = ELBOW_MIN_BEND_DEG;
    hi[1] = ELBOW_LIMIT_DEG;
  } else {
    lo[1] = -ELBOW_LIMIT_DEG;
    hi[1] = -ELBOW_MIN_BEND_DEG;
  }
}

// One damped least squares step with box bounds on the joint change d (rad):
//   minimise |J d - e|^2 + DLS_LAMBDA^2 |d|^2  subject to  lo <= d <= hi
// The cost is convex, so the optimum is the free minimum if that is inside
// the box, else the best of the four edge minima. With two joints that
// enumeration is the exact active-set solution.
static void dlsBoxStep(const float J[2][2], const float e[2], const float lo[2],
                       const float hi[2], float d[2])
{
  float H[2][2], g[2];
  for (byte i = 0; i < 2; i++) {
    g[i] = J[0][i] * e[0] + J[1][i] * e[1];
    for (byte j = 0; j < 2; j++) {
      H[i][j] = J[0][i] * J[0][j] + J[1][i] * J[1][j];
    }
    H[i][i] += DLS_LAMBDA * DLS_LAMBDA;
  }
  float det = H[0][0] * H[1][1] - H[0][1] * H[1][0];
  d[0] = (H[1][1] * g[0] - H[0][1] * g[1]) / det;
  d[1] = (H[0][0] * g[1] - H[1][0] * g[0]) / det;
  if (d[0] >= lo[0] && d[0] <= hi[0] && d[1] >= lo[1] && d[1] <= hi[1]) {
    return;
  }
  float best = INFINITY;
  for (byte i = 0; i < 2; i++) {
    byte j = 1 - i;
    for (byte k = 0; k < 2; k++) {
      float di = k ? hi[i] : lo[i];
      float dj = constrain((g[j] - H[j][i] * di) / H[j][j], lo[j], hi[j]);
      float cost = 0.5 * (H[i][i] * di * di + 2 * H[i][j] * di * dj + H[j][j] * dj * dj) -
                   g[i] * di - g[j] * dj;
      if (cost < best) {
        best = cost;
        d[i] = di;
        d[j] = dj;
      }
    }
  }
}

// How far past the software limits, in total degrees. 0 = inside.
static float limitExcess(float t1, float t2)
{
  return max(0.0f, fabs(t1) - SHOULDER_LIMIT_DEG) + max(0.0f, fabs(t2) - ELBOW_LIMIT_DEG);
}

static s16 toCounts(float deg, int sign, int eepromCounts)
{
  long c = CENTER + lround(sign * deg * COUNTS_PER_DEG);
  return (s16)constrain(c, CENTER - eepromCounts, CENTER + eepromCounts);
}

// The side the elbow is physically on; dead straight counts as preferred.
static int elbowSideOf(float t2)
{
  if (t2 == 0) {
    return PREFERRED_ELBOW_DIR;
  }
  return t2 < 0 ? -1 : 1;
}

// ---------------------------------------------------------------- servos

static int readPosRetry(byte id)
{
  for (byte i = 0; i < READ_TRIES; i++) {
    int v = st.ReadPos(id);
    if (v >= 0) {
      return v;
    }
    delay(5);
  }
  return -1;
}

static int readWordRetry(byte id, byte addr)
{
  for (byte i = 0; i < READ_TRIES; i++) {
    int v = st.readWord(id, addr);
    if (v >= 0) {
      return v;
    }
    delay(5);
  }
  return -1;
}

// Streams one tick's worth of motion: speed is set so each joint arrives
// about when the next goal is sent.
static void sendJoints(float t1, float t2)
{
  s16 target[2] = {toCounts(t1, SHOULDER_SIGN, SHOULDER_EEPROM_COUNTS),
                   toCounts(t2, ELBOW_SIGN, ELBOW_EEPROM_COUNTS)};
  u16 speed[2];
  u8 acc[2] = {0, 0};
  for (byte i = 0; i < 2; i++) {
    float cps = abs(target[i] - curCounts[i]) * (1000.0 / TICK_MS) * SERVO_SPEED_MARGIN;
    speed[i] = (u16)constrain(cps, MIN_SERVO_SPEED, MAX_SERVO_SPEED);
    curCounts[i] = target[i];
  }
  st.SyncWritePosEx(jointIds, 2, target, speed, acc);
  curT1 = t1;
  curT2 = t2;
}

static bool initArm()
{
  int raw[2];
  for (byte i = 0; i < 2; i++) {
    raw[i] = readPosRetry(jointIds[i]);
    if (raw[i] < 0) {
      Serial.print(F("E servo ID "));
      Serial.print(jointIds[i]);
      Serial.println(F(" not responding. Check power and wiring, then reset."));
      return false;
    }
  }
  for (byte i = 0; i < 2; i++) {
    st.WritePosEx(jointIds[i], raw[i], MIN_SERVO_SPEED, 0);
    st.EnableTorque(jointIds[i], 1);
    curCounts[i] = raw[i];
  }
  curT1 = SHOULDER_SIGN * (raw[0] - CENTER) / COUNTS_PER_DEG;
  curT2 = ELBOW_SIGN * (raw[1] - CENTER) / COUNTS_PER_DEG;
  elbowDir = elbowSideOf(curT2);
  pos = fk(curT1, curT2);

  Serial.print(F("I start shoulder "));
  Serial.print(curT1, 1);
  Serial.print(F(" deg, elbow "));
  Serial.print(curT2, 1);
  Serial.print(F(" deg, elbow side "));
  Serial.println(elbowDir > 0 ? F("+") : F("-"));
  if (limitExcess(curT1, curT2) > 0) {
    Serial.println(F("E start pose is outside the software limits; jog back inside"));
  }
  return true;
}

// The arm still runs if this fails; O and C then reply with an error.
static void initGripper()
{
  if (GRIPPER_OPEN == GRIPPER_CLOSE) {
    Serial.println(F("E gripper positions not set; copy them from config.cpp 'l'"));
    return;
  }
  int raw = readPosRetry(GRIPPER_ID);
  if (raw < 0) {
    Serial.println(F("E gripper (ID 3) not responding"));
    return;
  }
  int lo = readWordRetry(GRIPPER_ID, SMS_STS_MIN_ANGLE_LIMIT_L);
  int hi = readWordRetry(GRIPPER_ID, SMS_STS_MAX_ANGLE_LIMIT_L);
  if (lo < 0 || hi < 0) {
    Serial.println(F("E gripper angle limit read failed"));
    return;
  }
  if (min(GRIPPER_OPEN, GRIPPER_CLOSE) < lo || max(GRIPPER_OPEN, GRIPPER_CLOSE) > hi) {
    Serial.println(F("E gripper open/close are outside its angle limits; re-run config.cpp 'l'"));
    return;
  }
  st.WritePosEx(GRIPPER_ID, raw, MIN_SERVO_SPEED, 0);
  st.EnableTorque(GRIPPER_ID, 1);
  gripperReady = true;
  Serial.println(F("I gripper ready"));
}

static void moveGripper(int target)
{
  if (!gripperReady) {
    Serial.println(F("E gripper not available"));
    return;
  }
  st.WritePosEx(GRIPPER_ID, target, GRIPPER_SPEED, GRIPPER_ACC);
}

// ---------------------------------------------------------------- Z stepper

static void initZTimer()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS11); // CTC, /8
    OCR1A = 65535;
    TCNT1 = 0;
    TIMSK1 = _BV(OCIE1A);
  }
}

ISR(TIMER1_COMPA_vect)
{
  int8_t d = zDir;
  if (d == 0) {
    return;
  }
  if (d > 0) {
    if (zSteps >= zMaxSteps) {
      return;
    }
    if (Z_HOME_ENABLED && digitalRead(Z_HOME_PIN) == Z_SWITCH_PRESSED) {
      return;
    }
  } else if (zSteps <= zMinSteps) {
    return;
  }
  digitalWrite(Z_STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(Z_STEP_PIN, LOW);
  zSteps += d;
}

static long readZSteps()
{
  long s;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    s = zSteps;
  }
  return s;
}

static void setZSteps(long s)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    zSteps = s;
  }
}

static void setZLimits(long lo, long hi)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    zMinSteps = lo;
    zMaxSteps = hi;
  }
}

// Signed steps/s, positive = up. The ISR enforces the soft limits.
static void setZRate(float stepsPerSec)
{
  float rate = fabs(stepsPerSec);
  int8_t dir = 0;
  if (rate >= MIN_STEP_RATE) {
    dir = stepsPerSec > 0 ? 1 : -1;
  }
  if (dir != zDirSet) {
    zDir = 0;
    if (dir != 0) {
      digitalWrite(Z_DIR_PIN, dir > 0 ? Z_UP_LEVEL : !Z_UP_LEVEL);
      delayMicroseconds(DIR_SETUP_US);
    }
    zDirSet = dir;
  }
  if (dir == 0) {
    return;
  }
  uint16_t top = (uint16_t)constrain(TIMER_HZ / rate, (float)MIN_STEP_TICKS, 65535.0f) - 1;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    OCR1A = top;
    // In CTC mode a compare value below the counter would wait for a full wrap.
    if (TCNT1 >= top) {
      TCNT1 = 0;
    }
    zDir = dir;
  }
}

static bool zSwitchPressed()
{
  return digitalRead(Z_HOME_PIN) == Z_SWITCH_PRESSED;
}

// Moves up until the switch closes. Fails after maxMm without finding it.
static bool seekSwitch(float speed, float maxMm)
{
  long start = readZSteps();
  long maxSteps = lround(maxMm * STEPS_PER_MM);
  setZRate(speed * STEPS_PER_MM);
  while (!zSwitchPressed()) {
    if (readZSteps() - start > maxSteps) {
      setZRate(0);
      return false;
    }
  }
  setZRate(0);
  return true;
}

static void moveZDown(float mm, float speed)
{
  long end = readZSteps() - lround(mm * STEPS_PER_MM);
  setZRate(-speed * STEPS_PER_MM);
  while (readZSteps() > end) {
  }
  setZRate(0);
}

static void homeZ()
{
  vz = 0;
  setZRate(0);
  if (!Z_HOME_ENABLED) {
    setZSteps(lround(Z_MAX_MM * STEPS_PER_MM));
    Serial.println(F("I no Z switch: current Z set as the top"));
    return;
  }
  Serial.println(F("I homing Z"));
  setZLimits(LONG_MIN, LONG_MAX);
  bool ok = seekSwitch(Z_HOME_FAST, Z_TRAVEL_MM + 10);
  if (ok) {
    moveZDown(Z_HOME_BACKOFF, Z_HOME_FAST);
    ok = seekSwitch(Z_HOME_SLOW, Z_HOME_BACKOFF * 2);
  }
  if (ok) {
    setZSteps(lround(Z_MAX_MM * STEPS_PER_MM));
    Serial.println(F("I Z homed"));
  } else {
    Serial.println(F("E Z switch not found; Z position unknown"));
  }
  setZLimits(lround(Z_MIN_MM * STEPS_PER_MM), lround(Z_MAX_MM * STEPS_PER_MM));
}

// ---------------------------------------------------------------- jog

static float approach(float from, float to, float maxDelta)
{
  if (to > from + maxDelta) {
    return from + maxDelta;
  }
  if (to < from - maxDelta) {
    return from - maxDelta;
  }
  return to;
}

// Resolved-rate step toward target (robot frame). Joints that would pass
// their jog bounds are pinned there and the rest of the motion is the least
// squares best, so the tool slides along the limit. A joint already outside
// its bounds may only move back toward them.
static bool stepToward(Point target)
{
  float lo[2], hi[2];
  jogBounds(lo, hi);
  float q0[2] = {radians(curT1), radians(curT2)};
  float q[2] = {q0[0], q0[1]};
  for (byte it = 0; it < IK_ITERATIONS; it++) {
    float a1 = q[0] + HALF_PI;
    float a12 = a1 + q[1];
    float c1 = L1 * cos(a1), s1 = L1 * sin(a1);
    float c12 = L2 * cos(a12), s12 = L2 * sin(a12);
    float e[2] = {target.x - (c1 + c12), target.y - (s1 + s12)};
    if (fabs(e[0]) < IK_TOL_MM && fabs(e[1]) < IK_TOL_MM) {
      break;
    }
    float J[2][2] = {{-s1 - s12, -s12}, {c1 + c12, c12}};
    float dlo[2], dhi[2], d[2];
    for (byte i = 0; i < 2; i++) {
      dlo[i] = min(0.0f, radians(lo[i]) - q[i]);
      dhi[i] = max(0.0f, radians(hi[i]) - q[i]);
    }
    dlsBoxStep(J, e, dlo, dhi, d);
    q[0] += d[0];
    q[1] += d[1];
  }

  float dq[2] = {degrees(q[0] - q0[0]), degrees(q[1] - q0[1])};
  float maxStep = MAX_JOINT_SPEED_DEG * TICK_MS / 1000.0;
  float far = max(fabs(dq[0]), fabs(dq[1]));
  if (far > maxStep) {
    dq[0] *= maxStep / far;
    dq[1] *= maxStep / far;
  }
  float t1 = curT1 + dq[0];
  float t2 = curT2 + dq[1];
  Point p = fk(t1, t2);
  float moved = hypot(p.x - pos.x, p.y - pos.y);
  float wanted = hypot(target.x - pos.x, target.y - pos.y);
  if (moved < 0.01 * wanted) {
    return false;
  }
  sendJoints(t1, t2);
  pos = p;
  return true;
}

static void jogXY(float dt)
{
  vx = approach(vx, cmdVx, XY_ACC * dt);
  vy = approach(vy, cmdVy, XY_ACC * dt);
  if (vx == 0 && vy == 0) {
    xyBlocked = false;
    return;
  }
  Point d = tableToRobot({vx * dt, vy * dt});
  bool moved = stepToward({pos.x + d.x, pos.y + d.y});
  if (!moved && !xyBlocked) {
    Serial.println(F("E XY limit"));
  }
  xyBlocked = !moved;
}

// Joint-space swing to elbow side dir. Keeps the tool's XY when the elbow is
// bent at least ELBOW_SIDE_BAND_DEG; closer to straight it also bends to that
// angle, keeping the tool's direction from the shoulder (moves < 0.2 mm).
static bool startSwing(int dir)
{
  Point p = pos;
  if (fabs(curT2) < ELBOW_SIDE_BAND_DEG) {
    float k = reachAt(ELBOW_SIDE_BAND_DEG) / hypot(pos.x, pos.y);
    p = {pos.x * k, pos.y * k};
  }
  float t1, t2;
  if (!ikSolve(p, dir, t1, t2) || limitExcess(t1, t2) > 0) {
    return false;
  }
  swingT1 = t1;
  swingT2 = t2;
  swingDir = dir;
  swinging = true;
  vx = vy = 0;
  return true;
}

static void startFlip()
{
  if (startSwing(-elbowDir)) {
    Serial.println(F("I flipping elbow"));
  } else {
    Serial.println(F("E other elbow side is outside the joint limits here"));
  }
}

static void stepSwing()
{
  float d1 = swingT1 - curT1;
  float d2 = swingT2 - curT2;
  float far = max(fabs(d1), fabs(d2));
  if (far <= FLIP_DEG_PER_TICK) {
    sendJoints(swingT1, swingT2);
    pos = fk(swingT1, swingT2);
    elbowDir = swingDir;
    swinging = false;
    Serial.print(F("I elbow side "));
    Serial.println(elbowDir > 0 ? F("+") : F("-"));
    return;
  }
  float k = FLIP_DEG_PER_TICK / far;
  sendJoints(curT1 + d1 * k, curT2 + d2 * k);
}

static void jogZ(float dt)
{
  float z = readZSteps() / STEPS_PER_MM;
  // Cap speed so it can still stop at Z_ACC before each soft limit.
  float upMax = sqrt(2 * Z_ACC * max(0.0f, Z_MAX_MM - z));
  float downMax = sqrt(2 * Z_ACC * max(0.0f, z - Z_MIN_MM));
  float target = constrain(cmdVz, -downMax, upMax);
  vz = approach(vz, target, Z_ACC * dt);
  setZRate(vz * STEPS_PER_MM);

  bool blocked = (cmdVz > 0 && upMax < 0.1) || (cmdVz < 0 && downMax < 0.1);
  if (blocked && !zBlocked) {
    Serial.println(F("E Z limit"));
  }
  zBlocked = blocked;
}

static void stopAll()
{
  cmdVx = cmdVy = cmdVz = 0;
  vx = vy = vz = 0;
  setZRate(0);
}

static void report()
{
  Point p = robotToTable(pos);
  Serial.print(F("P "));
  Serial.print(p.x, 2);
  Serial.print(' ');
  Serial.print(p.y, 2);
  Serial.print(' ');
  Serial.print(readZSteps() / STEPS_PER_MM + Z_TOOL_OFFSET, 2);
  Serial.print(' ');
  Serial.print(curT1, 1);
  Serial.print(' ');
  Serial.println(curT2, 1);
}

// ---------------------------------------------------------------- commands

static char line[48];
static byte lineLen = 0;

static void handleVelocity(char *s)
{
  char *end;
  float v[3];
  for (byte i = 0; i < 3; i++) {
    v[i] = strtod(s, &end);
    if (end == s) {
      Serial.println(F("E bad V command"));
      return;
    }
    s = end;
  }
  float mag = sqrt(v[0] * v[0] + v[1] * v[1]);
  float scale = mag > XY_MAX_SPEED ? XY_MAX_SPEED / mag : 1.0;
  cmdVx = v[0] * scale;
  cmdVy = v[1] * scale;
  cmdVz = constrain(v[2], -Z_MAX_SPEED, Z_MAX_SPEED);
  lastCmdMs = millis();
}

static void handleLine(char *s)
{
  switch (s[0]) {
  case 'V': handleVelocity(s + 1); break;
  case 'S':
    stopAll();
    if (swinging) {
      swinging = false;
      pos = fk(curT1, curT2);
      elbowDir = elbowSideOf(curT2);
      if (fabs(curT2) < ELBOW_MIN_BEND_DEG) {
        Serial.println(F("E elbow stopped nearly straight; press F to bend it"));
      }
    }
    break;
  case 'F':
    if (!swinging) {
      startFlip();
    }
    break;
  case 'H':
    stopAll();
    homeZ();
    break;
  case 'O': moveGripper(GRIPPER_OPEN); break;
  case 'C': moveGripper(GRIPPER_CLOSE); break;
  default: Serial.println(F("E unknown command")); break;
  }
}

static void readCommands()
{
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        line[lineLen] = 0;
        lineLen = 0;
        if (ready) {
          handleLine(line);
        }
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

// ----------------------------------------------------------------

static unsigned long lastTickMs = 0;
static unsigned long lastReportMs = 0;

void setup()
{
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  digitalWrite(Z_STEP_PIN, LOW);
  pinMode(Z_HOME_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial2.begin(BUS_BAUD);
  st.pSerial = &Serial2;
  st.IOTimeOut = 5;
  delay(1000);
  Serial.println(F("I *** SCARA jog ***"));

  initZTimer();
  if (!initArm()) {
    return;
  }
  initGripper();
  if (Z_HOME_ENABLED) {
    homeZ();
  } else {
    Serial.println(F("I no Z switch: assuming Z is at the top"));
  }
  ready = true;
  lastTickMs = millis();
  Serial.println(F("I ready"));
  if (fabs(curT2) < ELBOW_SIDE_BAND_DEG) {
    if (startSwing(PREFERRED_ELBOW_DIR)) {
      Serial.println(F("I bending elbow off straight"));
    } else {
      Serial.println(F("E can't bend elbow off straight here; jog or press F"));
    }
  }
}

void loop()
{
  readCommands();
  if (!ready) {
    return;
  }
  unsigned long now = millis();
  if (now - lastCmdMs > WATCHDOG_MS) {
    cmdVx = cmdVy = cmdVz = 0;
  }
  if (now - lastTickMs >= TICK_MS) {
    float dt = min((now - lastTickMs) / 1000.0, 0.05);
    lastTickMs = now;
    if (swinging) {
      stepSwing();
    } else {
      jogXY(dt);
    }
    jogZ(dt);
  }
  if (now - lastReportMs >= REPORT_MS) {
    lastReportMs = now;
    report();
  }
}
