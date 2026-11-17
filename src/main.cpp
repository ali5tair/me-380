#include <Arduino.h>
#include <EEPROM.h>
#include <SCServo.h>
#include <limits.h>
#include <math.h>
#include <util/atomic.h>

// SCARA pick-and-place demo, run from the PlatformIO serial monitor (115200).
// On power-up Z homes to the top. Then, in a loop: asks for the target box
// height (0.5, 2 or 4 in), picks the part off the table at PICK, places it on
// the box at PLACE, rises, does a little dance and asks again. Type s at any
// time during motion to stop everything.
//
// Frames and joint conventions are the same as manual.cpp: table frame XY in
// mm with the origin on the shoulder axis, +Y along the base plate's forward
// edge. Z is the carriage position, 0 at the top switch and negative going
// down; after homing it never goes above Z_CEILING_MM.
//
// Each move is planned before it starts: minimum-jerk profiles in XY and Z,
// overlapped where the box allows, then sampled to check reach, joint limits,
// joint speed and box clearance. A move that fails the checks is not run.
// XY moves arc around the shoulder (radius and angle interpolated): a straight
// line from PICK to PLACE would pass inside the arm's minimum reach.

struct Point {
  float x, y;
};

// ---------------------------------------------------------------- task

static const Point PICK = {133.73, 81.8};  // table frame, mm
// Box centre, table frame, mm. Shifted 15 mm toward the base along a line
// 14.155 deg CW of +Y (delta ≈ -3.67, -14.54).
static const Point PLACE = {-42.87, 154.96};

static const float MM_PER_INCH = 25.4;
static const float TARGET_HEIGHTS_IN[] = {0.5, 2.0, 4.0};
// The part is released this far above the box top.
static const float PLACE_LIFT_MM = 0.25 * MM_PER_INCH;

// Box footprint is square to the table frame.
static const float BOX_SIZE_MM = 76.2;
// Gripper body extent around the grip centre, added to the box footprint.
static const float GRIPPER_RADIUS_MM = 45.0;
// Gripper bottom above the box top whenever the gripper is over the box,
// except for the final vertical drop at PLACE.
static const float BOX_CLEARANCE_MM = 20.0;
static const float PLACE_XY_TOL_MM = 1.0;
// The final straight-down approach may start once XY is this close to its
// target. Below PLACE_XY_TOL_MM, since the planner estimates the distance
// left from the arc length.
static const float ARRIVE_TOL_MM = 0.5;
// Lowest travel height, so the gripper passes over the part at PICK.
static const float CRUISE_MIN_MM = 40.0;
// Straight up before moving sideways, and straight down for the last stretch.
static const float LIFT_CLEAR_MM = 30.0;
static const float PICK_APPROACH_MM = 40.0;
static const float PLACE_APPROACH_MM = 15.0;
// After the release the gripper rises to travel height, but at least this far
// above the top of the cube sitting on the box, before dancing over it.
static const float CUBE_HEIGHT_MM = 20.0;
static const float CUBE_CLEARANCE_MM = 10.0;

// Gripper waits end when the servo reports it has stopped (or stalled on the
// part), but no sooner than GRIP_MIN_MS and no later than GRIP_TIMEOUT_MS.
static const unsigned long GRIP_MIN_MS = 100;
static const unsigned long GRIP_TIMEOUT_MS = 800;
static const unsigned long GRIP_POLL_MS = 10;

// Wiggle dance: figure-8 in XY with a Z bob.
static const float WIGGLE_SIZE_MM = 30.0; // width; the height is half of it
static const float WIGGLE_LOOPS = 1;
static const float WIGGLE_TIME_S = 3.0;
// Easing from rest to the steady loop speed, and back.
static const float WIGGLE_RAMP_S = 0.75;
static const float WIGGLE_BOB_MM = 8.0;
// ---------------------------------------------------------------- motion

static const float XY_SPEED = 350.0;  // mm/s, peak
static const float XY_ACC = 1000.0;   // mm/s^2, peak
static const float Z_SPEED = 250.0;   // mm/s, peak
static const float Z_ACC = 300.0;    // mm/s^2, peak
// Joint-space move from the power-up pose.
static const float JOINT_SPEED_DEG = 60.0;
static const float JOINT_ACC_DEG = 200.0;
// Below the servos' ~300 deg/s.
static const float MAX_JOINT_SPEED_DEG = 240.0;

// Overlap of Z and XY motion is tried from full down to none in this many steps.
static const byte OVERLAP_STEPS = 8;
static const int VALIDATE_SAMPLES = 200;
static const byte SPEED_RETRIES = 4;
static const float SPEED_STRETCH = 1.05;
static const float CHECK_EPS_MM = 0.05;

static const unsigned long TICK_MS = 20;
static const unsigned long Z_TICK_MS = 5;
// Z steps toward where the trajectory will be this far ahead.
static const float Z_LEAD_S = 0.02;
static const float Z_RATE_MARGIN = 1.5;
// Servo goals are sent this far ahead along the trajectory.
static const float JOINT_LEAD_S = 0.04;
static const float Z_SETTLE_SPEED = 2.0; // mm/s
static const unsigned long Z_SETTLE_TIMEOUT_MS = 3000;
// After each move, both joints must read back within ARM_ARRIVE_COUNTS of the
// final goal before anything else happens; the remaining gap is closed in
// about ARM_CATCHUP_S.
static const int ARM_ARRIVE_COUNTS = 2;
static const float ARM_CATCHUP_S = 0.1;
static const unsigned long ARM_ARRIVE_TIMEOUT_MS = 600;
static const unsigned long ARM_POLL_MS = 10;
// A move that starts farther than this from where the arm is would jump.
static const float START_TOL_MM = 1.0;
static const float START_TOL_DEG = 1.0;

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

// Elbow side for every move (-1 = clockwise viewed from above).
static const int ELBOW_DIR = -1;
// Dead straight, the arm can't move radially at all (singular).
static const float ELBOW_MIN_BEND_DEG = 2.0;

// Speed 0 means full speed to the servo, so never send it.
static const u16 MIN_SERVO_SPEED = 50;   // counts/s
static const u16 MAX_SERVO_SPEED = 3400; // counts/s

static const byte READ_TRIES = 5;

// ---------------------------------------------------------------- gripper

static const byte GRIPPER_ID = 3;
// Taught by config.cpp 'l' into MCU EEPROM (same layout as config/manual).
static const int GRIPPER_EEPROM_ADDR = 0;
static const uint16_t GRIPPER_EEPROM_MAGIC = 0x4731; // 'G' '1'
struct GripperEeprom {
  uint16_t magic;
  uint16_t open;
  uint16_t close;
};
static int gripperOpen = 0;
static int gripperClose = 0;
static const u16 GRIPPER_SPEED = 1500; // counts/s
static const byte GRIPPER_ACC = 50;

// ---------------------------------------------------------------- Z

static const byte Z_STEP_PIN = 2;
static const byte Z_DIR_PIN = 3;
// Flip if "up" moves the carriage down.
static const byte Z_UP_LEVEL = HIGH;
// Driver ENABLE input, active low; must match Z_EN_PIN in config.cpp.
static const byte Z_EN_PIN = 4;
static const byte Z_ENABLED_LEVEL = LOW;

// Top limit switch, normally closed to GND, so a missing switch or broken
// wire reads as pressed. Homing tells the two cases apart.
static const byte Z_HOME_PIN = 8;
static const byte Z_SWITCH_PRESSED = HIGH;

static const float LEAD_MM = 8.0;
static const float PULSES_PER_REV = 1600;
static const float STEPS_PER_MM = PULSES_PER_REV / LEAD_MM;
static const float Z_TRAVEL_MM = 150.0;
static const float Z_MAX_MM = 0.0;
static const float Z_MIN_MM = -Z_TRAVEL_MM;
// Highest Z after homing, so motion never presses the switch.
static const float Z_CEILING_MM = -10.0;
// Gripper bottom above the table at Z_MAX_MM; at Z_MIN_MM it is on the table.
static const float GRIP_HEIGHT_AT_TOP_MM = Z_TRAVEL_MM;
// Switch contact point to gripper bottom, measured vertically.
static const float SWITCH_TO_GRIP_MM = 94.6;

static const unsigned int STEP_PULSE_US = 5;
static const unsigned int DIR_SETUP_US = 10;

static const float Z_HOME_SPEED = 5.0; // mm/s
// Step down this far with the switch reading pressed: if it is still
// pressed, there is no switch.
static const float Z_PROBE_MM = 3.0;

// Timer1 at /8 prescale: 2 MHz, so 65535 ticks (~31 steps/s) is the slowest rate.
static const float TIMER_HZ = 2000000.0;
static const float MIN_STEP_RATE = 31.0;
static const uint16_t MIN_STEP_TICKS = 100;

// ---------------------------------------------------------------- bus

// TX2 = pin 16, RX2 = pin 17. Don't call Serial2.end(): the driver board pulls
// an undriven TX line low and corrupts the bus.
static const unsigned long BUS_BAUD = 1000000;

SMS_STS st;

static u8 jointIds[2] = {SHOULDER_ID, ELBOW_ID};

// ---------------------------------------------------------------- state

static bool ready = false;
static bool armPlaced = false;
static bool gripperReady = false;
static float curT1 = 0, curT2 = 0; // commanded joint angles, deg
static Point curPos = {0, L1 + L2}; // table frame
static float curZ = 0;
static float boxTop = 0; // box top above the table, mm

static volatile long zSteps = 0;
static volatile long zMinSteps = LONG_MIN;
static volatile long zMaxSteps = LONG_MAX;
static volatile int8_t zDir = 0;
static volatile bool switchPresent = false;
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

static bool jointsInLimits(float t1, float t2)
{
  float bend = ELBOW_DIR * t2;
  return fabs(t1) <= SHOULDER_LIMIT_DEG && bend >= ELBOW_MIN_BEND_DEG &&
         bend <= ELBOW_LIMIT_DEG;
}

static s16 toCounts(float deg, int sign, int eepromCounts)
{
  long c = CENTER + lround(sign * deg * COUNTS_PER_DEG);
  return (s16)constrain(c, CENTER - eepromCounts, CENTER + eepromCounts);
}

static float heightOf(float z)
{
  return z + GRIP_HEIGHT_AT_TOP_MM;
}

static float zForHeight(float h)
{
  return h - GRIP_HEIGHT_AT_TOP_MM;
}

// Lowest allowed gripper bottom height above the table with the grip centre at p.
static float minHeightAt(Point p)
{
  float reach = BOX_SIZE_MM / 2 + GRIPPER_RADIUS_MM;
  float dx = p.x - PLACE.x;
  float dy = p.y - PLACE.y;
  if (fabs(dx) > reach || fabs(dy) > reach) {
    return 0;
  }
  if (hypot(dx, dy) <= PLACE_XY_TOL_MM) {
    return boxTop + PLACE_LIFT_MM;
  }
  return boxTop + BOX_CLEARANCE_MM;
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

// q is the trajectory JOINT_LEAD_S ahead of qNow. Each servo is sent q as its
// goal at the trajectory's own speed, so it is always still travelling when
// the next goal arrives instead of reaching each goal and pausing.
static void jointCounts(const float q[2], s16 counts[2])
{
  counts[0] = toCounts(q[0], SHOULDER_SIGN, SHOULDER_EEPROM_COUNTS);
  counts[1] = toCounts(q[1], ELBOW_SIGN, ELBOW_EEPROM_COUNTS);
}

static void sendJoints(const float q[2], const float qNow[2])
{
  s16 target[2];
  jointCounts(q, target);
  u16 speed[2];
  u8 acc[2] = {0, 0};
  for (byte i = 0; i < 2; i++) {
    float cps = fabs(q[i] - qNow[i]) * COUNTS_PER_DEG / JOINT_LEAD_S;
    speed[i] = (u16)constrain(cps, MIN_SERVO_SPEED, MAX_SERVO_SPEED);
  }
  st.SyncWritePosEx(jointIds, 2, target, speed, acc);
  curT1 = q[0];
  curT2 = q[1];
}

static bool initArm()
{
  int raw[2];
  for (byte i = 0; i < 2; i++) {
    raw[i] = readPosRetry(jointIds[i]);
    if (raw[i] < 0) {
      Serial.print(F("Servo ID "));
      Serial.print(jointIds[i]);
      Serial.println(F(" not responding. Check power and wiring, then reset."));
      return false;
    }
  }
  for (byte i = 0; i < 2; i++) {
    st.WritePosEx(jointIds[i], raw[i], MIN_SERVO_SPEED, 0);
    st.EnableTorque(jointIds[i], 1);
  }
  curT1 = SHOULDER_SIGN * (raw[0] - CENTER) / COUNTS_PER_DEG;
  curT2 = ELBOW_SIGN * (raw[1] - CENTER) / COUNTS_PER_DEG;
  curPos = robotToTable(fk(curT1, curT2));

  Serial.print(F("Arm at shoulder "));
  Serial.print(curT1, 1);
  Serial.print(F(" deg, elbow "));
  Serial.print(curT2, 1);
  Serial.println(F(" deg"));
  return true;
}

// The arm still runs if this fails, with the gripper left alone.
static void initGripper()
{
  int raw = readPosRetry(GRIPPER_ID);
  if (raw < 0) {
    Serial.println(F("Gripper (ID 3) not responding; running without it"));
    return;
  }
  int lo = readWordRetry(GRIPPER_ID, SMS_STS_MIN_ANGLE_LIMIT_L);
  int hi = readWordRetry(GRIPPER_ID, SMS_STS_MAX_ANGLE_LIMIT_L);
  if (lo < 0 || hi < 0) {
    Serial.println(F("Gripper angle limit read failed; running without it"));
    return;
  }
  GripperEeprom slot;
  EEPROM.get(GRIPPER_EEPROM_ADDR, slot);
  if (slot.magic != GRIPPER_EEPROM_MAGIC || slot.open == slot.close ||
      !((slot.open == (uint16_t)lo && slot.close == (uint16_t)hi) ||
        (slot.open == (uint16_t)hi && slot.close == (uint16_t)lo))) {
    Serial.println(F("Gripper open/close missing or mismatched; run config.cpp 'l'"));
    return;
  }
  gripperOpen = slot.open;
  gripperClose = slot.close;
  st.WritePosEx(GRIPPER_ID, raw, MIN_SERVO_SPEED, 0);
  st.EnableTorque(GRIPPER_ID, 1);
  gripperReady = true;
}

static void moveGripper(int target)
{
  if (gripperReady) {
    st.WritePosEx(GRIPPER_ID, target, GRIPPER_SPEED, GRIPPER_ACC);
  }
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

static bool zSwitchPressed()
{
  return digitalRead(Z_HOME_PIN) == Z_SWITCH_PRESSED;
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
    if (switchPresent && zSwitchPressed()) {
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

static void setTravelLimits()
{
  setZLimits(lround(Z_MIN_MM * STEPS_PER_MM), lround(Z_CEILING_MM * STEPS_PER_MM));
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

// ---------------------------------------------------------------- stop

static void halt()
{
  setZRate(0);
  Serial.println(F("Stopped. Reset the board to start again."));
  while (true) {
  }
}

static void pollAbort()
{
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      halt();
    }
  }
}

static void waitMs(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms) {
    pollAbort();
  }
}

// ---------------------------------------------------------------- homing

// Moves up until the switch closes. Fails after maxMm without finding it.
static bool seekSwitch(float speed, float maxMm)
{
  long start = readZSteps();
  long maxSteps = lround(maxMm * STEPS_PER_MM);
  setZRate(speed * STEPS_PER_MM);
  while (!zSwitchPressed()) {
    pollAbort();
    if (readZSteps() - start > maxSteps) {
      setZRate(0);
      return false;
    }
  }
  setZRate(0);
  return true;
}

// Blocking relative move, positive = up. Stops early if blocked.
static void moveZBy(float mm, float speed)
{
  long end = readZSteps() + lround(mm * STEPS_PER_MM);
  unsigned long timeoutMs = fabs(mm) / speed * 2000 + 500;
  unsigned long start = millis();
  setZRate((mm > 0 ? speed : -speed) * STEPS_PER_MM);
  while (mm > 0 ? readZSteps() < end : readZSteps() > end) {
    pollAbort();
    if (millis() - start > timeoutMs) {
      break;
    }
  }
  setZRate(0);
}

// Up to the first click of the switch, then down to Z_CEILING_MM.
static bool homeZ()
{
  setZLimits(LONG_MIN, LONG_MAX);
  switchPresent = true;
  if (zSwitchPressed()) {
    // Either already at the top, or no switch (which also reads pressed).
    switchPresent = false;
    moveZBy(-Z_PROBE_MM, Z_HOME_SPEED);
    switchPresent = !zSwitchPressed();
    if (!switchPresent) {
      moveZBy(Z_PROBE_MM, Z_HOME_SPEED);
      Serial.println(F("No Z switch: assuming Z started at the top"));
    }
  }
  if (switchPresent) {
    Serial.println(F("Homing Z"));
    if (!seekSwitch(Z_HOME_SPEED, Z_TRAVEL_MM + 10)) {
      Serial.println(F("Z switch not found; Z position unknown"));
      return false;
    }
  }
  setZSteps(lround(Z_MAX_MM * STEPS_PER_MM));
  setTravelLimits();
  moveZBy(Z_CEILING_MM - Z_MAX_MM, Z_HOME_SPEED);
  Serial.println(F("Z homed"));
  return true;
}

// ---------------------------------------------------------------- trajectories

// Minimum-jerk position fraction: zero velocity and acceleration at both ends.
static float minJerk(float u)
{
  return u * u * u * (10 + u * (-15 + 6 * u));
}

static float minJerkInv(float s)
{
  float lo = 0, hi = 1;
  for (byte i = 0; i < 24; i++) {
    float mid = (lo + hi) / 2;
    if (minJerk(mid) < s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return hi;
}

// Fraction done at time t of a min-jerk segment starting at t0.
static float mj(float t, float t0, float dur)
{
  if (t <= t0) {
    return 0;
  }
  if (dur <= 0 || t >= t0 + dur) {
    return 1;
  }
  return minJerk((t - t0) / dur);
}

// Shortest min-jerk duration over distance d within peak speed and acceleration.
static float durationFor(float d, float vmax, float amax)
{
  d = fabs(d);
  if (d < 0.01) {
    return 0;
  }
  return max(1.875 * d / vmax, sqrt(5.7735 * d / amax));
}

enum TrajKind : byte { TRANSFER, JOINT_MOVE, WIGGLE };

// TRANSFER: XY arcs a -> c around the shoulder; Z up za -> zs, then down zs -> zc.
// JOINT_MOVE: joints j0 -> j1 at constant za.
// WIGGLE: figure-8 around a, Z bobbing above za.
struct Traj {
  TrajKind kind;
  float T;
  Point a, c;
  float ra, rc, tha, dth; // polar a and c, table frame
  float za, zs, zc;
  float tUp, Tup, tXy, Txy, tDown, Tdown;
  float j0[2], j1[2];
};

// Two buffers, so a move can be planned before the one ahead of it has run.
static Traj trajA, trajB;

// Eases up to a steady loop speed over WIGGLE_RAMP_S, holds it, and eases back
// down, with continuous acceleration throughout, so every loop is traced at
// the same speed.
static float wigglePhase(const Traj &tr, float t)
{
  float total = TWO_PI * WIGGLE_LOOPS;
  float ramp = min(WIGGLE_RAMP_S, tr.T / 2);
  float w = total / (tr.T - ramp);
  if (t <= 0) {
    return 0;
  }
  if (t >= tr.T) {
    return total;
  }
  if (t < ramp) {
    float x = t / ramp;
    return w * ramp * x * x * x * (1 - x / 2);
  }
  if (t > tr.T - ramp) {
    float x = (tr.T - t) / ramp;
    return total - w * ramp * x * x * x * (1 - x / 2);
  }
  return w * (t - ramp / 2);
}

static Point trajXY(const Traj &tr, float t)
{
  if (tr.kind == WIGGLE) {
    float phi = wigglePhase(tr, t);
    float amp = WIGGLE_SIZE_MM / 2;
    return {tr.a.x + amp * sin(phi), tr.a.y + amp / 2 * sin(2 * phi)};
  }
  float s = mj(t, tr.tXy, tr.Txy);
  float r = tr.ra + (tr.rc - tr.ra) * s;
  float th = tr.tha + tr.dth * s;
  return {r * cos(th), r * sin(th)};
}

static float trajZ(const Traj &tr, float t)
{
  switch (tr.kind) {
  case TRANSFER:
    return tr.za + (tr.zs - tr.za) * mj(t, tr.tUp, tr.Tup) -
           (tr.zs - tr.zc) * mj(t, tr.tDown, tr.Tdown);
  case WIGGLE: return tr.za + WIGGLE_BOB_MM * (1 - cos(wigglePhase(tr, t))) / 2;
  default: return tr.za;
  }
}

static bool trajJoints(const Traj &tr, float t, float &t1, float &t2)
{
  if (tr.kind == JOINT_MOVE) {
    float s = mj(t, 0, tr.T);
    t1 = tr.j0[0] + (tr.j1[0] - tr.j0[0]) * s;
    t2 = tr.j0[1] + (tr.j1[1] - tr.j0[1]) * s;
    return true;
  }
  return ikSolve(tableToRobot(trajXY(tr, t)), ELBOW_DIR, t1, t2);
}

enum Check : byte { CHECK_OK, CHECK_REACH, CHECK_BOX, CHECK_SPEED };

// speedRatio > 1 means the fastest joint is that many times over its limit.
static Check checkTraj(const Traj &tr, float &speedRatio)
{
  int n = tr.T > 0 ? VALIDATE_SAMPLES : 0;
  float dt = n > 0 ? tr.T / n : 0;
  float prev[2] = {0, 0};
  float maxRate = 0;
  for (int i = 0; i <= n; i++) {
    float t = i * dt;
    float z = trajZ(tr, t);
    if (z < Z_MIN_MM - CHECK_EPS_MM || z > Z_CEILING_MM + CHECK_EPS_MM) {
      return CHECK_REACH;
    }
    float q[2];
    if (!trajJoints(tr, t, q[0], q[1])) {
      return CHECK_REACH;
    }
    // A joint move runs in a straight line in joint space, from wherever the
    // arm powered up, so only its end needs to be inside the limits.
    if (tr.kind != JOINT_MOVE && !jointsInLimits(q[0], q[1])) {
      return CHECK_REACH;
    }
    if (heightOf(z) < minHeightAt(robotToTable(fk(q[0], q[1]))) - CHECK_EPS_MM) {
      return CHECK_BOX;
    }
    if (i > 0) {
      maxRate = max(maxRate, max(fabs(q[0] - prev[0]), fabs(q[1] - prev[1])) / dt);
    }
    prev[0] = q[0];
    prev[1] = q[1];
  }
  speedRatio = maxRate / MAX_JOINT_SPEED_DEG;
  return speedRatio > 1 ? CHECK_SPEED : CHECK_OK;
}

static void reportCheck(Check r)
{
  switch (r) {
  case CHECK_REACH: Serial.println(F("Can't plan move: out of reach or past a joint limit")); break;
  case CHECK_BOX: Serial.println(F("Can't plan move: no path clears the box")); break;
  case CHECK_SPEED: Serial.println(F("Can't plan move: joints would be too fast")); break;
  default: break;
  }
}

// Stretches the whole trajectory until the joints are slow enough.
static bool finishPlan(Traj &tr)
{
  Check r = CHECK_SPEED;
  for (byte i = 0; i < SPEED_RETRIES; i++) {
    float ratio;
    r = checkTraj(tr, ratio);
    if (r != CHECK_SPEED) {
      break;
    }
    tr.T *= ratio * SPEED_STRETCH;
  }
  if (r != CHECK_OK) {
    reportCheck(r);
    return false;
  }
  return true;
}

// Where a trajectory finishes, table frame.
static void trajEnd(const Traj &tr, Point &p, float &z)
{
  float t1, t2;
  trajJoints(tr, tr.T, t1, t2);
  p = robotToTable(fk(t1, t2));
  z = trajZ(tr, tr.T);
}

// From the current joints, at constant Z.
static bool planJointMove(Traj &tr, float t1, float t2)
{
  tr.kind = JOINT_MOVE;
  tr.j0[0] = curT1;
  tr.j0[1] = curT2;
  tr.j1[0] = t1;
  tr.j1[1] = t2;
  tr.za = curZ;
  tr.T = durationFor(max(fabs(t1 - curT1), fabs(t2 - curT2)), JOINT_SPEED_DEG, JOINT_ACC_DEG);
  return finishPlan(tr);
}

static bool planWiggle(Traj &tr, Point centre, float z)
{
  tr.kind = WIGGLE;
  tr.a = centre;
  tr.za = z;
  tr.T = WIGGLE_TIME_S;
  return finishPlan(tr);
}

// From a at carriage height za to c at zc, travelling no lower than zs. The
// first liftClear of the climb and the last approach of the descent are
// straight up and down. XY starts as early as the climb allows and the descent
// as early as XY allows; if that cuts across the box, the overlap is reduced
// step by step down to plain up-across-down.
static bool planTransfer(Traj &tr, Point a, float za, Point c, float zc, float zs,
                         float liftClear, float approach)
{
  tr.kind = TRANSFER;
  tr.a = a;
  tr.za = za;
  tr.c = c;
  tr.zc = zc;
  tr.ra = hypot(tr.a.x, tr.a.y);
  tr.rc = hypot(c.x, c.y);
  tr.tha = atan2(tr.a.y, tr.a.x);
  tr.dth = wrapPi(atan2(c.y, c.x) - tr.tha);
  tr.zs = min(Z_CEILING_MM, max(zs, max(tr.za, zc + approach)));
  float up = tr.zs - tr.za;
  float down = tr.zs - tr.zc;
  tr.tUp = 0;
  tr.Tup = durationFor(up, Z_SPEED, Z_ACC);
  tr.Tdown = durationFor(down, Z_SPEED, Z_ACC);
  float tLift = up > 0 ? tr.Tup * minJerkInv(min(liftClear, up) / up) : 0;
  float tReach = down > approach ? tr.Tdown * minJerkInv((down - approach) / down) : 0;
  float arcLen = hypot(tr.rc - tr.ra, (tr.ra + tr.rc) / 2 * tr.dth);
  float xyTime = durationFor(arcLen, XY_SPEED, XY_ACC);
  // Fraction of the XY move after which it is within ARRIVE_TOL_MM of c.
  float nearFrac = arcLen > ARRIVE_TOL_MM ? minJerkInv(1 - ARRIVE_TOL_MM / arcLen) : 0;

  Check r = CHECK_BOX;
  for (int k = OVERLAP_STEPS; k >= 0; k--) {
    float overlap = (float)k / OVERLAP_STEPS;
    tr.Txy = xyTime;
    for (byte i = 0; i < SPEED_RETRIES; i++) {
      tr.tXy = tLift + (1 - overlap) * (tr.Tup - tLift);
      float xyEnd = tr.tXy + tr.Txy;
      float xyNear = tr.tXy + tr.Txy * nearFrac;
      // The descent never overlaps the climb, so the last `approach` of it
      // only starts once XY has all but arrived.
      tr.tDown = max(tr.Tup, xyNear - overlap * tReach);
      tr.T = max(xyEnd, tr.tDown + tr.Tdown);
      float ratio;
      r = checkTraj(tr, ratio);
      if (r == CHECK_OK) {
        return true;
      }
      if (r != CHECK_SPEED) {
        break;
      }
      tr.Txy *= ratio * SPEED_STRETCH;
    }
    if (r == CHECK_REACH) {
      break;
    }
  }
  reportCheck(r);
  return false;
}

// ---------------------------------------------------------------- execution

// Steps toward the trajectory Z_LEAD_S ahead: feed-forward of its velocity
// plus a correction for any lag.
static void trackZ(float zMm)
{
  float rate = (zMm * STEPS_PER_MM - readZSteps()) / Z_LEAD_S;
  float maxRate = Z_SPEED * Z_RATE_MARGIN * STEPS_PER_MM;
  setZRate(constrain(rate, -maxRate, maxRate));
}

// Creeps the last few steps, with a temporary soft limit at the target so the
// ISR stops exactly there.
static void settleZ(long target)
{
  long s = readZSteps();
  if (s == target) {
    return;
  }
  bool up = target > s;
  if (up) {
    setZLimits(lround(Z_MIN_MM * STEPS_PER_MM), target);
  } else {
    setZLimits(target, lround(Z_CEILING_MM * STEPS_PER_MM));
  }
  setZRate((up ? Z_SETTLE_SPEED : -Z_SETTLE_SPEED) * STEPS_PER_MM);
  unsigned long start = millis();
  while (readZSteps() != target && millis() - start < Z_SETTLE_TIMEOUT_MS) {
    pollAbort();
    if (up && switchPresent && zSwitchPressed()) {
      break;
    }
  }
  setZRate(0);
  setTravelLimits();
}

// The servos trail the trajectory, and its last goal goes out at the
// trajectory's near-zero end speed, so whatever gap is left would close at
// MIN_SERVO_SPEED. Re-sends that goal fast enough to close the gap, then waits
// until both joints read back there.
static void waitArm()
{
  float q[2] = {curT1, curT2};
  s16 target[2];
  jointCounts(q, target);
  u8 acc[2] = {0, 0};
  bool resent = false;
  unsigned long start = millis();
  while (millis() - start < ARM_ARRIVE_TIMEOUT_MS) {
    pollAbort();
    u16 speed[2];
    int worst = 0;
    bool read = true;
    for (byte i = 0; i < 2 && read; i++) {
      int pos = st.ReadPos(jointIds[i]);
      read = pos >= 0;
      int gap = abs(pos - target[i]);
      worst = max(worst, gap);
      speed[i] = (u16)constrain(gap / ARM_CATCHUP_S, MIN_SERVO_SPEED, MAX_SERVO_SPEED);
    }
    if (read && worst <= ARM_ARRIVE_COUNTS) {
      return;
    }
    if (read && !resent) {
      st.SyncWritePosEx(jointIds, 2, target, speed, acc);
      resent = true;
    }
    waitMs(ARM_POLL_MS);
  }
}

static void runTraj(const Traj &tr)
{
  float q0[2];
  if (fabs(trajZ(tr, 0) - curZ) > START_TOL_MM || !trajJoints(tr, 0, q0[0], q0[1]) ||
      fabs(q0[0] - curT1) > START_TOL_DEG || fabs(q0[1] - curT2) > START_TOL_DEG) {
    Serial.println(F("Move doesn't start where the arm is"));
    halt();
  }
  unsigned long t0 = millis();
  unsigned long lastTick = t0 - TICK_MS;
  unsigned long lastZ = t0 - Z_TICK_MS;
  while (true) {
    pollAbort();
    unsigned long now = millis();
    float t = (now - t0) / 1000.0;
    if (now - lastZ >= Z_TICK_MS) {
      lastZ = now;
      trackZ(trajZ(tr, t + Z_LEAD_S));
    }
    if (now - lastTick >= TICK_MS) {
      lastTick = now;
      float q[2], qNow[2];
      if (trajJoints(tr, min(t + JOINT_LEAD_S, tr.T), q[0], q[1]) &&
          trajJoints(tr, min(t, tr.T), qNow[0], qNow[1])) {
        sendJoints(q, qNow);
      }
      if (t >= tr.T) {
        break;
      }
    }
  }
  setZRate(0);
  settleZ(lround(trajZ(tr, tr.T) * STEPS_PER_MM));
  waitArm();
  curPos = robotToTable(fk(curT1, curT2));
  curZ = readZSteps() / STEPS_PER_MM;
}

static void runMove(const __FlashStringHelper *what, const Traj &tr)
{
  Serial.print(what);
  Serial.print(F(" ("));
  Serial.print(tr.T, 1);
  Serial.println(F(" s)"));
  runTraj(tr);
}

// ---------------------------------------------------------------- gripper waits

static unsigned long gripStartMs = 0;

// The next move is planned between startGripper and waitGripper, so the
// planning time overlaps the gripper's travel.
static void startGripper(int target)
{
  moveGripper(target);
  gripStartMs = millis();
}

static void waitGripper()
{
  if (!gripperReady) {
    return;
  }
  while (millis() - gripStartMs < GRIP_MIN_MS) {
    pollAbort();
  }
  while (millis() - gripStartMs < GRIP_TIMEOUT_MS) {
    pollAbort();
    if (st.ReadMove(GRIPPER_ID) == 0) {
      return;
    }
    waitMs(GRIP_POLL_MS);
  }
}

// ---------------------------------------------------------------- cycle

static void flushInput()
{
  delay(20);
  while (Serial.read() != -1) {
  }
}

static void readLine(char *buf, byte size)
{
  byte n = 0;
  while (true) {
    int c = Serial.read();
    if (c < 0) {
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (n > 0) {
        break;
      }
      continue;
    }
    if (n < size - 1) {
      buf[n++] = c;
    }
  }
  buf[n] = 0;
}

// Returns the box top above the table, mm.
static float promptHeight()
{
  flushInput();
  while (true) {
    Serial.println();
    Serial.println(F("Target height in inches? Type 0.5, 2 or 4, then Enter."));
    char buf[16];
    readLine(buf, sizeof(buf));
    char *end;
    float v = strtod(buf, &end);
    if (end != buf) {
      for (float h : TARGET_HEIGHTS_IN) {
        if (fabs(v - h) < 0.01) {
          Serial.print(F("Target height "));
          Serial.print(h, 1);
          Serial.println(F(" in"));
          return h * MM_PER_INCH;
        }
      }
    }
    Serial.print(F("Not one of the heights: "));
    Serial.println(buf);
  }
}

static float cruiseHeightZ()
{
  return zForHeight(max(boxTop + BOX_CLEARANCE_MM, CRUISE_MIN_MM));
}

// The first time, swings in joint space from the power-up pose to above the
// pick point (into trajA, at the current Z), then plans the descent onto the
// part into trajB. Climbs to travel height first only if starting over the box.
static bool planToPick()
{
  Point from = curPos;
  float fromZ = curZ;
  float travelZ = minHeightAt(from) > 0 ? cruiseHeightZ() : fromZ;
  if (!armPlaced) {
    float t1, t2;
    if (!ikSolve(tableToRobot(PICK), ELBOW_DIR, t1, t2) || !jointsInLimits(t1, t2)) {
      reportCheck(CHECK_REACH);
      return false;
    }
    if (!planJointMove(trajA, t1, t2)) {
      return false;
    }
    trajEnd(trajA, from, fromZ);
  }
  return planTransfer(trajB, from, fromZ, PICK, Z_MIN_MM, travelZ, LIFT_CLEAR_MM,
                      PICK_APPROACH_MM);
}

static bool runCycle()
{
  float placeZ = zForHeight(boxTop + PLACE_LIFT_MM);
  Point end;
  float endZ;

  Serial.println(F("Opening gripper"));
  startGripper(gripperOpen);
  if (!planToPick()) {
    return false;
  }
  waitGripper();
  if (!armPlaced) {
    runMove(F("Swinging over the pick point"), trajA);
    armPlaced = true;
  }
  runMove(F("Moving down to the part"), trajB);

  Serial.println(F("Gripping"));
  startGripper(gripperClose);
  // The carry stops PLACE_APPROACH_MM above the drop, so the arm has settled
  // over the target before the straight-down placement starts.
  float hoverZ = placeZ + PLACE_APPROACH_MM;
  trajEnd(trajB, end, endZ);
  if (!planTransfer(trajA, end, endZ, PLACE, hoverZ, cruiseHeightZ(), LIFT_CLEAR_MM, 0) ||
      !planTransfer(trajB, PLACE, hoverZ, PLACE, placeZ, hoverZ, 0, 0)) {
    return false;
  }
  waitGripper();
  runMove(F("Carrying to the box"), trajA);
  runMove(F("Lowering onto the box"), trajB);

  Serial.println(F("Releasing"));
  startGripper(gripperOpen);
  float retreatZ = max(trajA.zs, zForHeight(boxTop + CUBE_HEIGHT_MM + CUBE_CLEARANCE_MM));
  if (!planTransfer(trajB, curPos, curZ, curPos, retreatZ, retreatZ, 0, 0)) {
    return false;
  }
  trajEnd(trajB, end, endZ);
  if (!planWiggle(trajA, end, endZ)) {
    return false;
  }
  waitGripper();
  runMove(F("Rising"), trajB);
  runMove(F("Wiggle"), trajA);

  // Parks over the pick point, out of the way of the box, for the next cycle.
  trajEnd(trajA, end, endZ);
  if (!planTransfer(trajB, end, endZ, PICK, endZ, endZ, 0, 0)) {
    return false;
  }
  runMove(F("Parking over the pick point"), trajB);
  return true;
}

// ----------------------------------------------------------------

void setup()
{
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  digitalWrite(Z_STEP_PIN, LOW);
  digitalWrite(Z_EN_PIN, Z_ENABLED_LEVEL);
  pinMode(Z_EN_PIN, OUTPUT);
  pinMode(Z_HOME_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial2.begin(BUS_BAUD);
  st.pSerial = &Serial2;
  st.IOTimeOut = 5;
  delay(1000);
  Serial.println(F("\n*** SCARA pick and place ***"));
  Serial.println(F("Type s at any time during motion to stop."));

  initZTimer();
  if (!initArm()) {
    return;
  }
  initGripper();
  if (!homeZ()) {
    return;
  }
  curZ = readZSteps() / STEPS_PER_MM;
  ready = true;
}

void loop()
{
  if (!ready) {
    return;
  }
  boxTop = promptHeight();
  if (!runCycle()) {
    Serial.println(F("Cycle cancelled"));
  }
}
