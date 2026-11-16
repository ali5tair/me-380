#include <SCServo.h>

// Sets the ID of the ONE servo on the bus. The ID write is broadcast, so with
// several servos connected they would all get the same ID. New STS3215s ship
// as ID 1. Use shoulder = 1, elbow = 2, gripper = 3.
//
// Type the new ID in the serial monitor (115200) and press Enter.

static const unsigned long BUS_BAUD = 1000000;
static const byte BROADCAST = 0xFE;
static const byte REG_ID = 5;
static const byte REG_LOCK = 55;
static const int MAX_ID = 253;
// New servos ship as ID 1, so a short scan keeps each attempt fast.
static const int SCAN_MAX_ID = 20;
static const unsigned long SCAN_TIMEOUT_MS = 2;
static const unsigned long BUS_TIMEOUT_MS = 5;

SMS_STS st;

static void writeByte(byte id, byte addr, byte value)
{
  byte sum = id + 4 + INST_WRITE + addr + value;
  byte pkt[8] = {0xFF, 0xFF, id, 4, INST_WRITE, addr, value, (byte)~sum};
  Serial2.write(pkt, sizeof(pkt));
  Serial2.flush();
}

static void flushInput()
{
  delay(20);
  while (Serial.read() != -1) {
  }
}

// Reads one line; returns -1 unless it is only digits.
static long readNumber()
{
  char buf[8];
  byte n = 0;
  bool digitsOnly = true;
  while (true) {
    int c = Serial.read();
    if (c < 0) {
      continue;
    }
    if (c == '\n' || c == '\r') {
      break;
    }
    if (c < '0' || c > '9') {
      digitsOnly = false;
    }
    if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
  flushInput();
  buf[n] = 0;
  return n > 0 && digitsOnly ? atol(buf) : -1;
}

// Returns how many servos answered; firstId is the lowest one found.
static int scan(int &firstId)
{
  int found = 0;
  firstId = -1;
  st.IOTimeOut = SCAN_TIMEOUT_MS;
  for (int id = 0; id <= SCAN_MAX_ID; id++) {
    if (st.Ping(id) != id) {
      continue;
    }
    if (found == 0) {
      firstId = id;
    }
    found++;
    Serial.print(F("  found ID "));
    Serial.println(id);
  }
  st.IOTimeOut = BUS_TIMEOUT_MS;
  return found;
}

static void prompt()
{
  Serial.println();
  Serial.println(F("Connect ONE servo, type the new ID (1 shoulder, 2 elbow, 3 gripper), Enter."));
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(BUS_BAUD);
  st.pSerial = &Serial2;
  st.IOTimeOut = BUS_TIMEOUT_MS;
  delay(1000);
  Serial.println(F("\n*** STS3215 set ID ***"));
  prompt();
}

void loop()
{
  if (!Serial.available()) {
    return;
  }
  long newId = readNumber();
  if (newId < 1 || newId > MAX_ID) {
    Serial.println(F("Type a number 1-253."));
    prompt();
    return;
  }

  Serial.println(F("Scanning bus..."));
  int oldId;
  int found = scan(oldId);
  if (found == 0) {
    Serial.println(F("No servo answered. Check power and wiring."));
    prompt();
    return;
  }
  if (found > 1) {
    Serial.println(F("More than one servo on the bus. Disconnect all but one."));
    prompt();
    return;
  }

  writeByte(BROADCAST, REG_LOCK, 0);
  delay(10);
  writeByte(BROADCAST, REG_ID, newId);
  delay(50);
  writeByte(BROADCAST, REG_LOCK, 1);
  delay(50);

  Serial.print(F("ID "));
  Serial.print(oldId);
  Serial.print(F(" -> "));
  Serial.print(newId);
  Serial.println(st.Ping(newId) == newId ? F(": ok") : F(": no reply at the new ID"));
  prompt();
}
