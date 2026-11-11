// Arduino Mega 2560 TMS 시퀀스 제어기 (복원본)
//
// 보드에서 읽어낸 fixed_v4 펌웨어(2026-11-09 readback)를 역공학해 다시 작성했다.
// 변수와 함수 이름은 복원하면서 새로 붙였고, 동작과 시리얼 출력은 원본에 맞췄다.
// 원본과 다른 부분은 runState()의 유지 시간 계산 한 곳뿐이다.

#include <Servo.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// 핀
// ---------------------------------------------------------------------------
const uint8_t SERVO_PIN = 9;
const uint8_t ESTOP_PIN = 2;   // INPUT_PULLUP. LOW=정상, HIGH=ABORT
const uint8_t SIREN_PIN = 22;
const uint8_t IGN_PIN   = 23;

// ---------------------------------------------------------------------------
// 서보
// ---------------------------------------------------------------------------
const int SERVO_CLOSED_DEG = 60;   // 1162 us
const int SERVO_LOW_DEG    = 80;   // 1368 us
const int SERVO_FULL_DEG   = 120;  // 1781 us
const int SERVO_MIN_DEG    = 57;   // 1131 us
const int SERVO_MAX_DEG    = 123;  // 1812 us

const float SERVO_SPEED_DPS = 90000000.0f;  // 원본 값. 사실상 속도 제한이 없다.
const float TARGET_TOL_DEG  = 0.5f;
const unsigned long SERVO_UPDATE_US = 5000;

// ---------------------------------------------------------------------------
// 시퀀스
// ---------------------------------------------------------------------------
enum {
  ST_IDLE,
  ST_PRE_LOG,
  ST_SIREN,
  ST_WAIT,
  ST_IGNITION,
  ST_LOW_FLOW,
  ST_FLOW_INC,
  ST_FULL_FLOW,
  ST_CLOSED,
  ST_POST_LOG,
  ST_END,
  ST_ABORT
};

const uint16_t PRE_LOG_MS   = 5000;
const uint16_t SIREN_MS     = 15000;
const uint16_t WAIT_MS      = 12000;
const uint16_t IGNITION_MS  = 3000;
const uint16_t LOW_HOLD_MS  = 7000;
const uint16_t FULL_HOLD_MS = 15000;
const uint16_t POST_LOG_MS  = 10000;

// 상태 1(PRE_LOG)부터 10(END)까지의 시간. 인덱스는 상태 번호 - 1.
const uint16_t STATE_DURATION_MS[] = {
  PRE_LOG_MS, SIREN_MS, WAIT_MS, IGNITION_MS, LOW_HOLD_MS,
  0, FULL_HOLD_MS, 0, POST_LOG_MS, 0
};

const char* const STATE_NAMES[] = {
  "IDLE", "PRE_LOG", "SIREN", "WAIT", "IGNITION", "LOW_FLOW",
  "FLOW_INC", "FULL_FLOW", "CLOSED", "POST_LOG", "END", "ABORT"
};

const char* const NEXT_STATE_NAMES[] = {
  "PRE_LOG", "SIREN", "WAIT", "IGNITION", "LOW_FLOW", "FLOW_INC",
  "FULL_FLOW", "CLOSED", "POST_LOG", "END", "RESET_REQUIRED", "RESET_REQUIRED"
};

// ---------------------------------------------------------------------------
// 통신
// ---------------------------------------------------------------------------
const unsigned long SERIAL_BAUD = 115200;
const unsigned long STATUS_PERIOD_MS = 100;
const int HOLD_AFTER_TARGET = 1;

const uint8_t MANUAL_CMD_MAX = 39;

// ---------------------------------------------------------------------------
// 전역 상태
// ---------------------------------------------------------------------------
Servo valveServo;

int state = ST_IDLE;
unsigned long stateEnteredMs = 0;
bool goAccepted = false;

float currentDeg = SERVO_CLOSED_DEG;
float targetDeg = SERVO_CLOSED_DEG;
int lastServoDeg = SERVO_CLOSED_DEG;
unsigned long lastServoUs = 0;

bool atTarget = false;
unsigned long arrivedMs = 0;

bool sirenOn = false;
bool igniterOn = false;

unsigned long lastStatusMs = 0;

char manualCmd[MANUAL_CMD_MAX + 1];
uint8_t manualLen = 0;
bool inManualCmd = false;
bool discardLine = false;

// ---------------------------------------------------------------------------
// 출력 제어
// ---------------------------------------------------------------------------
void setIgniter(bool on) {
  igniterOn = on;
  digitalWrite(IGN_PIN, on ? HIGH : LOW);
}

void setSiren(bool on) {
  sirenOn = on;
  digitalWrite(SIREN_PIN, on ? HIGH : LOW);
}

void outputsOff() {
  setSiren(false);
  setIgniter(false);
}

void setTargetClosed() {
  targetDeg = SERVO_CLOSED_DEG;
}

// 속도 제한을 거치지 않고 밸브를 바로 닫는다.
void forceClosedNow() {
  targetDeg = SERVO_CLOSED_DEG;
  currentDeg = SERVO_CLOSED_DEG;
  lastServoDeg = SERVO_CLOSED_DEG;
  valveServo.write(SERVO_CLOSED_DEG);
  lastServoUs = micros();
}

void safeState() {
  outputsOff();
  forceClosedNow();
}

bool estopActive() {
  return digitalRead(ESTOP_PIN) == HIGH;
}

// ---------------------------------------------------------------------------
// 서보 이동
// ---------------------------------------------------------------------------
void updateServo() {
  const unsigned long nowUs = micros();

  if (lastServoUs == 0) {
    lastServoUs = nowUs;
    return;
  }

  const unsigned long dtUs = nowUs - lastServoUs;
  if (dtUs < SERVO_UPDATE_US) {
    return;
  }
  lastServoUs = nowUs;

  const float maxStep = ((float)dtUs / 1000000.0f) * SERVO_SPEED_DPS;
  const float diff = targetDeg - currentDeg;

  if (fabs(diff) > maxStep) {
    currentDeg += (diff > 0.0f) ? maxStep : -maxStep;
  } else {
    currentDeg = targetDeg;
  }

  const long rounded = lround(currentDeg);
  const int commandDeg = (int)constrain(rounded, (long)SERVO_MIN_DEG, (long)SERVO_MAX_DEG);

  if (commandDeg != lastServoDeg) {
    lastServoDeg = commandDeg;
    valveServo.write(commandDeg);
  }
}

void trackTarget(float deg) {
  if (fabs(deg - currentDeg) > TARGET_TOL_DEG) {
    atTarget = false;
    arrivedMs = 0;
    return;
  }
  if (!atTarget) {
    atTarget = true;
    arrivedMs = millis();
  }
}

// ---------------------------------------------------------------------------
// 시리얼 출력
// ---------------------------------------------------------------------------
void printEvent(const __FlashStringHelper* message) {
  Serial.print(F("EVENT|"));
  Serial.println(message);
}

void printError(const __FlashStringHelper* code, const __FlashStringHelper* detail) {
  Serial.print(F("ERROR|"));
  Serial.print(code);
  Serial.print(F("|"));
  Serial.println(detail);
}

void printManualEvent(const char* name, const char* value) {
  Serial.print(F("EVENT|MANUAL_"));
  Serial.print(name);
  Serial.print(F("="));
  Serial.println(value);
}

void printConfig() {
  Serial.print(F("CONFIG|schema=1|closed_deg="));
  Serial.print(SERVO_CLOSED_DEG);
  Serial.print(F("|low_deg="));
  Serial.print(SERVO_LOW_DEG);
  Serial.print(F("|full_deg="));
  Serial.print(SERVO_FULL_DEG);
  Serial.print(F("|servo_min_deg="));
  Serial.print(SERVO_MIN_DEG);
  Serial.print(F("|servo_max_deg="));
  Serial.print(SERVO_MAX_DEG);
  Serial.print(F("|speed_dps="));
  Serial.print(SERVO_SPEED_DPS, 3);
  Serial.print(F("|hold_after_target="));
  Serial.print(HOLD_AFTER_TARGET);
  Serial.print(F("|target_tol_deg="));
  Serial.print(TARGET_TOL_DEG, 3);
  Serial.print(F("|status_period_ms="));
  Serial.println(STATUS_PERIOD_MS);

  Serial.print(F("SEQDEF|order="));
  for (int i = ST_IDLE; i <= ST_END; i++) {
    Serial.print(STATE_NAMES[i]);
    if (i != ST_END) {
      Serial.print(',');
    }
  }
  Serial.print(F("|durations_ms="));
  for (int i = ST_IDLE; i <= ST_END; i++) {
    Serial.print(STATE_NAMES[i]);
    Serial.print(':');
    Serial.print(i >= ST_PRE_LOG ? (unsigned long)STATE_DURATION_MS[i - 1] : 0UL);
    if (i != ST_END) {
      Serial.print(',');
    }
  }
  Serial.print(F("|fixed_timer=PRE_LOG,SIREN,WAIT,IGNITION,POST_LOG|hold_timer=LOW_FLOW,FULL_FLOW"));
  Serial.println(F("|motion=FLOW_INC,CLOSED"));
}

void printStatus(bool force) {
  const unsigned long nowMs = millis();
  if (!force && nowMs - lastStatusMs < STATUS_PERIOD_MS) {
    return;
  }
  lastStatusMs = nowMs;

  const unsigned long stateElapsedMs = nowMs - stateEnteredMs;

  unsigned long timerElapsedMs = stateElapsedMs;
  if (state == ST_LOW_FLOW || state == ST_FULL_FLOW) {
    timerElapsedMs = atTarget ? nowMs - arrivedMs : 0;
  }

  unsigned long remainingMs = 0;
  if (state >= ST_PRE_LOG && state <= ST_END) {
    const unsigned long durationMs = STATE_DURATION_MS[state - 1];
    if (timerElapsedMs < durationMs) {
      remainingMs = durationMs - timerElapsedMs;
    }
  }

  Serial.print(F("STATUS|"));
  Serial.print(F("current="));
  Serial.print(STATE_NAMES[state]);
  Serial.print(F("|next="));
  Serial.print(NEXT_STATE_NAMES[state]);
  Serial.print(F("|state_elapsed_ms="));
  Serial.print(stateElapsedMs);
  Serial.print(F("|timer_elapsed_ms="));
  Serial.print(timerElapsedMs);
  Serial.print(F("|remaining_ms="));
  Serial.print(remainingMs);
  Serial.print(F("|estop="));
  Serial.print(estopActive() ? F("HIGH") : F("LOW"));
  Serial.print(F("|abort="));
  Serial.print(state == ST_ABORT ? F("1") : F("0"));
  Serial.print(F("|siren_led="));
  Serial.print(sirenOn ? F("ON") : F("OFF"));
  Serial.print(F("|igniter_led="));
  Serial.print(igniterOn ? F("ON") : F("OFF"));
  Serial.print(F("|servo_deg="));
  Serial.print(lastServoDeg);
  Serial.print(F("|target_deg="));
  Serial.print(targetDeg, 1);
  Serial.print(F("|at_target="));
  Serial.print(fabs(targetDeg - currentDeg) > TARGET_TOL_DEG ? F("0") : F("1"));
  Serial.print(F("|speed_dps="));
  Serial.print(SERVO_SPEED_DPS, 1);
  Serial.print(F("|prelog_ready="));
  Serial.print(state == ST_PRE_LOG && stateElapsedMs >= PRE_LOG_MS ? F("1") : F("0"));
  Serial.print(F("|reset_required="));
  Serial.print(state == ST_END || state == ST_ABORT ? F("1") : F("0"));
  Serial.println();
}

void printBanner() {
  Serial.println();
  Serial.println(F("============================================================"));
  Serial.println(F("Arduino Mega TMS sequence + GCS MANUAL output controller"));
  Serial.println(F("SIREN=D22, IGNITER=D23, SERVO=D9, ESTOP=D2"));
  Serial.println(F("Servo angles: CLOSED=60, LOW=80, FULL=120 deg"));
  Serial.print(F("Servo max speed: "));
  Serial.print(SERVO_SPEED_DPS, 1);
  Serial.println(F(" deg/s"));
  Serial.print(F("Manual servo allowed range: "));
  Serial.print(SERVO_MIN_DEG);
  Serial.print(F(" .. "));
  Serial.print(SERVO_MAX_DEG);
  Serial.println(F(" deg; OUT-OF-RANGE = REJECT, NO CLAMP"));
  Serial.println(F("Baud: 115200"));
  Serial.println();
  Serial.println(F("Sequence commands:"));
  Serial.println(F("  1 = start sequence / PRE_LOG"));
  Serial.println(F("  0 = ABORT"));
  Serial.println(F("  g = GO after PRE_LOG minimum 5s"));
  Serial.println(F("  r = reset after ABORT or END"));
  Serial.println(F("  s = status"));
  Serial.println();
  Serial.println(F("Manual GCS commands:"));
  Serial.println(F("  !SIREN=1 / !SIREN=0"));
  Serial.println(F("  !IGNITER=1 / !IGNITER=0"));
  Serial.println(F("  !SERVO=57..123"));
  Serial.println(F("  ABORT: IGNITER ON blocked; SERVO locked CLOSED=60"));
  Serial.println();
  Serial.println(F("Sequence:"));
  Serial.println(F("  IDLE -> PRE_LOG 5s + GCS GO"));
  Serial.println(F("  SIREN 15s -> WAIT 12s -> IGNITION 3s"));
  Serial.println(F("  LOW_FLOW -> hold 7s after 80deg arrival"));
  Serial.println(F("  FLOW_INC -> FULL_FLOW -> hold 15s after 120deg arrival"));
  Serial.println(F("  CLOSED 60deg -> POST_LOG 10s -> END"));
  Serial.println(F("  END / ABORT require RESET for a new automatic sequence"));
  Serial.println();
  Serial.println(F("ESTOP: LOW=normal, HIGH=abort"));
  Serial.println(F("============================================================"));
}

// 빈 줄, EVENT, 강제 STATUS 순서로 출력한다.
void reportEvent(const __FlashStringHelper* message) {
  Serial.println();
  printEvent(message);
  printStatus(true);
}

// ---------------------------------------------------------------------------
// 상태 전환
// ---------------------------------------------------------------------------
void enterState(int nextState) {
  state = nextState;
  stateEnteredMs = millis();
  atTarget = false;
  arrivedMs = 0;
  goAccepted = false;

  switch (state) {
    case ST_SIREN:
      setTargetClosed();
      setIgniter(false);
      setSiren(true);
      break;
    case ST_WAIT:
      setTargetClosed();
      setSiren(false);
      setIgniter(false);
      break;
    case ST_IGNITION:
      setTargetClosed();
      setSiren(false);
      setIgniter(true);
      break;
    case ST_LOW_FLOW:
      outputsOff();
      targetDeg = SERVO_LOW_DEG;
      break;
    case ST_FLOW_INC:
    case ST_FULL_FLOW:
      outputsOff();
      targetDeg = SERVO_FULL_DEG;
      break;
    default:
      outputsOff();
      setTargetClosed();
      break;
  }
}

void abortNow() {
  state = ST_ABORT;
  stateEnteredMs = millis();
  atTarget = false;
  arrivedMs = 0;
  goAccepted = false;
  safeState();
  reportEvent(F("ABORT - indicators OFF, servo commanded CLOSED immediately"));
}

void advanceTo(int nextState, const __FlashStringHelper* message) {
  enterState(nextState);
  reportEvent(message);
}

void runState(unsigned long nowMs) {
  const unsigned long elapsedMs = nowMs - stateEnteredMs;

  switch (state) {
    case ST_PRE_LOG:
      if (elapsedMs >= PRE_LOG_MS && goAccepted) {
        advanceTo(ST_SIREN, F("STEP - PRE_LOG -> SIREN"));
      }
      break;

    case ST_SIREN:
      if (elapsedMs >= SIREN_MS) {
        advanceTo(ST_WAIT, F("STEP - SIREN -> WAIT"));
      }
      break;

    case ST_WAIT:
      if (elapsedMs >= WAIT_MS) {
        advanceTo(ST_IGNITION, F("STEP - WAIT -> IGNITION"));
      }
      break;

    case ST_IGNITION:
      if (elapsedMs >= IGNITION_MS) {
        advanceTo(ST_LOW_FLOW, F("STEP - IGNITION -> LOW_FLOW"));
      }
      break;

    // 원본과 다른 부분: 원본은 trackTarget() 전에 측정한 nowMs로 유지 시간을 계산한다.
    // 두 시각 사이에 millis()가 넘어가면 뺄셈이 언더플로되어 유지 시간이 건너뛰어진다.
    // 여기서는 trackTarget() 뒤에 millis()를 다시 읽는다.
    case ST_LOW_FLOW:
      trackTarget(SERVO_LOW_DEG);
      if (atTarget && millis() - arrivedMs >= LOW_HOLD_MS) {
        advanceTo(ST_FLOW_INC, F("STEP - LOW_FLOW -> FLOW_INC"));
      }
      break;

    case ST_FLOW_INC:
      if (fabs(SERVO_FULL_DEG - currentDeg) <= TARGET_TOL_DEG) {
        advanceTo(ST_FULL_FLOW, F("STEP - FLOW_INC -> FULL_FLOW"));
      }
      break;

    case ST_FULL_FLOW:
      trackTarget(SERVO_FULL_DEG);
      if (atTarget && millis() - arrivedMs >= FULL_HOLD_MS) {
        advanceTo(ST_CLOSED, F("STEP - FULL_FLOW -> CLOSED"));
      }
      break;

    case ST_CLOSED:
      if (fabs(SERVO_CLOSED_DEG - currentDeg) <= TARGET_TOL_DEG) {
        advanceTo(ST_POST_LOG, F("STEP - CLOSED reached -> POST_LOG"));
      }
      break;

    case ST_POST_LOG:
      if (elapsedMs >= POST_LOG_MS) {
        advanceTo(ST_END, F("END - POST_LOG complete; RESET required"));
      }
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// 시리얼 입력
// ---------------------------------------------------------------------------
bool parseOnOff(const char* text, bool* value) {
  if (strcmp(text, "1") == 0 || strcmp(text, "ON") == 0) {
    *value = true;
    return true;
  }
  if (strcmp(text, "0") == 0 || strcmp(text, "OFF") == 0) {
    *value = false;
    return true;
  }
  return false;
}

void runManualSiren(const char* text) {
  bool on = false;
  if (!parseOnOff(text, &on)) {
    printError(F("SIREN_PARSE"), F("USE_0_OR_1"));
    return;
  }
  setSiren(on);
  printManualEvent("SIREN", on ? "ON" : "OFF");
  printStatus(true);
}

void runManualIgniter(const char* text) {
  bool on = false;
  if (!parseOnOff(text, &on)) {
    printError(F("IGNITER_PARSE"), F("USE_0_OR_1"));
    return;
  }
  if (on && state == ST_ABORT) {
    printError(F("IGNITER_BLOCKED"), F("ABORT_LATCHED"));
    printStatus(true);
    return;
  }
  if (on && estopActive()) {
    printError(F("IGNITER_BLOCKED"), F("ESTOP_HIGH"));
    printStatus(true);
    return;
  }
  setIgniter(on);
  printManualEvent("IGNITER", on ? "ON" : "OFF");
  printStatus(true);
}

void runManualServo(const char* text) {
  char* end = NULL;
  const float value = strtod(text, &end);

  if (end == text || end == NULL || *end != '\0' || isnan(value) || isinf(value)) {
    printError(F("SERVO_PARSE"), F("NUMERIC_ANGLE_REQUIRED"));
    return;
  }
  if (state == ST_ABORT) {
    printError(F("SERVO_BLOCKED"), F("ABORT_LATCHED_CLOSED_LOCK"));
    forceClosedNow();
    printStatus(true);
    return;
  }
  if (estopActive()) {
    printError(F("SERVO_BLOCKED"), F("ESTOP_HIGH"));
    forceClosedNow();
    printStatus(true);
    return;
  }
  if (value < (float)SERVO_MIN_DEG || value > (float)SERVO_MAX_DEG) {
    Serial.print(F("ERROR|SERVO_LIMIT|requested="));
    Serial.print(value, 2);
    Serial.print(F("|allowed="));
    Serial.print(SERVO_MIN_DEG);
    Serial.print(F(".."));
    Serial.print(SERVO_MAX_DEG);
    Serial.println();
    printStatus(true);
    return;
  }

  targetDeg = value;
  Serial.print(F("EVENT|MANUAL_SERVO="));
  Serial.print(value, 2);
  Serial.println();
  printStatus(true);
}

void runManualCommand() {
  if (state >= ST_PRE_LOG && state <= ST_POST_LOG) {
    Serial.print(F("ERROR|MANUAL_BLOCKED|AUTO_SEQUENCE_RUNNING|state="));
    Serial.println(STATE_NAMES[state]);
    printStatus(true);
    return;
  }
  if (manualCmd[0] == '\0') {
    printError(F("MANUAL_PARSE"), F("EMPTY_COMMAND"));
    return;
  }

  if (strncmp(manualCmd, "SIREN=", 6) == 0) {
    runManualSiren(manualCmd + 6);
  } else if (strncmp(manualCmd, "IGNITER=", 8) == 0) {
    runManualIgniter(manualCmd + 8);
  } else if (strncmp(manualCmd, "SERVO=", 6) == 0) {
    runManualServo(manualCmd + 6);
  } else {
    printError(F("MANUAL_PARSE"), F("UNKNOWN_COMMAND"));
  }
}

void commandStart() {
  if (state == ST_END || state == ST_ABORT) {
    reportEvent(F("BLOCKED - RESET required before next START"));
  } else if (state != ST_IDLE) {
    reportEvent(F("BLOCKED - sequence is not IDLE"));
  } else if (estopActive()) {
    Serial.println();
    printEvent(F("BLOCKED - ESTOP HIGH"));
    abortNow();
  } else {
    enterState(ST_PRE_LOG);
    reportEvent(F("START - PRE_LOG entered; minimum 5 s + explicit GO required"));
  }
}

void commandGo() {
  if (state != ST_PRE_LOG) {
    reportEvent(F("BLOCKED - GO accepted only in PRE_LOG"));
  } else if (millis() - stateEnteredMs < PRE_LOG_MS) {
    reportEvent(F("BLOCKED - PRE_LOG minimum 5 s not complete"));
  } else if (estopActive()) {
    printEvent(F("BLOCKED - ESTOP HIGH during PRE_LOG"));
    abortNow();
  } else {
    goAccepted = true;
    reportEvent(F("GO - PRE_LOG complete; SIREN authorized"));
  }
}

void commandReset() {
  if (estopActive()) {
    safeState();
    reportEvent(F("BLOCKED - ESTOP still HIGH"));
  } else if (state == ST_IDLE) {
    reportEvent(F("RESET - already IDLE"));
  } else if (state != ST_END && state != ST_ABORT) {
    reportEvent(F("BLOCKED - RESET allowed only after ABORT or END"));
  } else {
    enterState(ST_IDLE);
    reportEvent(F("RESET - system IDLE and ready"));
  }
}

// 한 글자 명령을 처리했으면 true를 돌려준다. true면 이번 루프의 시리얼 읽기를 끝낸다.
bool handleSequenceCommand(char c) {
  if (c == '0') {
    abortNow();
    return true;
  }
  if (c == '1') {
    commandStart();
    return true;
  }

  switch (c & 0xDF) {
    case 'G':
      commandGo();
      return true;
    case 'R':
      commandReset();
      return true;
    case 'S':
      printConfig();
      printStatus(true);
      return true;
    default:
      Serial.println(F("INFO|commands: 1=start PRE_LOG, g=GO, 0=ABORT, r=reset, s=status, !SIREN=0/1, !IGNITER=0/1, !SERVO=57..123"));
      return false;
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    const char c = Serial.read();

    if (discardLine) {
      if (c == '\n' || c == '\r') {
        discardLine = false;
      }
      continue;
    }

    if (inManualCmd) {
      if (c == '\n' || c == '\r') {
        manualCmd[manualLen] = '\0';
        inManualCmd = false;
        runManualCommand();
        manualLen = 0;
      } else if (manualLen >= MANUAL_CMD_MAX) {
        inManualCmd = false;
        manualLen = 0;
        discardLine = true;
        printError(F("MANUAL_PARSE"), F("COMMAND_TOO_LONG"));
      } else {
        manualCmd[manualLen++] = c;
      }
      continue;
    }

    if (c == '!') {
      inManualCmd = true;
      manualLen = 0;
      continue;
    }
    if (c == '\r' || c == ' ' || c == '\t' || c == '\n') {
      continue;
    }
    if (handleSequenceCommand(c)) {
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  digitalWrite(SIREN_PIN, LOW);
  digitalWrite(IGN_PIN, LOW);
  pinMode(SIREN_PIN, OUTPUT);
  pinMode(IGN_PIN, OUTPUT);

  valveServo.attach(SERVO_PIN);
  forceClosedNow();
  enterState(ST_IDLE);
  lastStatusMs = 0;

  printBanner();
  printConfig();
  printStatus(true);
}

void loop() {
  if (estopActive()) {
    if (state == ST_ABORT) {
      outputsOff();
      forceClosedNow();
      printStatus(false);
    } else {
      abortNow();
    }
    return;
  }

  readSerial();

  if (state == ST_ABORT) {
    targetDeg = SERVO_CLOSED_DEG;
    currentDeg = SERVO_CLOSED_DEG;
    lastServoDeg = SERVO_CLOSED_DEG;
    printStatus(false);
    return;
  }

  updateServo();
  runState(millis());
  updateServo();
  printStatus(false);
}
