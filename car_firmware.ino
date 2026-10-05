#include <Servo.h>
#include <WiFiS3.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <ctype.h>

// ===== Motor Pins (L298N) - Arduino R4 GPIO =====
const int ENA = 5, IN1 = 6, IN2 = 7;
const int ENB = 8, IN3 = 9, IN4 = 10;
const uint8_t MOTOR_A_FORWARD_IN1 = LOW;
const uint8_t MOTOR_A_FORWARD_IN2 = HIGH;
const uint8_t MOTOR_B_FORWARD_IN3 = LOW;
const uint8_t MOTOR_B_FORWARD_IN4 = HIGH;

// ===== Drive Speeds =====
const int SEARCH_SPEED_A = 75;    // Extra-slow crawl (steady)
const int SEARCH_SPEED_B = 75;
const int SEARCH_BOOST_SPEED_A = 100;  // Gentle startup boost to overcome stall friction
const int SEARCH_BOOST_SPEED_B = 100;
const unsigned long SEARCH_BOOST_MS = 220;

// ===== MH Line Sensor Pins (reserved for future line-following) =====
const int SENSOR_L = 2;
const int SENSOR_R = A5;

// ===== Servo Pins & Objects =====
const int PIN_BASE = 3, PIN_FORW = 11, PIN_VERT = 12, PIN_CLAW = 13;
Servo baseServo, forwardServo, verticalServo, clawServo;

// ===== Arm Targets =====
int basePos = 90;
int forPos = 90;
int vertPos = 90;
int clawPos = 180;

int baseNow = 90;  // Smoothed base-servo angle
const int CLAW_OPEN = 180;
const int CLAW_CLOSED = 0;

const int BASE_DEFAULT_ANGLE = 90;
const int BASE_SCAN_ANGLE = 145;  // Right scan angle (camera mounted on arm)
const int ARM_DEFAULT_FORWARD = 90;
const int ARM_DEFAULT_VERTICAL = 90;
const int ARM_SEARCH_FORWARD = 45;    // Full backward while searching
const int ARM_SEARCH_VERTICAL = 65;   // Raised position while searching

const int BASE_SERVO_STEP_DEG = 1;
const unsigned long BASE_SERVO_UPDATE_MS = 30;
unsigned long lastBaseServoUpdate = 0;

// ===== Pick Sequence Tuning =====
const unsigned long PICK_STOP_SETTLE_MS = 220;
const unsigned long PICK_GRAB_CLOSE_HOLD_MS = 500;
const unsigned long PICK_BACKOFF_SETTLE_MS = 900;
const unsigned long PICK_DROP_HOLD_MS = 700;
const unsigned long PICK_RETURN_AFTER_DROP_MS = 900;
const unsigned long PICK_CLOSE_FINISH_MS = 350;
const int ARM_BACKOFF_FORWARD = 45;    // Full backward position before drop (reversed servo direction)
const int SERVO_ANGLE_TOLERANCE = 1;

// ===== WiFi & Command Socket =====
const char* ssid = "swifi";
const char* password = "12345678";
const uint16_t COMMAND_PORT = 80;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 30000;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000;

WiFiServer commandServer(COMMAND_PORT);
WiFiClient commandClient;
bool commandServerStarted = false;
unsigned long lastWiFiReconnectAttempt = 0;

// ===== State Machine =====
enum RobotState {
  STATE_IDLE,
  STATE_SEARCHING,
  STATE_PICKING
};

enum PickPhase {
  PICK_PHASE_SETTLE,
  PICK_PHASE_GRAB,
  PICK_PHASE_RETURN_DEFAULT,
  PICK_PHASE_BACKOFF,
  PICK_PHASE_DROP,
  PICK_PHASE_RETURN_AFTER_DROP,
  PICK_PHASE_CLOSE_AT_DEFAULT
};

RobotState currentState = STATE_IDLE;
PickPhase pickPhase = PICK_PHASE_SETTLE;
unsigned long stateStartTime = 0;
unsigned long pickPhaseStartedAt = 0;
String targetItem = "";

// ===== Function Declarations =====
void handleIncomingCommands();
bool processCommand(const String& command, String& response);
String normalizeLabel(const String& value);
bool itemNamesMatch(const String& a, const String& b);

bool hasValidWifiIp();
bool connectWiFiWithDhcp(unsigned long timeoutMs);
void startCommandServerIfNeeded();
void maintainWiFiConnection();

void startSearchMission(const String& itemName);
void startPickSequence();
void enterIdleState(const char* reason);
void setIdleArmPose();
void runSearchBehavior();
void executePickingSequence(unsigned long elapsed);
const char* stateName(RobotState state);
void setPickPhase(PickPhase phase);
bool baseAtTargetAngle(int targetAngle);

void setupMotors();
void setupServos();
void writeAllServos();
int stepToward(int current, int target, int step);
void setMotor(bool a1, bool a2, bool b1, bool b2, int speedA, int speedB);
void driveForwardSlow();
void driveForwardAtSpeed(int speedA, int speedB);
void stopMotors();

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!connectWiFiWithDhcp(WIFI_CONNECT_TIMEOUT_MS)) {
    Serial.println("Continuing in offline mode until WiFi gets a valid IP.");
  }
  startCommandServerIfNeeded();

  pinMode(SENSOR_L, INPUT);
  pinMode(SENSOR_R, INPUT);

  setupMotors();
  setupServos();
  setIdleArmPose();

  Serial.println("=== Robot Picker Ready ===");
  Serial.println("Commands:");
  Serial.println("  mission_start|<item>");
  Serial.println("  pick_now|<item>");
  Serial.println("  mission_cancel");
  Serial.println("  <item> (legacy one-shot pick)");
}

// ===== Main Loop =====
void loop() {
  maintainWiFiConnection();
  handleIncomingCommands();

  unsigned long elapsed = millis() - stateStartTime;

  switch (currentState) {
    case STATE_IDLE:
      stopMotors();
      setIdleArmPose();
      break;

    case STATE_SEARCHING:
      runSearchBehavior();
      break;

    case STATE_PICKING:
      executePickingSequence(elapsed);
      break;
  }

  writeAllServos();
  delay(20);  // ~50 Hz update
}

// ===== Command Handling =====
void handleIncomingCommands() {
  if (!hasValidWifiIp()) {
    return;
  }

  startCommandServerIfNeeded();

  if (!commandClient || !commandClient.connected()) {
    WiFiClient incoming = commandServer.available();
    if (incoming) {
      commandClient = incoming;
      Serial.println("Command client connected");
    }
  }

  if (commandClient && commandClient.connected() && commandClient.available()) {
    String cmd = commandClient.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) {
      return;
    }

    String reply;
    bool ok = processCommand(cmd, reply);
    if (ok) {
      commandClient.println("OK:" + reply);
    } else {
      commandClient.println("ERR:" + reply);
    }
  }
}

bool processCommand(const String& commandIn, String& response) {
  String command = commandIn;
  command.trim();

  Serial.println("Command received: " + command);

  if (command.length() == 0) {
    response = "empty_command";
    return false;
  }

  if (command.equalsIgnoreCase("status")) {
    response = String("STATE:") + stateName(currentState);
    if (targetItem.length() > 0) {
      response += ":item=" + targetItem;
    }
    return true;
  }

  if (command.equalsIgnoreCase("mission_cancel") || command.equalsIgnoreCase("stop")) {
    enterIdleState("Mission cancelled by remote command.");
    response = "MISSION_CANCELLED";
    return true;
  }

  if (command.startsWith("mission_start|")) {
    int sep = command.indexOf('|');
    String item = command.substring(sep + 1);
    item.trim();

    if (item.length() == 0) {
      response = "missing_item";
      return false;
    }

    if (currentState != STATE_IDLE) {
      response = "busy_state_" + String(stateName(currentState));
      return false;
    }

    startSearchMission(item);
    response = "MISSION_STARTED:" + item;
    return true;
  }

  if (command.startsWith("pick_now|")) {
    int sep = command.indexOf('|');
    String item = command.substring(sep + 1);
    item.trim();

    if (item.length() == 0) {
      response = "missing_item";
      return false;
    }

    if (currentState != STATE_SEARCHING) {
      response = "not_searching_state_" + String(stateName(currentState));
      return false;
    }

    if (targetItem.length() > 0 && !itemNamesMatch(targetItem, item)) {
      response = "item_mismatch_expected_" + targetItem;
      return false;
    }

    startPickSequence();
    response = "PICKING:" + item;
    return true;
  }

  // Backward compatibility: plain item name triggers immediate one-shot pick.
  if (currentState != STATE_IDLE) {
    response = "busy_state_" + String(stateName(currentState));
    return false;
  }

  targetItem = command;
  startPickSequence();
  response = "PICKING:" + targetItem;
  return true;
}

String normalizeLabel(const String& value) {
  String out;
  out.reserve(value.length());

  for (int i = 0; i < value.length(); i++) {
    unsigned char c = (unsigned char)value.charAt(i);
    if (isalnum(c)) {
      out += (char)tolower(c);
    }
  }

  return out;
}

bool itemNamesMatch(const String& a, const String& b) {
  String na = normalizeLabel(a);
  String nb = normalizeLabel(b);
  if (na.length() == 0 || nb.length() == 0) {
    return false;
  }
  return na == nb;
}

const char* stateName(RobotState state) {
  switch (state) {
    case STATE_IDLE: return "IDLE";
    case STATE_SEARCHING: return "SEARCHING";
    case STATE_PICKING: return "PICKING";
    default: return "UNKNOWN";
  }
}

void setPickPhase(PickPhase phase) {
  pickPhase = phase;
  pickPhaseStartedAt = millis();
}

bool baseAtTargetAngle(int targetAngle) {
  return abs(baseNow - targetAngle) <= SERVO_ANGLE_TOLERANCE;
}

// ===== Mission State Helpers =====
void startSearchMission(const String& itemName) {
  targetItem = itemName;
  currentState = STATE_SEARCHING;
  stateStartTime = millis();

  // Make side-looking direction visible immediately at mission start.
  basePos = BASE_SCAN_ANGLE;
  forPos = ARM_SEARCH_FORWARD;
  vertPos = ARM_SEARCH_VERTICAL;
  baseNow = BASE_SCAN_ANGLE;
  baseServo.write(baseNow);
  clawPos = CLAW_OPEN;

  Serial.println("Mission started for item: " + targetItem);
  Serial.println("Robot moving backward slowly and scanning right side...");
}

void startPickSequence() {
  currentState = STATE_PICKING;
  stateStartTime = millis();
  setPickPhase(PICK_PHASE_SETTLE);
  stopMotors();

  Serial.println("Target locked. Stopping and starting pick sequence.");
}

void setIdleArmPose() {
  basePos = BASE_DEFAULT_ANGLE;
  forPos = ARM_DEFAULT_FORWARD;
  vertPos = ARM_DEFAULT_VERTICAL;
  clawPos = CLAW_CLOSED;
}

void enterIdleState(const char* reason) {
  currentState = STATE_IDLE;
  pickPhase = PICK_PHASE_SETTLE;
  stateStartTime = millis();
  targetItem = "";
  stopMotors();
  setIdleArmPose();

  if (reason != nullptr && reason[0] != '\0') {
    Serial.println(reason);
  }
}

void runSearchBehavior() {
  // Search mode: slow wheel motion with arm/camera side-facing and raised.
  basePos = BASE_SCAN_ANGLE;
  forPos = ARM_SEARCH_FORWARD;
  vertPos = ARM_SEARCH_VERTICAL;
  clawPos = CLAW_OPEN;

  unsigned long searchElapsed = millis() - stateStartTime;
  if (searchElapsed < SEARCH_BOOST_MS) {
    driveForwardAtSpeed(SEARCH_BOOST_SPEED_A, SEARCH_BOOST_SPEED_B);
  } else {
    driveForwardSlow();
  }
}

void executePickingSequence(unsigned long elapsed) {
  (void)elapsed;
  stopMotors();

  switch (pickPhase) {
    case PICK_PHASE_SETTLE:
      // Brief settle window after movement stop.
      basePos = BASE_SCAN_ANGLE;
      forPos = ARM_DEFAULT_FORWARD;
      vertPos = ARM_DEFAULT_VERTICAL;
      clawPos = CLAW_OPEN;
      if (millis() - pickPhaseStartedAt >= PICK_STOP_SETTLE_MS) {
        setPickPhase(PICK_PHASE_GRAB);
      }
      return;

    case PICK_PHASE_GRAB:
      // Close claw and hold item.
      clawPos = CLAW_CLOSED;
      if (millis() - pickPhaseStartedAt >= PICK_GRAB_CLOSE_HOLD_MS) {
        setPickPhase(PICK_PHASE_RETURN_DEFAULT);
      }
      return;

    case PICK_PHASE_RETURN_DEFAULT:
      // Keep claw closed until arm reaches default pose.
      basePos = BASE_DEFAULT_ANGLE;
      forPos = ARM_DEFAULT_FORWARD;
      vertPos = ARM_DEFAULT_VERTICAL;
      clawPos = CLAW_CLOSED;
      if (baseAtTargetAngle(BASE_DEFAULT_ANGLE)) {
        setPickPhase(PICK_PHASE_BACKOFF);
      }
      return;

    case PICK_PHASE_BACKOFF:
      // After reaching default, move arm fully backward while still holding item.
      basePos = BASE_DEFAULT_ANGLE;
      forPos = ARM_BACKOFF_FORWARD;
      vertPos = ARM_DEFAULT_VERTICAL;
      clawPos = CLAW_CLOSED;
      if (millis() - pickPhaseStartedAt >= PICK_BACKOFF_SETTLE_MS) {
        setPickPhase(PICK_PHASE_DROP);
      }
      return;

    case PICK_PHASE_DROP:
      // Drop item from full-backward position.
      basePos = BASE_DEFAULT_ANGLE;
      forPos = ARM_BACKOFF_FORWARD;
      vertPos = ARM_DEFAULT_VERTICAL;
      clawPos = CLAW_OPEN;
      if (millis() - pickPhaseStartedAt >= PICK_DROP_HOLD_MS) {
        setPickPhase(PICK_PHASE_RETURN_AFTER_DROP);
      }
      return;

    case PICK_PHASE_RETURN_AFTER_DROP:
      // Return to default position after drop while keeping claw open.
      basePos = BASE_DEFAULT_ANGLE;
      forPos = ARM_DEFAULT_FORWARD;
      vertPos = ARM_DEFAULT_VERTICAL;
      clawPos = CLAW_OPEN;
      if (millis() - pickPhaseStartedAt >= PICK_RETURN_AFTER_DROP_MS) {
        setPickPhase(PICK_PHASE_CLOSE_AT_DEFAULT);
      }
      return;

    case PICK_PHASE_CLOSE_AT_DEFAULT:
      // Final state requested: at default position with claw closed.
      basePos = BASE_DEFAULT_ANGLE;
      forPos = ARM_DEFAULT_FORWARD;
      vertPos = ARM_DEFAULT_VERTICAL;
      clawPos = CLAW_CLOSED;
      if (millis() - pickPhaseStartedAt >= PICK_CLOSE_FINISH_MS) {
        Serial.println("Pick cycle completed. Ready for next mission.");
        enterIdleState("");
      }
      return;
  }
}

// ===== WiFi Helpers =====
bool hasValidWifiIp() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  IPAddress ip = WiFi.localIP();
  return !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
}

bool connectWiFiWithDhcp(unsigned long timeoutMs) {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.disconnect();
  delay(150);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (hasValidWifiIp()) {
      Serial.println("\nWiFi connected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi link is up, but DHCP still returned 0.0.0.0");
  } else {
    Serial.println("Failed to connect WiFi");
  }
  return false;
}

void startCommandServerIfNeeded() {
  if (!hasValidWifiIp() || commandServerStarted) {
    return;
  }

  commandServer.begin();
  commandServerStarted = true;

  Serial.print("Command server listening on port ");
  Serial.print(COMMAND_PORT);
  Serial.println(" (plain TCP)");
}

void maintainWiFiConnection() {
  if (hasValidWifiIp()) {
    startCommandServerIfNeeded();
    return;
  }

  if (commandServerStarted) {
    commandServerStarted = false;
    if (commandClient) {
      commandClient.stop();
    }
  }

  unsigned long now = millis();
  if (now - lastWiFiReconnectAttempt < WIFI_RECONNECT_INTERVAL_MS) {
    return;
  }
  lastWiFiReconnectAttempt = now;

  Serial.println("WiFi not ready, retrying...");
  connectWiFiWithDhcp(12000);
  startCommandServerIfNeeded();
}

// ===== Motor Helpers =====
void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
}

void setMotor(bool a1, bool a2, bool b1, bool b2, int speedA, int speedB) {
  analogWrite(ENA, constrain(speedA, 0, 255));
  analogWrite(ENB, constrain(speedB, 0, 255));
  digitalWrite(IN1, a1);
  digitalWrite(IN2, a2);
  digitalWrite(IN3, b1);
  digitalWrite(IN4, b2);
}

void driveForwardSlow() {
  driveForwardAtSpeed(SEARCH_SPEED_A, SEARCH_SPEED_B);
}

void driveForwardAtSpeed(int speedA, int speedB) {
  setMotor(MOTOR_A_FORWARD_IN1, MOTOR_A_FORWARD_IN2, MOTOR_B_FORWARD_IN3, MOTOR_B_FORWARD_IN4, speedA, speedB);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ===== Servo Helpers =====
void setupServos() {
  baseServo.attach(PIN_BASE);
  forwardServo.attach(PIN_FORW);
  verticalServo.attach(PIN_VERT);
  clawServo.attach(PIN_CLAW);
  baseNow = basePos;
  writeAllServos();
}

int stepToward(int current, int target, int step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void writeAllServos() {
  unsigned long now = millis();
  if (now - lastBaseServoUpdate >= BASE_SERVO_UPDATE_MS) {
    lastBaseServoUpdate = now;
    baseNow = stepToward(baseNow, basePos, BASE_SERVO_STEP_DEG);
  }

  baseServo.write(baseNow);
  forwardServo.write(forPos);
  verticalServo.write(vertPos);
  clawServo.write(clawPos);
}
