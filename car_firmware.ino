#include <Wire.h>
#include <DHTesp.h>
#include <WiFiS3.h>
#include <WiFiSSLClient.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <math.h>
#include <string.h>   // strcmp() used by motor telemetry

// ===== Wi-Fi & Firebase Credentials =====
#define WIFI_SSID     "swifi"
#define WIFI_PASSWORD "123456784"
#define FIREBASE_HOST "iotcaverobot-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_API_KEY "sNrMi7df7BewoNMtBQPOlIhMxTwNHhlmktyvISo5"

// ===== Pin Definitions =====
#define ENA 11
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7
#define ENB 3
#define DHTPIN 2
#define LEFT_ENCODER_PIN 8
#define RIGHT_ENCODER_PIN 9
#define SERVO_PIN 10
#define TRIG_PIN 12
#define ECHO_PIN A0

// ===== Motor Speed =====
#define SLOW_SPEED 80
#define TURN_SPEED 100
#define BACKUP_SPEED 70        // === NEW: slower reverse ===

// ===== Robot Physical Parameters =====
#define WHEEL_RADIUS 0.0300
#define WHEEL_BASE 0.113
#define ENCODER_SLOTS 12
#define COUNTS_PER_REV (ENCODER_SLOTS * 2)

// ===== Ultrasonic / Servo Configuration =====
#define SERVO_CENTER 90
#define SERVO_MIN 20
#define SERVO_MAX 160
#define OBSTACLE_THRESHOLD 0.35
#define DANGER_THRESHOLD 0.20
#define MAX_RANGE 3.0
#define SCAN_INTERVAL 100
#define FULL_SCAN_INTERVAL 5000

// ===== FAST SCAN SETTINGS =====
#define SERVO_SETTLE_MS 150
#define SONAR_TIMEOUT_US 12000

// ===== Obstacle Detection Filtering =====
#define SONAR_SAMPLES 3
#define SONAR_SAMPLE_DELAY 20

// ===== CAVE MAPPING PARAMETERS (RAM-OPTIMIZED) =====
#define MAP_GRID_SIZE 0.15
#define WALL_CONFIDENCE 3
#define CAVE_FEATURES_MAX 20
#define FULL_360_SCAN_INTERVAL 10000
#define MAX_OBSTACLE_POINTS 100
#define GRID_MAX_X 32
#define GRID_MAX_Y 32

// ===== MPU =====
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75

// ===== VISION FUSION =====
#define VISION_PORT            4212
#define VISION_MAGIC           0xC8
#define VISION_TIMEOUT_MS      500
#define VISION_STOP_CONFIRM    3
#define VISION_GO_CONFIRM      5
#define VISION_STUCK_TIMEOUT   3000

// ===== VISION PACKET COMMAND CODES (byte 1) =====
#define VISION_CMD_HAZARD_STOP 0   // AI sees an obstacle -> temporary brake
#define VISION_CMD_HAZARD_GO   1   // corridor clear      -> resume
#define VISION_CMD_OP_STOP     2   // STOP button -> latch motors OFF
#define VISION_CMD_OP_START    3   // START button -> latch motors ON

// ===== IMPROVED BACKUP / RECOVERY =====
#define BACKUP_DISTANCE_M      0.06     // target backup distance (6 cm)
#define BACKUP_MAX_MS          250      // hard time cap
#define MAX_CONSEC_BACKUPS     2        // after this many in a row, spin instead
#define SPIN_180_MS            1200     // duration for ~180 deg in-place turn
#define FORWARD_STABLE_MS      2000     // reset backup counter after this much forward

// ===== Cave Features =====
enum CaveFeatureType {
  FEATURE_WALL,
  FEATURE_JUNCTION,
  FEATURE_DEAD_END,
  FEATURE_CHAMBER,
  FEATURE_NARROW_PASS
};

struct CaveFeature {
  float x, y;
  CaveFeatureType type;
  float heading;
  float width;
  float ceiling_height;
  unsigned long timestamp;
  int confidence;
};

struct GridCell {
  uint8_t wall_count;
  uint8_t free_count;
};

// ===== EKF =====
class PositionEKF {
private:
  float x, y, theta;
  float P[3][3];
  float Q[3][3];
  float R_imu;
public:
  PositionEKF() {
    x = 0; y = 0; theta = 0;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        P[i][j] = (i==j) ? 1.0 : 0.0;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        Q[i][j] = 0;
    Q[0][0] = 0.01; Q[1][1] = 0.01; Q[2][2] = 0.001;
    R_imu = 0.01;
  }
  void predict(float ld, float rd) {
    float dDist = (ld + rd) / 2.0;
    float dTheta = (rd - ld) / WHEEL_BASE;
    x += dDist * cos(theta + dTheta/2);
    y += dDist * sin(theta + dTheta/2);
    theta += dTheta;
    while (theta > PI) theta -= 2*PI;
    while (theta < -PI) theta += 2*PI;
    P[0][0] += Q[0][0]; P[1][1] += Q[1][1]; P[2][2] += Q[2][2];
  }
  void updateIMU(float imuTheta) {
    float S_theta = P[2][2] + R_imu;
    float K_theta = P[2][2] / S_theta;
    float y_theta = imuTheta - theta;
    theta += K_theta * y_theta;
    P[2][2] = (1 - K_theta) * P[2][2];
    while (theta > PI) theta -= 2*PI;
    while (theta < -PI) theta += 2*PI;
  }
  float getX() { return x; }
  float getY() { return y; }
  float getTheta() { return theta; }
  float getPosUncertainty() { return sqrt(P[0][0] + P[1][1]); }
  void reset() {
    x = 0; y = 0; theta = 0;
    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
        P[i][j] = (i==j) ? 1.0 : 0.0;
  }
};

class SimpleKalman {
public:
  float Q_angle = 0.001f;
  float Q_bias = 0.003f;
  float R_measure = 0.03f;
  float angle = 0.0f;
  float bias = 0.0f;
  float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  void setAngle(float a) { angle = a; }
  float getAngle(float newAngle, float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;
    float P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp; P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp; P[1][1] -= K[1] * P01_temp;
    return angle;
  }
};

// ===== Objects =====
DHTesp dht;
WiFiSSLClient client;
Servo scanServo;
SimpleKalman kalmanX, kalmanY;
PositionEKF positionEKF;

// ===== Encoder =====
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
unsigned long lastLeftCount = 0, lastRightCount = 0, lastEncoderTime = 0;

// ===== Scanning =====
float currentSonarDistance = 0;
int currentServoAngle = SERVO_CENTER;
bool obstacleDetected = false;
bool fullScanInProgress = false;
unsigned long lastFullScanTime = 0;
int scanStepIndex = 0;
unsigned long lastScanStepTime = 0;
const int scanAngles[] = {20, 40, 60, 80, 90, 100, 120, 140, 160};
const int SCAN_ANGLE_COUNT = 9;

// ===== Servo watchdog =====
unsigned long lastServoMoveTime = 0;
int lastServoCommandedAngle = SERVO_CENTER;

// ===== Cave map =====
struct ObstaclePoint { float x, y; };
ObstaclePoint obstacleMap[MAX_OBSTACLE_POINTS];
int obstacleMapCount = 0;
CaveFeature caveFeatures[CAVE_FEATURES_MAX];
int caveFeatureCount = 0;
GridCell caveGrid[GRID_MAX_X][GRID_MAX_Y];

// ===== Timing =====
unsigned long lastPrintTime = 0;
const long PRINT_INTERVAL = 2000;
unsigned long lastDHTTime = 0;
const long DHT_INTERVAL = 30000;
unsigned long lastFirebaseTime = 0;
const long FIREBASE_INTERVAL = 5000;
unsigned long lastImuTime = 0;
unsigned long lastPositionUpdate = 0;
const long POSITION_UPDATE_INTERVAL = 50;
unsigned long lastFull360Scan = 0;

float latestTemp = 0, latestHum = 0;
bool dhtValid = false;
float kalAngleX = 0, kalAngleY = 0, yawAngle = 0;
bool autoAvoidance = true;
bool mappingMode = true;

// ===== VISION FUSION STATE =====
WiFiUDP visionUdp;
volatile unsigned long visionLastPacket = 0;
int visionStopStreak = 0, visionGoStreak = 0;
bool visionStopConfirmed = false;
unsigned long visionStopStart = 0;
uint8_t visionConfidence = 0;
uint32_t visionPacketCount = 0;
uint32_t visionPacketLost = 0;
uint8_t lastVisionSeq = 0;

// ===== DRIVE ARM LATCH (START / STOP BUTTON) =====
// false at boot = robot waits with motors locked. START button sets it true,
// STOP button clears it. The vision hazard codes (0/1) never touch this latch,
// so an obstacle brake cannot re-arm a robot the operator has stopped.
bool driveArmed = false;

// ===== THREAT SCORE (fusion diagnostic + gate) =====
float threatSonar  = 0.0f;
float threatVision = 0.0f;
float threatFused  = 0.0f;

// ===== MPU helpers =====
bool initMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  if (Wire.available()) {
    byte who = Wire.read();
    if (who != 0x71 && who != 0x70) return false;
  } else return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  delay(100);
  return true;
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, 14);
  if (Wire.available() < 14) return false;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// ===== Ultrasonic =====
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return -1;
  float distanceM = (duration * 0.0343) / 2.0 / 100.0;
  if (distanceM < 0.02 || distanceM > MAX_RANGE) return -1;
  return distanceM;
}

float readUltrasonicDistanceFiltered() {
  float readings[SONAR_SAMPLES];
  int validCount = 0;
  for (int i = 0; i < SONAR_SAMPLES; i++) {
    float d = readUltrasonicDistance();
    if (d > 0) readings[validCount++] = d;
    if (i < SONAR_SAMPLES - 1) delayMicroseconds(5000);
  }
  if (validCount == 0) return -1;
  if (validCount == 1) return readings[0];
  for (int i = 0; i < validCount - 1; i++)
    for (int j = 0; j < validCount - i - 1; j++)
      if (readings[j] > readings[j+1]) {
        float t = readings[j]; readings[j] = readings[j+1]; readings[j+1] = t;
      }
  if (validCount >= 3) return readings[1];
  float sum = 0;
  for (int i = 0; i < validCount; i++) sum += readings[i];
  return sum / validCount;
}

// ===== Servo =====
void setServoAngleNoWait(int angle) {
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  scanServo.write(angle);
  currentServoAngle = angle;
  lastServoCommandedAngle = angle;
  lastServoMoveTime = millis();
}

void setServoAngle(int angle) {
  setServoAngleNoWait(angle);
  delay(SERVO_SETTLE_MS);
}

bool isServoSettled() {
  return (millis() - lastServoMoveTime) >= SERVO_SETTLE_MS;
}

// ===== DRIVE ARM / DISARM (START & STOP BUTTONS) =====
// Re-assertible: STOP always leaves the motors latched OFF, START latches them ON.
void stopMotors();   // forward declaration (defined in the Motor section below)
void setDriveArmed(bool armed) {
  driveArmed = armed;
  if (!driveArmed) stopMotors();
  Serial.print("[DRIVE] ");
  Serial.println(driveArmed
      ? "START -> ARMED (motors enabled, robot rolling)"
      : "STOP  -> DISARMED (motors locked OFF)");
}

// ===== VISION FUSION =====
void checkVisionUDP() {
  while (true) {
    int sz = visionUdp.parsePacket();
    if (sz <= 0) break;
    uint8_t buf[8];
    int n = visionUdp.read(buf, sizeof(buf));
    if (n < 4) continue;
    if (buf[0] != VISION_MAGIC) continue;

    uint8_t cmd  = buf[1];
    uint8_t conf = buf[2];
    uint8_t seq  = buf[3];

    // ---- Operator START / STOP (dashboard buttons) ----
    // Handled first and separately from the hazard codes so they are never
    // swallowed by the vision staleness/confirm bookkeeping below.
    if (cmd == VISION_CMD_OP_START || cmd == VISION_CMD_OP_STOP) {
      bool want = (cmd == VISION_CMD_OP_START);
      if (want != driveArmed) {
        setDriveArmed(want);
      } else if (!driveArmed) {
        stopMotors();                 // re-assert while stopped
      }
      continue;
    }

    if (visionPacketCount > 0 && seq != (uint8_t)(lastVisionSeq + 1))
      visionPacketLost++;
    lastVisionSeq = seq;

    visionLastPacket = millis();
    visionConfidence = conf;
    visionPacketCount++;

    if (cmd == VISION_CMD_HAZARD_STOP) {
      visionStopStreak++;
      visionGoStreak = 0;
      if (visionStopStreak >= VISION_STOP_CONFIRM && !visionStopConfirmed) {
        visionStopConfirmed = true;
        visionStopStart = millis();
        Serial.print("[VISION] STOP confirmed (conf=");
        Serial.print(conf);
        Serial.println(")");
      }
    } else {
      visionGoStreak++;
      visionStopStreak = 0;
      if (visionGoStreak >= VISION_GO_CONFIRM && visionStopConfirmed) {
        visionStopConfirmed = false;
        Serial.println("[VISION] GO confirmed");
      }
    }
  }
}

bool visionFresh() {
  return visionPacketCount > 0 &&
         (millis() - visionLastPacket) < VISION_TIMEOUT_MS;
}

bool visionBlocking() {
  return visionFresh() && visionStopConfirmed;
}

// ===== THREAT SCORE — fused proximity from sonar + vision =====
// Both sonar and vision map to [0..1]; 0 = clear, 1 = imminent.
// Fusion uses max() so either sensor alone can trigger.
void updateThreatScore() {
  // Sonar: 1.0 at 0m, 0.0 at OBSTACLE_THRESHOLD
  if (currentSonarDistance > 0 &&
      currentSonarDistance < OBSTACLE_THRESHOLD) {
    if (currentSonarDistance <= DANGER_THRESHOLD) {
      threatSonar = 1.0f;
    } else {
      threatSonar = 1.0f - (currentSonarDistance - DANGER_THRESHOLD)
                          / (OBSTACLE_THRESHOLD - DANGER_THRESHOLD);
    }
  } else {
    threatSonar = 0.0f;
  }

  // Vision: proportional to confidence when STOP is confirmed
  if (visionFresh() && visionStopConfirmed) {
    threatVision = visionConfidence / 100.0f;
  } else {
    threatVision = 0.0f;
  }

  // Fused: max of the two (either sensor can cause a threat)
  threatFused = threatSonar > threatVision ? threatSonar : threatVision;
}

// ===== Cave mapping =====
void worldToGrid(float x, float y, int &gx, int &gy) {
  gx = (int)((x / MAP_GRID_SIZE) + (GRID_MAX_X / 2));
  gy = (int)((y / MAP_GRID_SIZE) + (GRID_MAX_Y / 2));
  gx = constrain(gx, 0, GRID_MAX_X - 1);
  gy = constrain(gy, 0, GRID_MAX_Y - 1);
}

void updateGridFromSonar(float distance, int servoAngleDeg) {
  if (distance < 0) return;
  float rx = positionEKF.getX();
  float ry = positionEKF.getY();
  float rtheta = positionEKF.getTheta();
  float sonarRad = (servoAngleDeg - 90.0) * PI / 180.0;
  float worldAngle = rtheta + sonarRad;
  float raySteps = distance / (MAP_GRID_SIZE * 0.5);
  for (float step = 0; step < raySteps; step += 1.0) {
    float cd = step * (MAP_GRID_SIZE * 0.5);
    float cx = rx + cd * cos(worldAngle);
    float cy = ry + cd * sin(worldAngle);
    int gx, gy; worldToGrid(cx, cy, gx, gy);
    caveGrid[gx][gy].free_count++;
  }
  float ox = rx + distance * cos(worldAngle);
  float oy = ry + distance * sin(worldAngle);
  int gx, gy; worldToGrid(ox, oy, gx, gy);
  caveGrid[gx][gy].wall_count += 2;
}

void addObstaclePoint(float distance, int servoAngleDeg) {
  if (obstacleMapCount >= MAX_OBSTACLE_POINTS) {
    for (int i = 0; i < MAX_OBSTACLE_POINTS - 1; i++)
      obstacleMap[i] = obstacleMap[i + 1];
    obstacleMapCount = MAX_OBSTACLE_POINTS - 1;
  }
  float rx = positionEKF.getX();
  float ry = positionEKF.getY();
  float rtheta = positionEKF.getTheta();
  float sonarRad = (servoAngleDeg - 90.0) * PI / 180.0;
  float worldAngle = rtheta + sonarRad;
  float px = rx + distance * cos(worldAngle);
  float py = ry + distance * sin(worldAngle);
  for (int i = 0; i < obstacleMapCount; i++)
    if (fabs(obstacleMap[i].x - px) < 0.08 && fabs(obstacleMap[i].y - py) < 0.08)
      return;
  obstacleMap[obstacleMapCount].x = px;
  obstacleMap[obstacleMapCount].y = py;
  obstacleMapCount++;
  updateGridFromSonar(distance, servoAngleDeg);
}

void detectCaveFeatures() {
  if (obstacleMapCount < 5) return;
  float rx = positionEKF.getX();
  float ry = positionEKF.getY();
  float rtheta = positionEKF.getTheta();
  int wallsDetected[8] = {0};
  float dirH[8] = {0, 45, 90, 135, 180, 225, 270, 315};
  for (int dir = 0; dir < 8; dir++) {
    float heading = (dirH[dir] * PI / 180.0);
    float cd = 0.5;
    float cx = rx + cd * cos(heading);
    float cy = ry + cd * sin(heading);
    for (int i = 0; i < obstacleMapCount; i++) {
      float dx = obstacleMap[i].x - cx;
      float dy = obstacleMap[i].y - cy;
      if (sqrt(dx*dx + dy*dy) < 0.15) wallsDetected[dir]++;
    }
  }
  int openDirections = 0;
  for (int i = 0; i < 8; i++) if (wallsDetected[i] == 0) openDirections++;
  CaveFeatureType ft = FEATURE_WALL;
  if (openDirections >= 4) ft = FEATURE_CHAMBER;
  else if (openDirections == 1) ft = FEATURE_DEAD_END;
  else if (openDirections >= 2) ft = FEATURE_JUNCTION;
  for (int i = 0; i < caveFeatureCount; i++) {
    float dx = caveFeatures[i].x - rx;
    float dy = caveFeatures[i].y - ry;
    if (sqrt(dx*dx + dy*dy) < 0.30) return;
  }
  if (caveFeatureCount < CAVE_FEATURES_MAX) {
    caveFeatures[caveFeatureCount].x = rx;
    caveFeatures[caveFeatureCount].y = ry;
    caveFeatures[caveFeatureCount].type = ft;
    caveFeatures[caveFeatureCount].heading = rtheta;
    caveFeatures[caveFeatureCount].timestamp = millis();
    caveFeatures[caveFeatureCount].confidence = 1;
    caveFeatureCount++;
    Serial.print("CAVE FEATURE: ");
    switch(ft) {
      case FEATURE_CHAMBER: Serial.println("CHAMBER detected"); break;
      case FEATURE_JUNCTION: Serial.println("JUNCTION detected"); break;
      case FEATURE_DEAD_END: Serial.println("DEAD END detected"); break;
      case FEATURE_NARROW_PASS: Serial.println("NARROW PASSAGE"); break;
      default: Serial.println("WALL"); break;
    }
  }
}

bool performFull360Scan() {
  if (!fullScanInProgress) {
    fullScanInProgress = true;
    scanStepIndex = 0;
    lastScanStepTime = millis();
    setServoAngleNoWait(scanAngles[0]);
    Serial.println(">>> FULL 360 CAVE SCAN STARTED <<<");
    return false;
  }
  unsigned long elapsed = millis() - lastScanStepTime;
  if (elapsed >= SERVO_SETTLE_MS) {
    float d = readUltrasonicDistanceFiltered();
    if (d > 0) {
      addObstaclePoint(d, scanAngles[scanStepIndex]);
      currentSonarDistance = d;
    }
    scanStepIndex++;
    if (scanStepIndex >= SCAN_ANGLE_COUNT) {
      setServoAngleNoWait(SERVO_CENTER);
      fullScanInProgress = false;
      detectCaveFeatures();
      Serial.println(">>> CAVE SCAN COMPLETE <<<");
      return true;
    }
    setServoAngleNoWait(scanAngles[scanStepIndex]);
    lastScanStepTime = millis();
  }
  return false;
}

// ===== Encoder ISRs =====
void leftEncoderISR() { leftEncoderCount++; }
void rightEncoderISR() { rightEncoderCount++; }

// ===== Odometry =====
void updatePosition() {
  noInterrupts();
  unsigned long cl = leftEncoderCount;
  unsigned long cr = rightEncoderCount;
  interrupts();
  unsigned long now = millis();
  float dt = (now - lastEncoderTime) / 1000.0;
  if (dt <= 0) return;
  long ld = cl - lastLeftCount;
  long rd = cr - lastRightCount;
  float ldist = (ld / (float)COUNTS_PER_REV) * (2 * PI * WHEEL_RADIUS);
  float rdist = (rd / (float)COUNTS_PER_REV) * (2 * PI * WHEEL_RADIUS);
  positionEKF.predict(ldist, rdist);
  positionEKF.updateIMU(yawAngle * PI / 180.0);
  lastLeftCount = cl;
  lastRightCount = cr;
  lastEncoderTime = now;
}

void readAndStoreDHT() {
  float t = dht.getTemperature();
  float h = dht.getHumidity();
  if (isnan(t) || isnan(h)) dhtValid = false;
  else { latestTemp = t; latestHum = h; dhtValid = true; }
}

// ===== MOTOR COMMAND MIRROR (remote debug telemetry) =====
// Records exactly what the driver pins were last told, so Firebase / Serial
// can prove whether a "no movement" report is firmware or hardware.
const char* motorCmd = "STOP";
int motorPwmA = 0;
int motorPwmB = 0;

void setMotorTelemetry(const char* cmd, int ena, int enb) {
  bool changed = (strcmp(cmd, motorCmd) != 0) || ena != motorPwmA || enb != motorPwmB;
  motorCmd = cmd; motorPwmA = ena; motorPwmB = enb;
  if (changed) {
    Serial.print("[MOTOR] "); Serial.print(motorCmd);
    Serial.print("  ENA="); Serial.print(motorPwmA);
    Serial.print("  ENB="); Serial.println(motorPwmB);
  }
}

// ===== Motor =====
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  setMotorTelemetry("STOP", 0, 0);
}
void moveForwardSlow() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SLOW_SPEED);
  analogWrite(ENB, SLOW_SPEED);
  setMotorTelemetry("FORWARD", SLOW_SPEED, SLOW_SPEED);
}
void turnLeftCurve(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed / 3);
  analogWrite(ENB, speed);
  setMotorTelemetry("TURN_LEFT", speed / 3, speed);
}
void turnRightCurve(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed / 3);
  setMotorTelemetry("TURN_RIGHT", speed, speed / 3);
}
void moveBackward(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  setMotorTelemetry("BACKWARD", speed, speed);
}
// === NEW: pure in-place spin for stuck recovery ===
void spinInPlaceLeft(int speed) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);   // left wheel backward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);    // right wheel forward
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  setMotorTelemetry("SPIN_LEFT", speed, speed);
}

// ============================================================
// AVOIDANCE — VISION + SONAR FUSION, IMPROVED BACKUP
// ============================================================
void obstacleAvoidanceSimple() {
  unsigned long now = millis();
  enum AvoidanceState {
    MOVING_FORWARD, BACKING_UP, SCANNING_SIDES,
    TURNING_LEFT, TURNING_RIGHT, WAITING_SERVO,
    SPIN_180                               // === NEW ===
  };
  static AvoidanceState state = MOVING_FORWARD;
  static unsigned long stateStart = 0;
  static unsigned long lastForwardScan = 0;
  static int scanPhase = 0;
  static float leftDist = 0;
  static unsigned long backupEncL = 0, backupEncR = 0;   // === NEW ===
  static int consecBackups = 0;                          // === NEW ===
  static unsigned long forwardSince = 0;                 // === NEW ===

  // ---- Reset consecutive-backup counter after stable forward motion ----
  if (state == MOVING_FORWARD && forwardSince > 0 &&
      (now - forwardSince) >= FORWARD_STABLE_MS) {
    if (consecBackups != 0) {
      Serial.print("[RECOVERY] forward stable — resetting consecBackups (was ");
      Serial.print(consecBackups);
      Serial.println(")");
      consecBackups = 0;
    }
  }

  // ============ VISION FUSION GATE ============
  if (state == MOVING_FORWARD && visionBlocking()) {
    stopMotors();
    unsigned long blockedFor = now - visionStopStart;
    static unsigned long lastVisionLog = 0;
    if (now - lastVisionLog > 1000) {
      Serial.print("[VISION] BLOCKED ");
      Serial.print(blockedFor);
      Serial.print(" ms | conf=");
      Serial.print(visionConfidence);
      Serial.print(" | threat=");
      Serial.println(threatFused, 2);
      lastVisionLog = now;
    }
    if (blockedFor >= VISION_STUCK_TIMEOUT) {
      Serial.println("[VISION] persistent — escalating to avoidance");
      state = SCANNING_SIDES;
      scanPhase = 0;
      stateStart = now;
      setServoAngleNoWait(SERVO_MIN);
    }
    return;
  }
  // ============================================

  if (state == MOVING_FORWARD) {
    if (forwardSince == 0) forwardSince = now;
    if (now - lastForwardScan >= SCAN_INTERVAL) {
      if (currentServoAngle != SERVO_CENTER) {
        setServoAngleNoWait(SERVO_CENTER);
        state = WAITING_SERVO;
        stateStart = now;
        return;
      }
      float frontDist = readUltrasonicDistanceFiltered();
      lastForwardScan = now;
      if (frontDist < 0) {
        currentSonarDistance = -1;
        obstacleDetected = false;
        moveForwardSlow();
        return;
      }
      currentSonarDistance = frontDist;
      if (mappingMode) addObstaclePoint(frontDist, SERVO_CENTER);

      if (frontDist < OBSTACLE_THRESHOLD) {
        obstacleDetected = true;
        Serial.print("OBSTACLE DETECTED: ");
        Serial.print(frontDist, 3);
        Serial.print(" m | threat=");
        Serial.println(threatFused, 2);
        stopMotors();

        if (frontDist < DANGER_THRESHOLD) {
          Serial.println("DANGER ZONE - short backup");
          // === NEW: record encoder positions at backup start ===
          noInterrupts();
          backupEncL = leftEncoderCount;
          backupEncR = rightEncoderCount;
          interrupts();
          consecBackups++;
          state = BACKING_UP;
          stateStart = now;
          moveBackward(BACKUP_SPEED);
        } else {
          Serial.println("Scanning left/right");
          state = SCANNING_SIDES;
          scanPhase = 0;
          stateStart = now;
          setServoAngleNoWait(SERVO_MIN);
        }
      } else {
        obstacleDetected = false;
        moveForwardSlow();
      }
    } else {
      moveForwardSlow();
    }
  }

  // === NEW: encoder-based, distance-limited, time-capped backup ===
  else if (state == BACKING_UP) {
    noInterrupts();
    unsigned long cl = leftEncoderCount;
    unsigned long cr = rightEncoderCount;
    interrupts();
    long dL = (long)cl - (long)backupEncL;
    long dR = (long)cr - (long)backupEncR;
    float avgCounts = (fabs((float)dL) + fabs((float)dR)) / 2.0f;
    float distBacked = (avgCounts / (float)COUNTS_PER_REV) * (2 * PI * WHEEL_RADIUS);

    bool distDone = distBacked >= BACKUP_DISTANCE_M;
    bool timeDone = (now - stateStart) >= BACKUP_MAX_MS;

    if (distDone || timeDone) {
      stopMotors();
      Serial.print("[BACKUP] done at ");
      Serial.print(distBacked * 100.0, 1);
      Serial.print(" cm (");
      Serial.print(timeDone ? "timeout" : "distance");
      Serial.print(") | consec=");
      Serial.println(consecBackups);
      state = SCANNING_SIDES;
      scanPhase = 0;
      stateStart = now;
      setServoAngleNoWait(SERVO_MIN);
    }
  }

  else if (state == SCANNING_SIDES) {
    unsigned long elapsed = now - stateStart;
    if (scanPhase == 0) {
      if (elapsed >= SERVO_SETTLE_MS) {
        leftDist = readUltrasonicDistanceFiltered();
        if (leftDist > 0 && mappingMode) addObstaclePoint(leftDist, SERVO_MIN);
        if (leftDist > 0) {
          Serial.print("LEFT: "); Serial.print(leftDist, 3); Serial.println(" m");
        }
        setServoAngleNoWait(SERVO_MAX);
        scanPhase = 1;
        stateStart = now;
      }
    } else {
      if (elapsed >= SERVO_SETTLE_MS) {
        float rightDist = readUltrasonicDistanceFiltered();
        if (rightDist > 0 && mappingMode) addObstaclePoint(rightDist, SERVO_MAX);
        if (rightDist > 0) {
          Serial.print("RIGHT: "); Serial.print(rightDist, 3); Serial.println(" m");
        }

        if (rightDist > leftDist && rightDist > DANGER_THRESHOLD) {
          Serial.println("Turning RIGHT");
          state = TURNING_RIGHT;
        } else if (leftDist > DANGER_THRESHOLD) {
          Serial.println("Turning LEFT");
          state = TURNING_LEFT;
        } else {
          // === NEW: cap consecutive backups; spin 180 if stuck ===
          Serial.print("Both sides blocked. consecBackups=");
          Serial.println(consecBackups);
          if (consecBackups < MAX_CONSEC_BACKUPS) {
            Serial.println("Backing up again (within limit)");
            noInterrupts();
            backupEncL = leftEncoderCount;
            backupEncR = rightEncoderCount;
            interrupts();
            consecBackups++;
            state = BACKING_UP;
            stateStart = now;
            moveBackward(BACKUP_SPEED);
          } else {
            Serial.println("STUCK — doing 180° spin-in-place");
            state = SPIN_180;
            stateStart = now;
            // Reset immediate servo position (not strictly required)
            setServoAngleNoWait(SERVO_CENTER);
          }
          return;
        }
        stateStart = now;
        setServoAngleNoWait(SERVO_CENTER);
      }
    }
  }

  else if (state == TURNING_LEFT) {
    if (now - stateStart < 300) turnLeftCurve(TURN_SPEED);
    else {
      stopMotors();
      state = MOVING_FORWARD;
      forwardSince = now;
      lastForwardScan = now + 200;
      moveForwardSlow();
    }
  }

  else if (state == TURNING_RIGHT) {
    if (now - stateStart < 300) turnRightCurve(TURN_SPEED);
    else {
      stopMotors();
      state = MOVING_FORWARD;
      forwardSince = now;
      lastForwardScan = now + 200;
      moveForwardSlow();
    }
  }

  else if (state == WAITING_SERVO) {
    if (isServoSettled()) {
      state = MOVING_FORWARD;
      forwardSince = now;
      lastForwardScan = now;
    }
  }

  // === NEW: 180° in-place spin — last-resort escape ===
  else if (state == SPIN_180) {
    if (now - stateStart >= SPIN_180_MS) {
      stopMotors();
      consecBackups = 0;
      forwardSince = 0;         // force a fresh stability timer
      state = MOVING_FORWARD;
      lastForwardScan = now + 200;
      Serial.println("[SPIN_180] complete — resuming forward");
      moveForwardSlow();
    } else {
      spinInPlaceLeft(TURN_SPEED);
    }
  }
}

// ===== Firebase =====
bool sendToFirebase(String json) {
  String url = "https://" + String(FIREBASE_HOST) + "/sensor_readings.json?auth=" + String(FIREBASE_API_KEY);
  if (!client.connect(FIREBASE_HOST, 443)) return false;
  String request = "POST " + url.substring(url.indexOf('/', 8)) + " HTTP/1.1\r\n";
  request += "Host: " + String(FIREBASE_HOST) + "\r\n";
  request += "Content-Type: application/json\r\n";
  request += "Content-Length: " + String(json.length()) + "\r\n";
  request += "Connection: close\r\n\r\n";
  request += json;
  client.print(request);
  unsigned long timeout = millis() + 5000;
  while (!client.available() && millis() < timeout) delay(10);
  if (client.available()) {
    while (client.available()) client.read();
    client.stop();
    return true;
  }
  client.stop();
  return false;
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(1000);

  Serial.println("Booting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifiStart > 30000) {
      Serial.println();
      Serial.println("!!! WiFi connect timeout.");
      while (true) delay(1000);
    }
  }
  Serial.println();

  visionUdp.begin(VISION_PORT);
  Serial.println("===========================================");
  Serial.print  ("  >>> Arduino IP: ");
  Serial.println(WiFi.localIP());
  Serial.print  ("  >>> Vision UDP listening on port ");
  Serial.println(VISION_PORT);
  Serial.println("  >>> Set this IP as ARDUINO_IP in the Python script");
  Serial.println("===========================================");

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER);
  currentServoAngle = SERVO_CENTER;
  delay(300);

  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  lastLeftCount = leftEncoderCount;
  lastRightCount = rightEncoderCount;
  lastEncoderTime = millis();

  if (initMPU()) {
    Serial.println("MPU initialized.");
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      float accX = ax / 16384.0, accY = ay / 16384.0, accZ = az / 16384.0;
      float roll  = atan2(accY, accZ) * 180.0 / PI;
      float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
      kalmanX.setAngle(roll);
      kalmanY.setAngle(pitch);
    }
    lastImuTime = micros();
  }

  dht.setup(DHTPIN, DHTesp::DHT22);
  readAndStoreDHT();

  for (int i = 0; i < GRID_MAX_X; i++)
    for (int j = 0; j < GRID_MAX_Y; j++) {
      caveGrid[i][j].wall_count = 0;
      caveGrid[i][j].free_count = 0;
    }

  lastDHTTime = millis();
  lastPrintTime = millis();
  lastFirebaseTime = millis();
  lastPositionUpdate = millis();
  lastFullScanTime = millis();
  lastFull360Scan = millis();

  Serial.println("========================================");
  Serial.println("CAVE EXPLORATION ROBOT READY + VISION");
  Serial.println("========================================");
  Serial.println("Commands:");
  Serial.println("  START=UI button / f = arm & drive");
  Serial.println("  STOP =UI button / s = disarm & brake");
  Serial.println("  p=position, r=reset, c=scan, g=full360");
  Serial.println("  m=manual, a=auto, t=mapping_toggle, v=vision_status");
  Serial.println("  x=motor bench test (proves wiring / PWM)");
  Serial.println("========================================");
  stopMotors();     // boot safe: motors locked until START is pressed
  setDriveArmed(false);
  Serial.println(">>> MOTORS LOCKED — press START to drive <<<");
}

// ===== Loop =====
void loop() {
  unsigned long now = millis();

  checkVisionUDP();
  updateThreatScore();     // <-- refresh fused threat every loop

  int16_t ax, ay, az, gx, gy, gz;
  bool imuOK = readMPU(ax, ay, az, gx, gy, gz);
  if (imuOK) {
    unsigned long cm = micros();
    float dt = (float)(cm - lastImuTime) / 1000000.0f;
    lastImuTime = cm;
    float accX = ax / 16384.0, accY = ay / 16384.0, accZ = az / 16384.0;
    float gyrX = gx / 131.0, gyrY = gy / 131.0, gyrZ = gz / 131.0;
    float roll  = atan2(accY, accZ) * 180.0 / PI;
    float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    kalAngleX = kalmanX.getAngle(roll, gyrX, dt);
    kalAngleY = kalmanY.getAngle(pitch, gyrY, dt);
    yawAngle += gyrZ * dt;
    while (yawAngle > 180) yawAngle -= 360;
    while (yawAngle < -180) yawAngle += 360;
  }

  if (now - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
    updatePosition();
    lastPositionUpdate = now;
  }

  if (Serial.available()) {
    char cmd = Serial.read();
    switch(cmd) {
      case 'p':
        Serial.print("X: "); Serial.print(positionEKF.getX(), 3);
        Serial.print(" Y: "); Serial.print(positionEKF.getY(), 3);
        Serial.print(" Th: "); Serial.print(positionEKF.getTheta() * 180.0 / PI, 1);
        Serial.print("deg Map: "); Serial.print(obstacleMapCount);
        Serial.print(" Features: "); Serial.print(caveFeatureCount);
        Serial.print(" Unc: "); Serial.println(positionEKF.getPosUncertainty(), 3);
        break;
      case 'r':
        positionEKF.reset();
        noInterrupts();
        leftEncoderCount = 0; rightEncoderCount = 0;
        interrupts();
        lastLeftCount = 0; lastRightCount = 0;
        yawAngle = 0;
        obstacleMapCount = 0; caveFeatureCount = 0;
        lastEncoderTime = millis();
        Serial.println("RESET");
        break;
      case 's': setDriveArmed(false); Serial.println("Stopped (DISARMED)"); break;
      case 'f':
        setServoAngleNoWait(SERVO_CENTER);
        setDriveArmed(true);
        moveForwardSlow();
        Serial.println("Forward (ARMED)");
        break;
      case 'm': autoAvoidance = false; Serial.println("MANUAL"); break;
      case 'a': autoAvoidance = true; Serial.println("AUTO ON"); break;
      case 'g':
        Serial.println(">>> FULL 360 SCAN <<<");
        stopMotors();
        while (!performFull360Scan()) {
          checkVisionUDP();                 // a STOP press must abort the scan
          if (!driveArmed) break;
          delay(10);
        }
        if (!driveArmed && fullScanInProgress) {
          fullScanInProgress = false;
          setServoAngleNoWait(SERVO_CENTER);
          Serial.println("[SCAN] aborted by STOP");
        }
        Serial.print("Points: "); Serial.print(obstacleMapCount);
        Serial.print(" Features: "); Serial.println(caveFeatureCount);
        if (driveArmed) moveForwardSlow(); else stopMotors();
        break;
      case 't':
        mappingMode = !mappingMode;
        Serial.print("Mapping Mode: ");
        Serial.println(mappingMode ? "ON" : "OFF");
        break;
      case 'x':
        // ===== MOTOR BENCH TEST — bypasses all logic to isolate the fault =====
        Serial.println("[TEST] Phase A: full speed (digitalWrite ENA/ENB HIGH)...");
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
        setMotorTelemetry("TEST_FULL", 255, 255);
        delay(1500);
        stopMotors();
        delay(400);
        Serial.println("[TEST] Phase B: PWM 200 (analogWrite)...");
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        analogWrite(ENA, 200); analogWrite(ENB, 200);
        setMotorTelemetry("TEST_PWM", 200, 200);
        delay(1500);
        stopMotors();
        Serial.println("[TEST] done. Report: wheels spun in A? B? neither?");
        break;
      case 'v':
        Serial.print("[VISION] fresh=");
        Serial.print(visionFresh() ? "YES" : "NO");
        Serial.print(" stop=");
        Serial.print(visionStopConfirmed ? "YES" : "NO");
        Serial.print(" conf=");
        Serial.print(visionConfidence);
        Serial.print(" pkts=");
        Serial.print(visionPacketCount);
        Serial.print(" lost=");
        Serial.print(visionPacketLost);
        Serial.print(" | threat sonar=");
        Serial.print(threatSonar, 2);
        Serial.print(" vision=");
        Serial.print(threatVision, 2);
        Serial.print(" fused=");
        Serial.println(threatFused, 2);
        break;
    }
  }

  // ===== START / STOP DRIVE LATCH =====
  if (!driveArmed) {
    stopMotors();                       // STOP pressed (or boot): motors held OFF
  } else if (autoAvoidance && !fullScanInProgress) {
    obstacleAvoidanceSimple();
  } else if (!autoAvoidance) {
    // MANUAL tele-op: START rolls forward, vision hazard still brakes us
    if (visionBlocking()) stopMotors();
    else moveForwardSlow();
  }

  if (autoAvoidance && mappingMode && !fullScanInProgress &&
      (now - lastFull360Scan >= FULL_SCAN_INTERVAL)) {
    Serial.println("\n>>> PERIODIC CAVE MAPPING SCAN <<<\n");
    lastFull360Scan = now;
  }

  if (now - lastDHTTime >= DHT_INTERVAL) {
    readAndStoreDHT();
    lastDHTTime = now;
  }

  if (now - lastPrintTime >= PRINT_INTERVAL) {
    printData(imuOK);
    lastPrintTime = now;
  }

  if (now - lastFirebaseTime >= FIREBASE_INTERVAL) {
    String json = buildJSON(imuOK);
    sendToFirebase(json);
    lastFirebaseTime = now;
  }

  delay(5);
}

// ===== JSON =====
String buildJSON(bool imuOK) {
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"position\":{";
  json += "\"x\":" + String(positionEKF.getX(), 3) + ",";
  json += "\"y\":" + String(positionEKF.getY(), 3) + ",";
  json += "\"theta\":" + String(positionEKF.getTheta(), 4) + ",";
  json += "\"uncertainty\":" + String(positionEKF.getPosUncertainty(), 3);
  json += "},";
  noInterrupts();
  unsigned long lc = leftEncoderCount, rc = rightEncoderCount;
  interrupts();
  json += "\"encoders\":{\"left\":" + String(lc) + ",\"right\":" + String(rc) + "},";
  if (imuOK) {
    json += "\"orientation\":{";
    json += "\"roll_x\":" + String(kalAngleX, 2) + ",";
    json += "\"pitch_y\":" + String(kalAngleY, 2) + ",";
    json += "\"yaw\":" + String(yawAngle, 2);
    json += "},";
  } else json += "\"orientation\":\"error\",";
  json += "\"armed\":" + String(driveArmed ? "true" : "false") + ",";
  json += "\"motors\":{\"cmd\":\"" + String(motorCmd) +
          "\",\"ena\":" + String(motorPwmA) +
          ",\"enb\":" + String(motorPwmB) + "},";
  json += "\"sonar\":{";
  json += "\"distance\":" + String(currentSonarDistance, 3) + ",";
  json += "\"servo_angle\":" + String(currentServoAngle) + ",";
  json += "\"obstacle\":" + String(obstacleDetected ? "true" : "false");
  json += "},";
  json += "\"vision\":{";
  json += "\"fresh\":" + String(visionFresh() ? "true" : "false") + ",";
  json += "\"stop\":" + String(visionStopConfirmed ? "true" : "false") + ",";
  json += "\"conf\":" + String(visionConfidence) + ",";
  json += "\"packets\":" + String(visionPacketCount) + ",";
  json += "\"lost\":" + String(visionPacketLost);
  json += "},";
  // === NEW: fused threat score ===
  json += "\"threat\":{";
  json += "\"sonar\":" + String(threatSonar, 3) + ",";
  json += "\"vision\":" + String(threatVision, 3) + ",";
  json += "\"fused\":" + String(threatFused, 3);
  json += "},";
  json += "\"cave_map\":[";
  int startIdx = (obstacleMapCount > 50) ? (obstacleMapCount - 50) : 0;
  for (int i = startIdx; i < obstacleMapCount; i++) {
    if (i > startIdx) json += ",";
    json += "{\"x\":" + String(obstacleMap[i].x, 3) +
            ",\"y\":" + String(obstacleMap[i].y, 3) + "}";
  }
  json += "],";
  json += "\"cave_features\":{\"count\":" + String(caveFeatureCount) + ",\"features\":[";
  for (int i = 0; i < caveFeatureCount; i++) {
    if (i > 0) json += ",";
    json += "{\"x\":" + String(caveFeatures[i].x, 3) +
            ",\"y\":" + String(caveFeatures[i].y, 3) +
            ",\"type\":" + String(caveFeatures[i].type) + "}";
  }
  json += "]},";
  if (dhtValid)
    json += "\"temp\":" + String(latestTemp, 1) + ",\"humidity\":" + String(latestHum, 1);
  else
    json += "\"temp\":\"error\",\"humidity\":\"error\"";
  json += "}";
  return json;
}

void printData(bool imuOK) {
  Serial.println("\n========== CAVE MAP DATA ==========");
  Serial.print("Position: (");
  Serial.print(positionEKF.getX(), 2); Serial.print(", ");
  Serial.print(positionEKF.getY(), 2); Serial.print(") ");
  Serial.print("Heading: ");
  Serial.print(positionEKF.getTheta() * 180.0 / PI, 1);
  Serial.print("deg Unc: ");
  Serial.println(positionEKF.getPosUncertainty(), 3);
  Serial.print("Sonar: ");
  Serial.print(currentSonarDistance, 2);
  Serial.print(" m @ "); Serial.print(currentServoAngle);
  Serial.print("deg | Obstacle: "); Serial.println(obstacleDetected ? "YES" : "no");

  Serial.print("[VISION] ");
  if (!visionFresh()) {
    Serial.print("STALE (last packet ");
    if (visionPacketCount == 0) Serial.print("NEVER RECEIVED");
    else {
      Serial.print((millis() - visionLastPacket) / 1000.0, 1);
      Serial.print("s ago");
    }
    Serial.println(")  <-- check ARDUINO_IP in Python script!");
  } else {
    Serial.print(visionStopConfirmed ? "STOP" : "GO");
    Serial.print(" conf=");
    Serial.print(visionConfidence);
    Serial.print(" | pkts=");
    Serial.print(visionPacketCount);
    Serial.print(" lost=");
    Serial.println(visionPacketLost);
  }

  // === NEW: fusion diagnostic line ===
  Serial.print("[THREAT] sonar=");
  Serial.print(threatSonar, 2);
  Serial.print(" vision=");
  Serial.print(threatVision, 2);
  Serial.print(" fused=");
  Serial.println(threatFused, 2);

  Serial.print("[MOTOR] cmd="); Serial.print(motorCmd);
  Serial.print(" ENA="); Serial.print(motorPwmA);
  Serial.print(" ENB="); Serial.println(motorPwmB);

  Serial.print("Map Points: "); Serial.print(obstacleMapCount);
  Serial.print(" | Cave Features: "); Serial.println(caveFeatureCount);
  if (caveFeatureCount > 0) {
    Serial.print("Last Feature: ");
    switch(caveFeatures[caveFeatureCount-1].type) {
      case FEATURE_CHAMBER: Serial.print("CHAMBER"); break;
      case FEATURE_JUNCTION: Serial.print("JUNCTION"); break;
      case FEATURE_DEAD_END: Serial.print("DEAD END"); break;
      case FEATURE_NARROW_PASS: Serial.print("NARROW"); break;
      default: Serial.print("WALL"); break;
    }
    Serial.println();
  }
  Serial.println("====================================\n");
}
