#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "BluetoothSerial.h"

// === BLUETOOTH CONFIGURATION ===
BluetoothSerial SerialBT;
char btCommand = 'S'; // default: stop

// === BUZZER CONFIGURATION ===
#define BUZZER_PIN 23

// === MPU CONFIGURATION ===
MPU6050 mpu;
#define INTERRUPT_PIN 4

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy;
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// === PID CONFIGURATION ===
#define SETPOINT_PITCH_ANGLE_OFFSET -1.1
double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;
double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;

#define PID_MIN_LIMIT -225
#define PID_MAX_LIMIT 225
#define PID_SAMPLE_TIME_IN_MILLI 10

#define PID_PITCH_KP 100
#define PID_PITCH_KI 150
#define PID_PITCH_KD 2

#define PID_YAW_KP 0.1
#define PID_YAW_KI 0.1
#define PID_YAW_KD 0

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

// === MOTOR PIN CONFIGURATION ===
#define ENA_PIN     33
#define IN1_PIN     14
#define IN2_PIN     27
#define IN3_PIN     26
#define IN4_PIN     25
#define ENB_PIN     32

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define CHANNEL_A 0
#define CHANNEL_B 1

#define MIN_ABSOLUTE_SPEED 10
#define MIN_MOVEMENT_THRESHOLD 15
#define MAX_SAFE_ANGLE 35

void setupMotors() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  ledcSetup(CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA_PIN, CHANNEL_A);

  ledcSetup(CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENB_PIN, CHANNEL_B);

  rotateMotor(0, 0);
}

void rotateMotor(int speed1, int speed2) {
  if (abs(speed1) < MIN_MOVEMENT_THRESHOLD) speed1 = 0;
  if (abs(speed2) < MIN_MOVEMENT_THRESHOLD) speed2 = 0;

  if (speed1 < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }

  if (speed2 < 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  }

  speed1 = constrain(abs(speed1) + (speed1 != 0 ? MIN_ABSOLUTE_SPEED : 0), 0, 255);
  speed2 = constrain(abs(speed2) + (speed2 != 0 ? MIN_ABSOLUTE_SPEED : 0), 0, 255);

  ledcWrite(CHANNEL_A, speed1);
  ledcWrite(CHANNEL_B, speed2);
}

void setupMPU() {
  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-522);
  mpu.setYAccelOffset(379);
  mpu.setZAccelOffset(1822);
  mpu.setXGyroOffset(-66);
  mpu.setYGyroOffset(37);
  mpu.setZGyroOffset(-89);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("DMP Init failed");
  }
}

void setupPID() {
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void handleBluetoothCommand() {
  if (SerialBT.available()) {
    btCommand = SerialBT.read();
    Serial.print("BT Command: ");
    Serial.println(btCommand);
  }

  switch (btCommand) {
    case 'F': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET - 3; setpointYawRate = 0; break;
    case 'B': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET + 3; setpointYawRate = 0; break;
    case 'L': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET; setpointYawRate = -60; break;
    case 'R': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET; setpointYawRate = 60; break;
    case 'G': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET - 3; setpointYawRate = -60; break;
    case 'I': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET - 3; setpointYawRate = 60; break;
    case 'H': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET + 3; setpointYawRate = -60; break;
    case 'J': setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET + 3; setpointYawRate = 60; break;
    case 'V': digitalWrite(BUZZER_PIN, HIGH); break;
    case 'v': digitalWrite(BUZZER_PIN, LOW); break;
    case 'S':
    default:
      setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
      setpointYawRate = 0;
      break;
  }
}

double limitYawEffect(double yawOut, double pitchOut) {
  double maxYaw = abs(pitchOut) * 0.6;
  return constrain(yawOut, -maxYaw, maxYaw);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot");
  Serial.println("Bluetooth ready. Pair with 'ESP32_Robot'");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  setupMotors();
  setupMPU();
  setupPID();
}

void loop() {
  handleBluetoothCommand();

  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    pitchGyroAngle = ypr[1] * 180.0 / M_PI;
    yawGyroRate = gy.z / 131.0;  // Konversi ke derajat/detik (default skala MPU6050)

    // Safety: Jika terlalu miring, matikan motor
    if (abs(pitchGyroAngle) > MAX_SAFE_ANGLE) {
      rotateMotor(0, 0);
      return;
    }

    pitchPID.Compute();
    yawPID.Compute();

    double limitedYawOutput = limitYawEffect(yawPIDOutput, pitchPIDOutput);
    rotateMotor(pitchPIDOutput + limitedYawOutput, pitchPIDOutput - limitedYawOutput);
  }
}
