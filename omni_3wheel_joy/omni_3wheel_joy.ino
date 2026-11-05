#include <Arduino.h>
#include <Bluepad32.h>
#include <math.h>

// ============================================================
// 3-Wheel Omni Robot
// ESP32 + PS4 + Bluepad32 + ZX-SONAR1M x 3
// ============================================================

// ============================================================
// Motor pins
// ============================================================
// Motor 1: Front-left

constexpr uint8_t MOTOR1_PWM_PIN = 25;
constexpr uint8_t MOTOR1_DIR_PIN = 26;
// Motor 2: Front-right
constexpr uint8_t MOTOR2_PWM_PIN = 27;
constexpr uint8_t MOTOR2_DIR_PIN = 14;
// Motor 3: Rear
constexpr uint8_t MOTOR3_PWM_PIN = 33;
constexpr uint8_t MOTOR3_DIR_PIN = 32;
// ============================================================
// ZX-SONAR1M pins
// ============================================================
constexpr uint8_t SONAR_FRONT_PIN = 36;
constexpr uint8_t SONAR_LEFT_PIN = 34;
constexpr uint8_t SONAR_RIGHT_PIN = 39;
// ============================================================

// PWM configuration
// ESP32 Arduino Core 3.x
// ============================================================

constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t MOTOR1_PWM_CHANNEL = 0;
constexpr uint8_t MOTOR2_PWM_CHANNEL = 1;
constexpr uint8_t MOTOR3_PWM_CHANNEL = 2;
constexpr int PWM_MAX = 255;
constexpr int MOTOR_MIN_PWM = 45;
// ============================================================

// Motor direction correction
//
// เปลี่ยนเป็น -1 หากล้อนั้นหมุนกลับทิศ
// ============================================================

constexpr int MOTOR1_DIRECTION = 1;
constexpr int MOTOR2_DIRECTION = 1;
constexpr int MOTOR3_DIRECTION = 1;
// ============================================================
// Robot dimensions
// ============================================================

// ระยะจากศูนย์กลางหุ่นยนต์ถึงล้อ หน่วยเมตร
constexpr float ROBOT_RADIUS = 0.18f;
// ============================================================
// Manual mode speed
// ============================================================
constexpr float MAX_MANUAL_VX = 0.80f;
constexpr float MAX_MANUAL_VY = 0.80f;
constexpr float MAX_MANUAL_WZ = 2.20f;
float manualSpeedScale = 0.50f;
constexpr float MIN_SPEED_SCALE = 0.20f;
constexpr float MAX_SPEED_SCALE = 1.00f;
// ============================================================
// Automatic mode speed
//
// ค่าทั้งหมดเป็นค่าปกติช่วง -1.0 ถึง 1.0
// ============================================================
// เดินหน้า
constexpr float AUTO_FORWARD_SPEED = 0.32f;
// ความเร็วเลื่อนหลบซ้าย-ขวา
constexpr float AUTO_STRAFE_SPEED = 0.38f;
// ความเร็วถอยหลัง
constexpr float AUTO_REVERSE_SPEED = 0.30f;
// สามารถใส่การหมุนเล็กน้อยขณะหลบได้
constexpr float AUTO_TURN_SPEED = 0.0f;
// ============================================================
// Automatic movement duration
// ============================================================
// หยุดก่อนหลบ
constexpr unsigned long STOP_BEFORE_AVOID_MS = 250;
// ระยะเวลาที่เลื่อนหลบซ้ายหรือขวา
constexpr unsigned long STRAFE_DURATION_MS = 650;
// ระยะเวลาถอยหลังเมื่อถูกปิดล้อม
constexpr unsigned long REVERSE_DURATION_MS = 850;
// หยุดหลังถอยก่อนประเมินเส้นทางใหม่
constexpr unsigned long STOP_AFTER_REVERSE_MS = 250;
// ============================================================
// Sonar settings
// ============================================================
// ระยะที่ถือว่าพบสิ่งกีดขวาง
constexpr float FRONT_STOP_DISTANCE_CM = 28.0f;
constexpr float SIDE_STOP_DISTANCE_CM = 22.0f;
// ระยะปล่อยสถานะ ใช้ Hysteresis
constexpr float FRONT_RELEASE_DISTANCE_CM = 34.0f;
constexpr float SIDE_RELEASE_DISTANCE_CM = 28.0f;
// จำนวนครั้งอ่าน ADC แล้วเฉลี่ย
constexpr int SONAR_SAMPLE_COUNT = 8;
// ระยะที่เซนเซอร์วัดได้
constexpr float SONAR_MIN_VALID_CM = 2.0f;
constexpr float SONAR_MAX_VALID_CM = 100.0f;

// ระยะเวลาระหว่างการอ่านเซนเซอร์
constexpr unsigned long SENSOR_INTERVAL_MS = 50;
// ============================================================
// Joystick settings
// ============================================================
constexpr int JOYSTICK_MAX = 512;
constexpr int JOYSTICK_DEADZONE = 60;
// ============================================================
// Safety
// ============================================================
constexpr unsigned long CONTROLLER_TIMEOUT_MS = 500;
unsigned long lastControllerUpdate = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastSerialPrintTime = 0;
// ============================================================
// Robot operating mode
// ============================================================

enum RobotMode {
  MODE_MANUAL,
  MODE_AUTOMATIC
};

RobotMode robotMode = MODE_AUTOMATIC;
// ============================================================
// Automatic state machine
// ============================================================
enum AutoState {
  AUTO_FORWARD,
  AUTO_STOP_BEFORE_LEFT,
  AUTO_STOP_BEFORE_RIGHT,
  AUTO_STRAFE_LEFT,
  AUTO_STRAFE_RIGHT,
  AUTO_STOP_BEFORE_REVERSE,
  AUTO_REVERSE,
  AUTO_STOP_AFTER_REVERSE,
  AUTO_EMERGENCY_STOP
};

AutoState autoState = AUTO_FORWARD;
unsigned long autoStateStartTime = 0;

// ============================================================

// Sonar data

// ============================================================

struct SonarData {

  int rawFront;

  int rawLeft;

  int rawRight;

  float frontCm;

  float leftCm;

  float rightCm;

  bool frontBlocked;

  bool leftBlocked;

  bool rightBlocked;
};

SonarData sonar = {

  0,

  0,

  0,

  100.0f,

  100.0f,

  100.0f,

  false,

  false,

  false

};

// ============================================================

// Controller

// ============================================================

ControllerPtr controller = nullptr;

bool motorEnabled = true;

bool previousSquareState = false;

bool previousCircleState = false;

bool previousTriangleState = false;

// ============================================================

// PWM compatibility (ESP32 Arduino Core 2.x / 3.x)

// ============================================================

uint8_t getPwmChannelForPin(uint8_t pwmPin)

{

  switch (pwmPin) {

    case MOTOR1_PWM_PIN:

      return MOTOR1_PWM_CHANNEL;

    case MOTOR2_PWM_PIN:

      return MOTOR2_PWM_CHANNEL;

    case MOTOR3_PWM_PIN:

      return MOTOR3_PWM_CHANNEL;

    default:

      return MOTOR1_PWM_CHANNEL;
  }
}

bool attachMotorPwm(uint8_t pwmPin, uint8_t pwmChannel)

{

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)

  return ledcAttach(

    pwmPin,

    PWM_FREQUENCY,

    PWM_RESOLUTION

  );

#else

  const double configuredFrequency = ledcSetup(

    pwmChannel,

    PWM_FREQUENCY,

    PWM_RESOLUTION

  );

  if (configuredFrequency <= 0.0) {

    return false;
  }

  ledcAttachPin(pwmPin, pwmChannel);

  return true;

#endif
}

void writeMotorPwm(uint8_t pwmPin, uint32_t pwmValue)

{

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)

  ledcWrite(pwmPin, pwmValue);

#else

  ledcWrite(getPwmChannelForPin(pwmPin), pwmValue);

#endif
}

// ============================================================

// Utility functions

// ============================================================

float clampFloat(float value, float minValue, float maxValue)

{

  if (value > maxValue) {

    return maxValue;
  }

  if (value < minValue) {

    return minValue;
  }

  return value;
}

const char* getModeName()

{

  return robotMode == MODE_AUTOMATIC

           ? "AUTOMATIC"

           : "MANUAL";
}

const char* getAutoStateName()

{

  switch (autoState) {

    case AUTO_FORWARD:

      return "FORWARD";

    case AUTO_STOP_BEFORE_LEFT:

      return "STOP_BEFORE_LEFT";

    case AUTO_STOP_BEFORE_RIGHT:

      return "STOP_BEFORE_RIGHT";

    case AUTO_STRAFE_LEFT:

      return "STRAFE_LEFT";

    case AUTO_STRAFE_RIGHT:

      return "STRAFE_RIGHT";

    case AUTO_STOP_BEFORE_REVERSE:

      return "STOP_BEFORE_REVERSE";

    case AUTO_REVERSE:

      return "REVERSE";

    case AUTO_STOP_AFTER_REVERSE:

      return "STOP_AFTER_REVERSE";

    case AUTO_EMERGENCY_STOP:

      return "EMERGENCY_STOP";

    default:

      return "UNKNOWN";
  }
}

// ============================================================

// Joystick processing

// ============================================================

float normalizeJoystick(int value)

{

  if (abs(value) <= JOYSTICK_DEADZONE) {

    return 0.0f;
  }

  float result;

  if (value > 0) {

    result =

      static_cast<float>(value - JOYSTICK_DEADZONE) /

      static_cast<float>(

        JOYSTICK_MAX - JOYSTICK_DEADZONE

      );

  } else {

    result =

      static_cast<float>(value + JOYSTICK_DEADZONE) /

      static_cast<float>(

        JOYSTICK_MAX - JOYSTICK_DEADZONE

      );
  }

  return clampFloat(result, -1.0f, 1.0f);
}

// ทำให้การขยับใกล้จุดศูนย์กลางนุ่มขึ้น

float applyExpo(float value)

{

  return value * fabsf(value);
}

// ============================================================

// Motor control

// ============================================================

void setMotor(

  uint8_t pwmPin,

  uint8_t dirPin,

  float speed,

  int directionCorrection)

{

  speed *= directionCorrection;

  speed = clampFloat(speed, -1.0f, 1.0f);

  if (!motorEnabled || fabsf(speed) < 0.01f) {

    writeMotorPwm(pwmPin, 0);

    return;
  }

  const bool forward = speed >= 0.0f;

  digitalWrite(

    dirPin,

    forward ? HIGH : LOW

  );

  int pwmValue =

    static_cast<int>(fabsf(speed) * PWM_MAX);

  if (pwmValue > 0 && pwmValue < MOTOR_MIN_PWM) {

    pwmValue = MOTOR_MIN_PWM;
  }

  pwmValue = constrain(pwmValue, 0, PWM_MAX);

  writeMotorPwm(pwmPin, pwmValue);
}

void stopRobot()

{

  writeMotorPwm(MOTOR1_PWM_PIN, 0);

  writeMotorPwm(MOTOR2_PWM_PIN, 0);

  writeMotorPwm(MOTOR3_PWM_PIN, 0);
}

// ============================================================

// Omni 3-wheel inverse kinematics

// ============================================================

void moveRobot(float vx, float vy, float wz)

{

  if (!motorEnabled) {

    stopRobot();

    return;
  }

  /*

   * Wheel 1 = 150 degrees

   * Wheel 2 = 30 degrees

   * Wheel 3 = 270 degrees

   *

   * W1 = -0.866Vx + 0.5Vy + L×Wz

   * W2 = 0.866Vx + 0.5Vy + L×Wz

   * W3 =     -1.0Vy + L×Wz

   */

  float wheel1 =

    (-0.8660254f * vx) +

    (0.5f * vy) +

    (ROBOT_RADIUS * wz);

  float wheel2 =

    (0.8660254f * vx) +

    (0.5f * vy) +

    (ROBOT_RADIUS * wz);

  float wheel3 =

    (-1.0f * vy) +

    (ROBOT_RADIUS * wz);

  // Normalize โดยรักษาสัดส่วนความเร็วทุกล้อ

  const float maximumWheel = max(

    fabsf(wheel1),

    max(

      fabsf(wheel2),

      fabsf(wheel3)

        )

  );

  if (maximumWheel > 1.0f) {

    wheel1 /= maximumWheel;

    wheel2 /= maximumWheel;

    wheel3 /= maximumWheel;
  }

  setMotor(

    MOTOR1_PWM_PIN,

    MOTOR1_DIR_PIN,

    wheel1,

    MOTOR1_DIRECTION

  );

  setMotor(

    MOTOR2_PWM_PIN,

    MOTOR2_DIR_PIN,

    wheel2,

    MOTOR2_DIRECTION

  );

  setMotor(

    MOTOR3_PWM_PIN,

    MOTOR3_DIR_PIN,

    wheel3,

    MOTOR3_DIRECTION

  );
}

// ============================================================

// ZX-SONAR1M functions

// ============================================================

int readAverageADC(uint8_t pin)

{

  uint32_t total = 0;

  for (int i = 0; i < SONAR_SAMPLE_COUNT; i++) {

    total += analogRead(pin);

    delayMicroseconds(500);
  }

  return static_cast<int>(

    total / SONAR_SAMPLE_COUNT

  );
}

float adcToDistanceCm(int adcValue)

{

  /*

   * ZX-SONAR1M:

   *

   * ADC 12 bit:

   * Distance ≈ ADC / 40

   *

   * 0  ≈ 0 cm

   * 400 ≈ 10 cm

   * 1000 ≈ 25 cm

   * 2000 ≈ 50 cm

   * 4000 ≈ 100 cm

   */

  float distance =

    static_cast<float>(adcValue) / 40.0f;

  distance = clampFloat(

    distance,

    0.0f,

    SONAR_MAX_VALID_CM

  );

  return distance;
}

bool updateBlockedState(

  float distanceCm,

  bool previousState,

  float detectDistance,

  float releaseDistance)

{

  /*

   * ถ้าอยู่ในสถานะไม่พบ:

   * ต้องใกล้กว่า detectDistance จึงถือว่าพบ

   *

   * ถ้าอยู่ในสถานะพบแล้ว:

   * ต้องไกลกว่า releaseDistance จึงปล่อยสถานะ

   */

  if (!previousState) {

    if (

      distanceCm >= SONAR_MIN_VALID_CM &&

      distanceCm <= detectDistance

    ) {

      return true;
    }

  } else {

    if (

      distanceCm <= 0.0f ||

      distanceCm >= releaseDistance

    ) {

      return false;
    }
  }

  return previousState;
}

void readSonarSensors()

{

  if (

    millis() - lastSensorReadTime <

    SENSOR_INTERVAL_MS

  ) {

    return;
  }

  lastSensorReadTime = millis();

  sonar.rawFront = readAverageADC(SONAR_FRONT_PIN);

  sonar.rawLeft = readAverageADC(SONAR_LEFT_PIN);

  sonar.rawRight = readAverageADC(SONAR_RIGHT_PIN);

  sonar.frontCm = adcToDistanceCm(sonar.rawFront);

  sonar.leftCm = adcToDistanceCm(sonar.rawLeft);

  sonar.rightCm = adcToDistanceCm(sonar.rawRight);

  sonar.frontBlocked = updateBlockedState(

    sonar.frontCm,

    sonar.frontBlocked,

    FRONT_STOP_DISTANCE_CM,

    FRONT_RELEASE_DISTANCE_CM

  );

  sonar.leftBlocked = updateBlockedState(

    sonar.leftCm,

    sonar.leftBlocked,

    SIDE_STOP_DISTANCE_CM,

    SIDE_RELEASE_DISTANCE_CM

  );

  sonar.rightBlocked = updateBlockedState(

    sonar.rightCm,

    sonar.rightBlocked,

    SIDE_STOP_DISTANCE_CM,

    SIDE_RELEASE_DISTANCE_CM

  );
}

// ============================================================

// Automatic state control

// ============================================================

void changeAutoState(AutoState newState)

{

  if (autoState == newState) {

    return;
  }

  autoState = newState;

  autoStateStartTime = millis();

  stopRobot();

  Serial.print("Auto state changed: ");

  Serial.println(getAutoStateName());
}

void evaluateObstacles()

{

  const bool front = sonar.frontBlocked;

  const bool left = sonar.leftBlocked;

  const bool right = sonar.rightBlocked;

  /*

   * ลำดับความสำคัญ:

   *

   * 1. เจอครบ 3 ด้าน -> ถอย

   * 2. ซ้ายและขวาปิด -> ถอย

   * 3. หน้าและซ้าย -> ไปขวา

   * 4. หน้าและขวา -> ไปซ้าย

   * 5. หน้าอย่างเดียว -> ไปซ้าย

   * 6. ซ้ายอย่างเดียว -> ไปขวา

   * 7. ขวาอย่างเดียว -> ไปซ้าย

   * 8. ไม่พบ -> เดินหน้า

   */

  if (front && left && right) {

    changeAutoState(AUTO_STOP_BEFORE_REVERSE);

    return;
  }

  if (left && right) {

    changeAutoState(AUTO_STOP_BEFORE_REVERSE);

    return;
  }

  if (front && left) {

    changeAutoState(AUTO_STOP_BEFORE_RIGHT);

    return;
  }

  if (front && right) {

    changeAutoState(AUTO_STOP_BEFORE_LEFT);

    return;
  }

  if (front) {

    changeAutoState(AUTO_STOP_BEFORE_LEFT);

    return;
  }

  if (left) {

    changeAutoState(AUTO_STOP_BEFORE_RIGHT);

    return;
  }

  if (right) {

    changeAutoState(AUTO_STOP_BEFORE_LEFT);

    return;
  }

  changeAutoState(AUTO_FORWARD);
}

// ============================================================

// Automatic mode

// ============================================================

void processAutomaticMode()

{

  const unsigned long stateElapsed =

    millis() - autoStateStartTime;

  /*

   * หากกำลังเดินหน้า ให้ตรวจสิ่งกีดขวางตลอดเวลา

   */

  if (autoState == AUTO_FORWARD) {

    evaluateObstacles();

    if (autoState == AUTO_FORWARD) {

      moveRobot(

        AUTO_FORWARD_SPEED,

        0.0f,

        0.0f

      );
    }

    return;
  }

  switch (autoState) {

      // ----------------------------------------------------

      // หยุดก่อนเลื่อนไปซ้าย

      // ----------------------------------------------------

    case AUTO_STOP_BEFORE_LEFT:

      stopRobot();

      /*

       * ระหว่างหยุด ถ้าพบว่าถูกปิดครบทุกด้าน

       * ให้เปลี่ยนไปถอยหลังทันที

       */

      if (

        sonar.frontBlocked &&

        sonar.leftBlocked &&

        sonar.rightBlocked

      ) {

        changeAutoState(

          AUTO_STOP_BEFORE_REVERSE

        );

        return;
      }

      if (

        stateElapsed >=

        STOP_BEFORE_AVOID_MS

      ) {

        changeAutoState(AUTO_STRAFE_LEFT);
      }

      break;

      // ----------------------------------------------------

      // หยุดก่อนเลื่อนไปขวา

      // ----------------------------------------------------

    case AUTO_STOP_BEFORE_RIGHT:

      stopRobot();

      if (

        sonar.frontBlocked &&

        sonar.leftBlocked &&

        sonar.rightBlocked

      ) {

        changeAutoState(

          AUTO_STOP_BEFORE_REVERSE

        );

        return;
      }

      if (

        stateElapsed >=

        STOP_BEFORE_AVOID_MS

      ) {

        changeAutoState(AUTO_STRAFE_RIGHT);
      }

      break;

      // ----------------------------------------------------

      // เลื่อนไปซ้าย

      //

      // +Vy = ซ้าย

      // ----------------------------------------------------

    case AUTO_STRAFE_LEFT:

      /*

       * หากระหว่างเลื่อนไปซ้าย

       * เซนเซอร์ซ้ายพบสิ่งกีดขวาง

       * ให้ถอยหลัง

       */

      if (

        sonar.leftBlocked &&

        sonar.rightBlocked

      ) {

        changeAutoState(

          AUTO_STOP_BEFORE_REVERSE

        );

        return;
      }

      moveRobot(

        0.0f,

        AUTO_STRAFE_SPEED,

        AUTO_TURN_SPEED

      );

      if (stateElapsed >= STRAFE_DURATION_MS) {

        changeAutoState(AUTO_FORWARD);
      }

      break;

      // ----------------------------------------------------

      // เลื่อนไปขวา

      //

      // -Vy = ขวา

      // ----------------------------------------------------

    case AUTO_STRAFE_RIGHT:

      if (

        sonar.leftBlocked &&

        sonar.rightBlocked

      ) {

        changeAutoState(

          AUTO_STOP_BEFORE_REVERSE

        );

        return;
      }

      moveRobot(

        0.0f,

        -AUTO_STRAFE_SPEED,

        -AUTO_TURN_SPEED

      );

      if (stateElapsed >= STRAFE_DURATION_MS) {

        changeAutoState(AUTO_FORWARD);
      }

      break;

      // ----------------------------------------------------

      // หยุดก่อนถอยหลัง

      // ----------------------------------------------------

    case AUTO_STOP_BEFORE_REVERSE:

      stopRobot();

      if (

        stateElapsed >=

        STOP_BEFORE_AVOID_MS

      ) {

        changeAutoState(AUTO_REVERSE);
      }

      break;

      // ----------------------------------------------------

      // ถอยหลัง

      // ----------------------------------------------------

    case AUTO_REVERSE:

      moveRobot(

        -AUTO_REVERSE_SPEED,

        0.0f,

        0.0f

      );

      if (

        stateElapsed >=

        REVERSE_DURATION_MS

      ) {

        changeAutoState(

          AUTO_STOP_AFTER_REVERSE

        );
      }

      break;

      // ----------------------------------------------------

      // หยุดหลังถอย

      // ----------------------------------------------------

    case AUTO_STOP_AFTER_REVERSE:

      stopRobot();

      if (

        stateElapsed >=

        STOP_AFTER_REVERSE_MS

      ) {

        /*

         * หลังถอยเสร็จ ตรวจใหม่ว่าควรไปทางไหน

         */

        evaluateObstacles();
      }

      break;

      // ----------------------------------------------------

      // Emergency stop

      // ----------------------------------------------------

    case AUTO_EMERGENCY_STOP:

      stopRobot();

      break;

    default:

      stopRobot();

      changeAutoState(AUTO_FORWARD);

      break;
  }
}

// ============================================================

// Manual mode

// ============================================================

void processManualGamepad(ControllerPtr ctl)

{

  float leftX =

    normalizeJoystick(ctl->axisX());

  float leftY =

    normalizeJoystick(ctl->axisY());

  float rightX =

    normalizeJoystick(ctl->axisRX());

  leftX = applyExpo(leftX);

  leftY = applyExpo(leftY);

  rightX = applyExpo(rightX);

  // ดันแกนซ้ายขึ้น = axisY ติดลบ

  float vx = -leftY * MAX_MANUAL_VX;

  // ดันแกนซ้ายซ้าย = axisX ติดลบ

  float vy = -leftX * MAX_MANUAL_VY;

  float wz = -rightX * MAX_MANUAL_WZ;

  // L1/R1 หมุนด้วยความเร็วคงที่

  if (ctl->l1()) {

    wz = 0.8f;
  }

  if (ctl->r1()) {

    wz = -0.8f;
  }

  // Trigger สำหรับปรับระดับความเร็ว

  const float l2 =

    static_cast<float>(ctl->brake()) /

    1023.0f;

  const float r2 =

    static_cast<float>(ctl->throttle()) /

    1023.0f;

  if (l2 > 0.05f) {

    manualSpeedScale =

      MAX_SPEED_SCALE - (l2 * 0.8f);
  }

  if (r2 > 0.05f) {

    manualSpeedScale =

      MIN_SPEED_SCALE + (r2 * 0.8f);
  }

  manualSpeedScale = clampFloat(

    manualSpeedScale,

    MIN_SPEED_SCALE,

    MAX_SPEED_SCALE

  );

  vx *= manualSpeedScale;

  vy *= manualSpeedScale;

  wz *= manualSpeedScale;

  /*

   * Manual safety:

   * แม้อยู่โหมด Manual ถ้ามีสิ่งกีดขวางด้านหน้า

   * และผู้ใช้กำลังสั่งเดินหน้า ให้หยุดการเดินหน้า

   *

   * แต่ยังสามารถถอยหลังหรือเลื่อนข้างได้

   */

  if (

    sonar.frontBlocked &&

    vx > 0.0f

  ) {

    vx = 0.0f;
  }

  if (

    sonar.leftBlocked &&

    vy > 0.0f

  ) {

    vy = 0.0f;
  }

  if (

    sonar.rightBlocked &&

    vy < 0.0f

  ) {

    vy = 0.0f;
  }

  moveRobot(vx, vy, wz);
}

// ============================================================

// Controller mode buttons

// ============================================================

void processControllerButtons(ControllerPtr ctl)

{

  /*

   * Bluepad32 mapping:

   *

   * ctl->a() = Cross

   * ctl->b() = Circle

   * ctl->x() = Square

   * ctl->y() = Triangle

   */

  // Cross = Emergency stop

  if (ctl->a()) {

    stopRobot();

    if (robotMode == MODE_AUTOMATIC) {

      autoState = AUTO_EMERGENCY_STOP;
    }

    return;
  }

  // Square = เปลี่ยน Auto/Manual

  const bool squareState = ctl->x();

  if (squareState && !previousSquareState) {

    stopRobot();

    if (robotMode == MODE_MANUAL) {

      robotMode = MODE_AUTOMATIC;

      changeAutoState(AUTO_FORWARD);

      ctl->setColorLED(0, 255, 0);

      Serial.println(

        "Changed to AUTOMATIC mode"

      );

    } else {

      robotMode = MODE_MANUAL;

      ctl->setColorLED(0, 100, 255);

      Serial.println(

        "Changed to MANUAL mode"

      );
    }

    ctl->playDualRumble(

      0,

      200,

      0x40,

      0x70

    );
  }

  previousSquareState = squareState;

  // Circle = Enable/Disable motor

  const bool circleState = ctl->b();

  if (circleState && !previousCircleState) {

    motorEnabled = !motorEnabled;

    stopRobot();

    if (motorEnabled) {

      Serial.println("Motor enabled");

      if (robotMode == MODE_AUTOMATIC) {

        ctl->setColorLED(0, 255, 0);

        changeAutoState(AUTO_FORWARD);

      } else {

        ctl->setColorLED(0, 100, 255);
      }

    } else {

      Serial.println("Motor disabled");

      ctl->setColorLED(255, 0, 0);
    }

    ctl->playDualRumble(

      0,

      150,

      0x30,

      0x60

    );
  }

  previousCircleState = circleState;

  // Triangle = ออกจาก Emergency Stop

  const bool triangleState = ctl->y();

  if (triangleState && !previousTriangleState) {

    if (

      robotMode == MODE_AUTOMATIC &&

      motorEnabled

    ) {

      changeAutoState(AUTO_FORWARD);

      Serial.println(

        "Automatic movement resumed"

      );
    }
  }

  previousTriangleState = triangleState;
}

// ============================================================

// Controller callbacks

// ============================================================

void onControllerConnected(ControllerPtr ctl)

{

  if (controller != nullptr) {

    return;
  }

  controller = ctl;

  Serial.println();

  Serial.println("PS4 controller connected");

  Serial.printf(

    "Model: %s\n",

    ctl->getModelName().c_str()

  );

  lastControllerUpdate = millis();

  if (robotMode == MODE_AUTOMATIC) {

    ctl->setColorLED(0, 255, 0);

  } else {

    ctl->setColorLED(0, 100, 255);
  }

  ctl->playDualRumble(

    0,

    250,

    0x40,

    0x80

  );
}

void onControllerDisconnected(ControllerPtr ctl)

{

  if (controller != ctl) {

    return;
  }

  controller = nullptr;

  Serial.println("PS4 controller disconnected");

  /*

   * ในโหมด Manual ต้องหยุดทันที

   *

   * ในตัวอย่างนี้เลือกให้โหมด Automatic

   * หยุดด้วยเพื่อความปลอดภัย

   */

  stopRobot();

  motorEnabled = false;
}

// ============================================================

// Debug output

// ============================================================

void printRobotStatus()

{

  if (

    millis() - lastSerialPrintTime <

    250

  ) {

    return;
  }

  lastSerialPrintTime = millis();

  Serial.printf(

    "Mode:%s State:%s | "

    "Front:%5.1fcm [%d] %d | "

    "Left:%5.1fcm [%d] %d | "

    "Right:%5.1fcm [%d] %d | "

    "Motor:%d\n",

    getModeName(),

    getAutoStateName(),

    sonar.frontCm,

    sonar.rawFront,

    sonar.frontBlocked,

    sonar.leftCm,

    sonar.rawLeft,

    sonar.leftBlocked,

    sonar.rightCm,

    sonar.rawRight,

    sonar.rightBlocked,

    motorEnabled

  );
}

// ============================================================

// Setup

// ============================================================

void setup()

{

  Serial.begin(115200);

  delay(1000);

  // Motor direction pins

  pinMode(MOTOR1_DIR_PIN, OUTPUT);

  pinMode(MOTOR2_DIR_PIN, OUTPUT);

  pinMode(MOTOR3_DIR_PIN, OUTPUT);

  // Sonar pins

  pinMode(SONAR_FRONT_PIN, INPUT);

  pinMode(SONAR_LEFT_PIN, INPUT);

  pinMode(SONAR_RIGHT_PIN, INPUT);

  // ADC 12-bit: 0-4095

  analogReadResolution(12);

  /*

   * ตั้งช่วงแรงดัน ADC

   * ZX-SONAR1M ให้เอาต์พุตสูงสุดประมาณ 3.3V

   */

  analogSetPinAttenuation(

    SONAR_FRONT_PIN,

    ADC_11db

  );

  analogSetPinAttenuation(

    SONAR_LEFT_PIN,

    ADC_11db

  );

  analogSetPinAttenuation(

    SONAR_RIGHT_PIN,

    ADC_11db

  );

  // PWM

  const bool pwm1Ready = attachMotorPwm(

    MOTOR1_PWM_PIN,

    MOTOR1_PWM_CHANNEL

  );

  const bool pwm2Ready = attachMotorPwm(

    MOTOR2_PWM_PIN,

    MOTOR2_PWM_CHANNEL

  );

  const bool pwm3Ready = attachMotorPwm(

    MOTOR3_PWM_PIN,

    MOTOR3_PWM_CHANNEL

  );

  if (

    !pwm1Ready ||

    !pwm2Ready ||

    !pwm3Ready

  ) {

    Serial.println(

      "ERROR: PWM configuration failed"

    );
  }

  stopRobot();

  // Bluepad32

  BP32.setup(

    &onControllerConnected,

    &onControllerDisconnected

  );

  BP32.enableVirtualDevice(false);

  /*

   * ใช้เพียงครั้งเดียวเมื่อต้องการลบข้อมูลจอยเดิม

   */

  // BP32.forgetBluetoothKeys();

  robotMode = MODE_AUTOMATIC;

  autoState = AUTO_FORWARD;

  autoStateStartTime = millis();

  /*

   * ถ้าต้องการให้เริ่มทำงานอัตโนมัติทันที

   * โดยไม่รอจอย ให้ตั้งเป็น true

   */

  motorEnabled = true;

  Serial.println();

  Serial.println("======================================");

  Serial.println("3-Wheel Omni Robot Ready");

  Serial.println("ZX-SONAR1M: Front GPIO36");

  Serial.println("ZX-SONAR1M: Left GPIO34");

  Serial.println("ZX-SONAR1M: Right GPIO39");

  Serial.println("Default mode: AUTOMATIC");

  Serial.println("Square: Auto/Manual");

  Serial.println("Cross: Emergency stop");

  Serial.println("Circle: Motor enable/disable");

  Serial.println("Triangle: Resume automatic mode");

  Serial.println("======================================");
}

// ============================================================

// Main loop

// ============================================================

void loop()

{

  // อ่านเซนเซอร์ทั้ง 3 ตัว

  readSonarSensors();

  // อัปเดตข้อมูล Bluetooth

  const bool dataUpdated = BP32.update();

  if (

    dataUpdated &&

    controller != nullptr &&

    controller->isConnected()

  ) {

    lastControllerUpdate = millis();

    processControllerButtons(controller);
  }

  if (!motorEnabled) {

    stopRobot();

    printRobotStatus();

    delay(2);

    return;
  }

  // --------------------------------------------------------

  // Automatic mode

  // --------------------------------------------------------

  if (robotMode == MODE_AUTOMATIC) {

    /*

     * Auto สามารถทำงานได้แม้ยังไม่เชื่อมต่อจอย

     * แต่กดปุ่มจอยเพื่อหยุดหรือเปลี่ยนโหมดได้

     */

    processAutomaticMode();

  }

  // --------------------------------------------------------

  // Manual mode

  // --------------------------------------------------------

  else {

    if (

      controller != nullptr &&

      controller->isConnected() &&

      millis() - lastControllerUpdate <=

        CONTROLLER_TIMEOUT_MS

    ) {

      processManualGamepad(controller);

    } else {

      stopRobot();
    }
  }

  printRobotStatus();

  delay(2);
}