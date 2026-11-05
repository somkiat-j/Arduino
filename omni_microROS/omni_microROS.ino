#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <tf2_msgs/msg/tf_message.h>
#include <rosidl_runtime_c/string_functions.h>

#if __has_include(<rclc_parameter/rclc_parameter.h>)
#include <rclc_parameter/rclc_parameter.h>
#define HAS_RCLC_PARAMETER 1
#else
#define HAS_RCLC_PARAMETER 0
#endif

// ============================================================
// 3-Wheel Omni Robot
// ESP32 + micro-ROS + teleop_twist_keyboard (/cmd_vel)
// ============================================================

// Wi-Fi and micro-ROS agent configuration.
// AGENT_IP must be the IPv4 address of the ROS 2 host, not 127.0.0.1.
char WIFI_SSID[] = "EMB5325";
char WIFI_PASSWORD[] = "cdti12345";
char AGENT_IP[] = "192.168.1.107";
constexpr uint16_t AGENT_PORT = 8888;

// ============================================================
// Motor pins
// ============================================================

// Motor 1: Front-left
constexpr uint8_t MOTOR1_PWM_PIN = 27;
constexpr uint8_t MOTOR1_DIR_PIN = 14;

// Motor 2: Front-right
constexpr uint8_t MOTOR2_PWM_PIN = 26;
constexpr uint8_t MOTOR2_DIR_PIN = 25;

// Motor 3: Rear
constexpr uint8_t MOTOR3_PWM_PIN = 13;
constexpr uint8_t MOTOR3_DIR_PIN = 12;

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
// Set to -1 if wheel direction is reversed
// ============================================================

constexpr int MOTOR1_DIRECTION = 1;
constexpr int MOTOR2_DIRECTION = 1;
constexpr int MOTOR3_DIRECTION = 1;

// ============================================================
// Robot dimensions
// ============================================================

// Distance from robot center to wheel (meters)
constexpr float ROBOT_RADIUS = 0.24f;

// Maximum tangential wheel speed at 100% PWM (m/s).
// Measure and tune this value for the actual motor, gearbox and battery.
constexpr float MAX_WHEEL_LINEAR_SPEED = 1.00f;

// ============================================================
// Encoder and odometry configuration
// ============================================================

// Encoder pins from user wiring:
// M1: A=32, B=33 | M2: A=34, B=35 | M3: A=2, B=15
constexpr uint8_t ENC1_A_PIN = 32;
constexpr uint8_t ENC1_B_PIN = 33;
constexpr uint8_t ENC2_A_PIN = 34;
constexpr uint8_t ENC2_B_PIN = 35;
constexpr uint8_t ENC3_A_PIN = 2;
constexpr uint8_t ENC3_B_PIN = 15;

// Encoder sign correction. Set to -1 if wheel direction is inverted.
constexpr int ENCODER1_DIRECTION = 1;
constexpr int ENCODER2_DIRECTION = 1;
constexpr int ENCODER3_DIRECTION = 1;

// Tune these values to your hardware.
// Measure each wheel independently; manufacturing and tyre wear can make the
// effective rolling radius slightly different between wheels.
constexpr float DEFAULT_ODOM_WHEEL1_RADIUS_M = 0.050f;
constexpr float DEFAULT_ODOM_WHEEL2_RADIUS_M = 0.050f;
constexpr float DEFAULT_ODOM_WHEEL3_RADIUS_M = 0.050f;
constexpr float DEFAULT_ENCODER_TICKS_PER_REV = 2048.0f;
constexpr unsigned long ODOM_PUBLISH_INTERVAL_MS = 20;
constexpr unsigned long PID_CONTROL_INTERVAL_MS = 10;

constexpr float DEFAULT_WHEEL_PID_KP = 0.45f;
constexpr float DEFAULT_WHEEL_PID_KI = 0.15f;
constexpr float DEFAULT_WHEEL_PID_KD = 0.02f;
constexpr float WHEEL_PID_INTEGRAL_LIMIT = 0.60f;

constexpr char ODOM_FRAME_ID[] = "odom";
constexpr char BASE_LINK_FRAME_ID[] = "base_link";

// The precompiled micro_ros_arduino library used by this project supports a
// maximum of 5 subscriptions. Keep optional runtime-configuration interfaces
// disabled so the essential Nav2 interfaces always fit. Enabling these
// requires rebuilding micro_ros_arduino with larger RMW_UXRCE_MAX_* limits.
constexpr bool ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES = false;
constexpr bool ENABLE_ROS_PARAMETER_SERVER = false;

// ============================================================
// cmd_vel constraints
// ============================================================

constexpr float MAX_CMD_VX = 0.80f;
constexpr float MAX_CMD_VY = 0.80f;
constexpr float MAX_CMD_WZ = 2.20f;

constexpr unsigned long CMD_VEL_TIMEOUT_MS = 400;

// ============================================================
// GY-25 IMU (UART) and heading-hold controller
// ============================================================

// GY-25 TX -> ESP32 GPIO16 (RX2), GY-25 RX -> ESP32 GPIO17 (TX2).
// The sensor and ESP32 must share GND.
constexpr uint8_t GY25_RX = 16;
constexpr uint8_t GY25_TX = 17;
constexpr uint32_t BAUD_RATES[] = {9600, 115200};
constexpr size_t FRAME_SIZE = 8;

constexpr unsigned long GY25_REQUEST_INTERVAL_MS = 100;
constexpr unsigned long GY25_BAUD_TRY_MS = 4000;
constexpr unsigned long GY25_WARNING_AFTER_MS = 12000;
constexpr unsigned long GY25_TIMEOUT_MS = 300;

// Change to -1 if the reported yaw increases when the robot turns clockwise.
constexpr float GY25_YAW_DIRECTION = 1.0f;

// Heading hold is active while translating and angular.z is near zero.
constexpr float DEFAULT_HEADING_HOLD_KP = 2.2f;
constexpr float DEFAULT_HEADING_HOLD_KD = 0.08f;
constexpr float HEADING_HOLD_MAX_WZ = 1.2f;
constexpr float ROTATION_COMMAND_DEADBAND = 0.05f;
constexpr float TRANSLATION_COMMAND_DEADBAND = 0.02f;

constexpr size_t MAX_YAW_RATE_FILTER_WINDOW = 15;
constexpr size_t DEFAULT_YAW_RATE_FILTER_WINDOW = 5;
constexpr unsigned long IMU_PUBLISH_INTERVAL_MS = 50;
constexpr float DEFAULT_IMU_LPF_ALPHA = 0.25f;
constexpr float DEFAULT_IMU_ORIENTATION_COVARIANCE = 0.03f;
constexpr float DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE = 0.06f;
constexpr float DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE = 0.10f;
constexpr char DEFAULT_IMU_FRAME_ID[] = "imu_link";
constexpr size_t IMU_FRAME_ID_MAX_LEN = 63;

constexpr char PARAM_HEADING_KP[] = "heading_kp";
constexpr char PARAM_HEADING_KD[] = "heading_kd";
constexpr char PARAM_YAW_RATE_WINDOW[] = "yaw_rate_window";
constexpr char PARAM_IMU_LPF_ALPHA[] = "imu_lpf_alpha";
constexpr char PARAM_ENABLE_TIMESTAMP_SYNC[] = "enable_timestamp_sync";
constexpr char PARAM_IMU_COV_PRESET_MODE[] = "imu_cov_preset_mode";
constexpr char PARAM_ODOM_WHEEL_RADIUS_M[] = "odom_wheel_radius_m";
constexpr char PARAM_ENCODER_TICKS_PER_REV[] = "encoder_ticks_per_rev";
constexpr char PARAM_IMU_ORI_COV_PREFIX[] = "imu_orientation_cov";
constexpr char PARAM_IMU_ANG_VEL_COV_PREFIX[] = "imu_angular_velocity_cov";
constexpr char PARAM_IMU_LIN_ACC_COV_PREFIX[] = "imu_linear_acceleration_cov";

enum class ImuCovPresetMode : int32_t {
    LOW_NOISE = 0,
    MEDIUM = 1,
    AGGRESSIVE = 2
};

HardwareSerial GY25(2);

uint8_t frame[FRAME_SIZE];
size_t frameIndex = 0;
uint32_t lastRequest = 0;
uint32_t lastFrame = 0;
bool warningPrinted = false;
uint8_t baudIndex = 0;
uint32_t baudStarted = 0;
uint32_t receivedBytes = 0;
bool baudLocked = false;

float gy25YawDeg = 0.0f;
float gy25PitchDeg = 0.0f;
float gy25RollDeg = 0.0f;
float gy25YawRawDeg = 0.0f;
float gy25PitchRawDeg = 0.0f;
float gy25RollRawDeg = 0.0f;
float previousGy25YawDeg = 0.0f;
float headingTargetDeg = 0.0f;
float gy25YawRateDegPerSecond = 0.0f;
float yawRateSamples[MAX_YAW_RATE_FILTER_WINDOW] = {0.0f};
size_t yawRateSampleIndex = 0;
size_t yawRateSampleCount = 0;
size_t yawRateFilterWindow = DEFAULT_YAW_RATE_FILTER_WINDOW;
float headingHoldKp = DEFAULT_HEADING_HOLD_KP;
float headingHoldKd = DEFAULT_HEADING_HOLD_KD;
float imuLpfAlpha = DEFAULT_IMU_LPF_ALPHA;
bool enableTimestampSync = true;
int32_t imuCovPresetMode = static_cast<int32_t>(ImuCovPresetMode::MEDIUM);
float odomWheelRadiusM[3] = {
    DEFAULT_ODOM_WHEEL1_RADIUS_M,
    DEFAULT_ODOM_WHEEL2_RADIUS_M,
    DEFAULT_ODOM_WHEEL3_RADIUS_M
};
float encoderTicksPerRev = DEFAULT_ENCODER_TICKS_PER_REV;
float wheelPidKp = DEFAULT_WHEEL_PID_KP;
float wheelPidKi = DEFAULT_WHEEL_PID_KI;
float wheelPidKd = DEFAULT_WHEEL_PID_KD;
char imuFrameId[IMU_FRAME_ID_MAX_LEN + 1] = "imu_link";
float imuOrientationCovariance[9] = {
    DEFAULT_IMU_ORIENTATION_COVARIANCE, 0.0f, 0.0f,
    0.0f, DEFAULT_IMU_ORIENTATION_COVARIANCE, 0.0f,
    0.0f, 0.0f, DEFAULT_IMU_ORIENTATION_COVARIANCE
};
float imuAngularVelocityCovariance[9] = {
    DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE, 0.0f, 0.0f,
    0.0f, DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE, 0.0f,
    0.0f, 0.0f, DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE
};
float imuLinearAccelerationCovariance[9] = {
    DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE, 0.0f, 0.0f,
    0.0f, DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE, 0.0f,
    0.0f, 0.0f, DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE
};
bool gy25ReceivedOnce = false;
bool headingTargetValid = false;
bool rotationWasCommanded = false;

unsigned long lastCmdVelUpdate = 0;
unsigned long lastSerialPrintTime = 0;
unsigned long lastImuPublishTime = 0;
unsigned long lastOdomPublishTime = 0;
unsigned long lastPidControlTime = 0;

float cmdVelVx = 0.0f;
float cmdVelVy = 0.0f;
float cmdVelWz = 0.0f;
bool cmdVelReceivedOnce = false;

// Encoder runtime state (quadrature x4)
volatile int32_t encoder1Ticks = 0;
volatile int32_t encoder2Ticks = 0;
volatile int32_t encoder3Ticks = 0;

volatile uint8_t encoder1PrevState = 0;
volatile uint8_t encoder2PrevState = 0;
volatile uint8_t encoder3PrevState = 0;

int32_t lastEncoder1Ticks = 0;
int32_t lastEncoder2Ticks = 0;
int32_t lastEncoder3Ticks = 0;
int32_t lastPidEncoder1Ticks = 0;
int32_t lastPidEncoder2Ticks = 0;
int32_t lastPidEncoder3Ticks = 0;

float odomX = 0.0f;
float odomY = 0.0f;
float odomYaw = 0.0f;

float targetWheelMps[3] = {0.0f, 0.0f, 0.0f};
float measuredWheelMps[3] = {0.0f, 0.0f, 0.0f};
float wheelPidIntegral[3] = {0.0f, 0.0f, 0.0f};
float wheelPidPrevError[3] = {0.0f, 0.0f, 0.0f};

// ============================================================
// micro-ROS entities
// ============================================================

rcl_allocator_t rosAllocator;
rclc_support_t rosSupport;
rcl_node_t rosNode = rcl_get_zero_initialized_node();
rcl_subscription_t cmdVelSubscriber = rcl_get_zero_initialized_subscription();
rcl_subscription_t resetHeadingSubscriber = rcl_get_zero_initialized_subscription();
rcl_subscription_t setFrameIdSubscriber = rcl_get_zero_initialized_subscription();
rcl_subscription_t resetImuCovSubscriber = rcl_get_zero_initialized_subscription();
rcl_subscription_t dumpParamsSubscriber = rcl_get_zero_initialized_subscription();
rcl_subscription_t resetOdomSubscriber = rcl_get_zero_initialized_subscription();
rcl_publisher_t imuEulerPublisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t odomPublisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t tfPublisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t wheelTicksPublisher = rcl_get_zero_initialized_publisher();
rcl_publisher_t wheelVelPublisher = rcl_get_zero_initialized_publisher();
rclc_executor_t rosExecutor = rclc_executor_get_zero_initialized_executor();
geometry_msgs__msg__Twist cmdVelMsg;
std_msgs__msg__Bool resetHeadingMsg;
std_msgs__msg__String setFrameIdMsg;
std_msgs__msg__Bool resetImuCovMsg;
std_msgs__msg__Bool dumpParamsMsg;
std_msgs__msg__Bool resetOdomMsg;
sensor_msgs__msg__Imu imuMsg;
nav_msgs__msg__Odometry odomMsg;
tf2_msgs__msg__TFMessage tfMsg;
geometry_msgs__msg__Vector3 wheelTicksMsg;
geometry_msgs__msg__Vector3 wheelVelMsg;

#if HAS_RCLC_PARAMETER
rclc_parameter_server_t parameterServer;
#endif
bool parameterServerReady = false;
bool frameIdMsgInitialized = false;

bool boolMsgsInitialized = false;

bool microRosReady = false;
bool odomTfMsgsInitialized = false;

enum class ConnectionState : uint8_t {
    WAITING_FOR_AGENT,
    AGENT_AVAILABLE,
    CONNECTED,
    DISCONNECTED
};

ConnectionState connectionState = ConnectionState::WAITING_FOR_AGENT;
unsigned long lastAgentPingTime = 0;
constexpr unsigned long AGENT_SEARCH_INTERVAL_MS = 500;
constexpr unsigned long AGENT_HEALTH_CHECK_INTERVAL_MS = 2000;
constexpr uint8_t AGENT_MAX_CONSECUTIVE_FAILURES = 3;
uint8_t consecutiveAgentPingFailures = 0;

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
    (void)pwmChannel;
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
// Utility
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

bool isCmdVelActive()
{
    if (!cmdVelReceivedOnce) {
        return false;
    }

    return (millis() - lastCmdVelUpdate) <= CMD_VEL_TIMEOUT_MS;
}

float wrapAngleDegrees(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

bool isGy25Active()
{
    return gy25ReceivedOnce &&
        (millis() - lastFrame <= GY25_TIMEOUT_MS);
}

float lowPassFilter(float previousValue, float currentValue, float alpha)
{
    return previousValue + (alpha * (currentValue - previousValue));
}

float lowPassFilterAngleDegrees(float previousAngle, float currentAngle, float alpha)
{
    const float wrappedDelta = wrapAngleDegrees(currentAngle - previousAngle);
    return wrapAngleDegrees(previousAngle + (alpha * wrappedDelta));
}

float clampUnit(float value)
{
    return clampFloat(value, 0.0f, 1.0f);
}

// ============================================================
// Encoder helpers
// ============================================================

uint8_t readEncoderState(uint8_t pinA, uint8_t pinB)
{
    const uint8_t a = static_cast<uint8_t>(digitalRead(pinA));
    const uint8_t b = static_cast<uint8_t>(digitalRead(pinB));
    return static_cast<uint8_t>((a << 1) | b);
}

void IRAM_ATTR updateQuadrature(
    volatile int32_t* ticks,
    volatile uint8_t* prevState,
    uint8_t currentState)
{
    static const int8_t transitionTable[16] = {
        0, -1,  1,  0,
        1,  0,  0, -1,
       -1,  0,  0,  1,
        0,  1, -1,  0
    };

    const uint8_t index = static_cast<uint8_t>(((*prevState) << 2) | currentState);
    *ticks += transitionTable[index];
    *prevState = currentState;
}

void IRAM_ATTR encoder1Isr()
{
    const uint8_t current = readEncoderState(ENC1_A_PIN, ENC1_B_PIN);
    updateQuadrature(&encoder1Ticks, &encoder1PrevState, current);
}

void IRAM_ATTR encoder2Isr()
{
    const uint8_t current = readEncoderState(ENC2_A_PIN, ENC2_B_PIN);
    updateQuadrature(&encoder2Ticks, &encoder2PrevState, current);
}

void IRAM_ATTR encoder3Isr()
{
    const uint8_t current = readEncoderState(ENC3_A_PIN, ENC3_B_PIN);
    updateQuadrature(&encoder3Ticks, &encoder3PrevState, current);
}

void initEncoders()
{
    pinMode(ENC1_A_PIN, INPUT_PULLUP);
    pinMode(ENC1_B_PIN, INPUT_PULLUP);

    // GPIO34/GPIO35 are input-only; use external pull-ups if required.
    pinMode(ENC2_A_PIN, INPUT);
    pinMode(ENC2_B_PIN, INPUT);

    pinMode(ENC3_A_PIN, INPUT_PULLUP);
    pinMode(ENC3_B_PIN, INPUT_PULLUP);

    encoder1PrevState = readEncoderState(ENC1_A_PIN, ENC1_B_PIN);
    encoder2PrevState = readEncoderState(ENC2_A_PIN, ENC2_B_PIN);
    encoder3PrevState = readEncoderState(ENC3_A_PIN, ENC3_B_PIN);

    attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), encoder1Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B_PIN), encoder1Isr, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), encoder2Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B_PIN), encoder2Isr, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENC3_A_PIN), encoder3Isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC3_B_PIN), encoder3Isr, CHANGE);
}

void getRosTimeFromSystem(int32_t* secOut, uint32_t* nsecOut)
{
    const unsigned long nowMs = millis();
    int64_t stampMillis = static_cast<int64_t>(nowMs);

    if (enableTimestampSync) {
        const int64_t epochMillis = rmw_uros_epoch_millis();
        if (epochMillis > 0) {
            stampMillis = epochMillis;
        }
    }

    *secOut = static_cast<int32_t>(stampMillis / 1000);
    *nsecOut = static_cast<uint32_t>((stampMillis % 1000) * 1000000ULL);
}

bool setImuFrameId(const char* newFrameId)
{
    if (newFrameId == nullptr) {
        return false;
    }

    const size_t len = strlen(newFrameId);
    if (len == 0 || len > IMU_FRAME_ID_MAX_LEN) {
        return false;
    }

    strncpy(imuFrameId, newFrameId, IMU_FRAME_ID_MAX_LEN);
    imuFrameId[IMU_FRAME_ID_MAX_LEN] = '\0';

    return rosidl_runtime_c__String__assign(
        &imuMsg.header.frame_id,
        imuFrameId
    );
}

bool buildCovarianceParamName(
    char* output,
    size_t outputSize,
    const char* prefix,
    size_t index)
{
    const size_t row = index / 3;
    const size_t col = index % 3;

    return snprintf(
        output,
        outputSize,
        "%s_%u%u",
        prefix,
        static_cast<unsigned int>(row),
        static_cast<unsigned int>(col)
    ) > 0;
}

bool tryParseCovarianceParamName(
    const char* name,
    const char* prefix,
    size_t* indexOut)
{
    const size_t prefixLen = strlen(prefix);
    if (strncmp(name, prefix, prefixLen) != 0) {
        return false;
    }

    if (
        name[prefixLen] != '_' ||
        name[prefixLen + 1] < '0' ||
        name[prefixLen + 1] > '2' ||
        name[prefixLen + 2] < '0' ||
        name[prefixLen + 2] > '2' ||
        name[prefixLen + 3] != '\0'
    ) {
        return false;
    }

    const size_t row = static_cast<size_t>(name[prefixLen + 1] - '0');
    const size_t col = static_cast<size_t>(name[prefixLen + 2] - '0');
    *indexOut = (row * 3) + col;
    return true;
}

void applyCovariancePreset(ImuCovPresetMode mode)
{
    float oriDiag = DEFAULT_IMU_ORIENTATION_COVARIANCE;
    float angDiag = DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE;
    float linDiag = DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE;

    switch (mode) {
        case ImuCovPresetMode::LOW_NOISE:
            oriDiag = 0.01f;
            angDiag = 0.02f;
            linDiag = 0.05f;
            break;

        case ImuCovPresetMode::MEDIUM:
            oriDiag = DEFAULT_IMU_ORIENTATION_COVARIANCE;
            angDiag = DEFAULT_IMU_ANGULAR_VELOCITY_COVARIANCE;
            linDiag = DEFAULT_IMU_LINEAR_ACCELERATION_COVARIANCE;
            break;

        case ImuCovPresetMode::AGGRESSIVE:
            oriDiag = 0.08f;
            angDiag = 0.15f;
            linDiag = 0.25f;
            break;
    }

    for (size_t i = 0; i < 9; i++) {
        imuOrientationCovariance[i] = 0.0f;
        imuAngularVelocityCovariance[i] = 0.0f;
        imuLinearAccelerationCovariance[i] = 0.0f;
    }

    imuOrientationCovariance[0] = oriDiag;
    imuOrientationCovariance[4] = oriDiag;
    imuOrientationCovariance[8] = oriDiag;

    imuAngularVelocityCovariance[0] = angDiag;
    imuAngularVelocityCovariance[4] = angDiag;
    imuAngularVelocityCovariance[8] = angDiag;

    imuLinearAccelerationCovariance[0] = linDiag;
    imuLinearAccelerationCovariance[4] = linDiag;
    imuLinearAccelerationCovariance[8] = linDiag;
}

void resetCovarianceToDefault()
{
    imuCovPresetMode = static_cast<int32_t>(ImuCovPresetMode::MEDIUM);
    applyCovariancePreset(ImuCovPresetMode::MEDIUM);
}

// wheelNumber is 1, 2 or 3. Returns NAN for an invalid wheel number.
float getOdomWheelRadiusM(uint8_t wheelNumber)
{
    if (wheelNumber < 1 || wheelNumber > 3) {
        return NAN;
    }

    return odomWheelRadiusM[wheelNumber - 1];
}

void printOdomWheelRadii()
{
    Serial.println("Odometry wheel radius (effective rolling radius):");
    for (uint8_t wheel = 1; wheel <= 3; wheel++) {
        const float radiusM = getOdomWheelRadiusM(wheel);
        Serial.printf(
            "  wheel %u: %.5f m (%.2f mm)\n",
            wheel,
            radiusM,
            radiusM * 1000.0f
        );
    }
}

void dumpCurrentParameters()
{
    Serial.println("========== PARAM DUMP ==========");
    Serial.printf("heading_kp: %.3f\n", headingHoldKp);
    Serial.printf("heading_kd: %.3f\n", headingHoldKd);
    Serial.printf("yaw_rate_window: %u\n", static_cast<unsigned int>(yawRateFilterWindow));
    Serial.printf("imu_lpf_alpha: %.3f\n", imuLpfAlpha);
    Serial.printf("enable_timestamp_sync: %d\n", static_cast<int>(enableTimestampSync));
    Serial.printf("imu_cov_preset_mode: %d\n", static_cast<int>(imuCovPresetMode));
    printOdomWheelRadii();
    Serial.printf("encoder_ticks_per_rev: %.1f\n", encoderTicksPerRev);
    Serial.printf("wheel_pid_kp: %.3f\n", wheelPidKp);
    Serial.printf("wheel_pid_ki: %.3f\n", wheelPidKi);
    Serial.printf("wheel_pid_kd: %.3f\n", wheelPidKd);
    Serial.printf("imu_frame_id: %s\n", imuFrameId);

    Serial.println("orientation_covariance:");
    Serial.printf("  [%.4f %.4f %.4f]\n", imuOrientationCovariance[0], imuOrientationCovariance[1], imuOrientationCovariance[2]);
    Serial.printf("  [%.4f %.4f %.4f]\n", imuOrientationCovariance[3], imuOrientationCovariance[4], imuOrientationCovariance[5]);
    Serial.printf("  [%.4f %.4f %.4f]\n", imuOrientationCovariance[6], imuOrientationCovariance[7], imuOrientationCovariance[8]);

    Serial.println("angular_velocity_covariance:");
    Serial.printf("  [%.4f %.4f %.4f]\n", imuAngularVelocityCovariance[0], imuAngularVelocityCovariance[1], imuAngularVelocityCovariance[2]);
    Serial.printf("  [%.4f %.4f %.4f]\n", imuAngularVelocityCovariance[3], imuAngularVelocityCovariance[4], imuAngularVelocityCovariance[5]);
    Serial.printf("  [%.4f %.4f %.4f]\n", imuAngularVelocityCovariance[6], imuAngularVelocityCovariance[7], imuAngularVelocityCovariance[8]);

    Serial.println("linear_acceleration_covariance:");
    Serial.printf("  [%.4f %.4f %.4f]\n", imuLinearAccelerationCovariance[0], imuLinearAccelerationCovariance[1], imuLinearAccelerationCovariance[2]);
    Serial.printf("  [%.4f %.4f %.4f]\n", imuLinearAccelerationCovariance[3], imuLinearAccelerationCovariance[4], imuLinearAccelerationCovariance[5]);
    Serial.printf("  [%.4f %.4f %.4f]\n", imuLinearAccelerationCovariance[6], imuLinearAccelerationCovariance[7], imuLinearAccelerationCovariance[8]);
    Serial.println("===============================");
}

void syncCovarianceParametersToServer()
{
#if HAS_RCLC_PARAMETER
    if (!parameterServerReady) {
        return;
    }

    rclc_parameter_set_int(&parameterServer, PARAM_IMU_COV_PRESET_MODE, static_cast<int64_t>(imuCovPresetMode));

    for (size_t i = 0; i < 9; i++) {
        char paramName[48];

        if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ORI_COV_PREFIX, i)) {
            rclc_parameter_set_double(&parameterServer, paramName, static_cast<double>(imuOrientationCovariance[i]));
        }

        if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ANG_VEL_COV_PREFIX, i)) {
            rclc_parameter_set_double(&parameterServer, paramName, static_cast<double>(imuAngularVelocityCovariance[i]));
        }

        if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_LIN_ACC_COV_PREFIX, i)) {
            rclc_parameter_set_double(&parameterServer, paramName, static_cast<double>(imuLinearAccelerationCovariance[i]));
        }
    }
#endif
}

size_t clampYawRateWindowSize(int64_t window)
{
    if (window < 1) {
        return 1;
    }

    if (window > static_cast<int64_t>(MAX_YAW_RATE_FILTER_WINDOW)) {
        return MAX_YAW_RATE_FILTER_WINDOW;
    }

    return static_cast<size_t>(window);
}

void resetYawRateFilter()
{
    for (size_t i = 0; i < MAX_YAW_RATE_FILTER_WINDOW; i++) {
        yawRateSamples[i] = 0.0f;
    }

    yawRateSampleIndex = 0;
    yawRateSampleCount = 0;
    gy25YawRateDegPerSecond = 0.0f;
}

void resetWheelPidState()
{
    noInterrupts();
    lastPidEncoder1Ticks = encoder1Ticks;
    lastPidEncoder2Ticks = encoder2Ticks;
    lastPidEncoder3Ticks = encoder3Ticks;
    interrupts();

    for (size_t i = 0; i < 3; i++) {
        targetWheelMps[i] = 0.0f;
        measuredWheelMps[i] = 0.0f;
        wheelPidIntegral[i] = 0.0f;
        wheelPidPrevError[i] = 0.0f;
    }

    lastPidControlTime = millis();
}

void resetOdomPose()
{
    odomX = 0.0f;
    odomY = 0.0f;
    odomYaw = 0.0f;

    noInterrupts();
    lastEncoder1Ticks = encoder1Ticks;
    lastEncoder2Ticks = encoder2Ticks;
    lastEncoder3Ticks = encoder3Ticks;
    interrupts();

    resetWheelPidState();
}

void pushYawRateSample(float sampleDegPerSecond)
{
    yawRateSamples[yawRateSampleIndex] = sampleDegPerSecond;
    yawRateSampleIndex = (yawRateSampleIndex + 1) % yawRateFilterWindow;

    if (yawRateSampleCount < yawRateFilterWindow) {
        yawRateSampleCount++;
    }

    float sum = 0.0f;
    for (size_t i = 0; i < yawRateSampleCount; i++) {
        sum += yawRateSamples[i];
    }

    gy25YawRateDegPerSecond =
        (yawRateSampleCount > 0)
            ? (sum / static_cast<float>(yawRateSampleCount))
            : 0.0f;
}

void startGY25Baud(uint32_t baud)
{
    GY25.end();
    delay(20);
    GY25.begin(baud, SERIAL_8N1, GY25_RX, GY25_TX);

    frameIndex = 0;
    receivedBytes = 0;
    baudStarted = millis();

    Serial.print("Trying GY-25 UART at ");
    Serial.print(baud);
    Serial.println(" baud...");
}

void requestAngles()
{
    // Query mode: request angle frame by A5 51.
    GY25.write(0xA5);
    GY25.write(0x51);
}

void processGy25Frame(const uint8_t* data)
{
    // AA YawH YawL PitchH PitchL RollH RollL 55
    const int16_t yawRaw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[1]) << 8) | data[2]
    );

    const int16_t pitchRaw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[3]) << 8) | data[4]
    );

    const int16_t rollRaw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[5]) << 8) | data[6]
    );

    const float rawYaw = wrapAngleDegrees(
        GY25_YAW_DIRECTION * static_cast<float>(yawRaw) / 100.0f
    );

    const float rawPitch = static_cast<float>(pitchRaw) / 100.0f;
    const float rawRoll = static_cast<float>(rollRaw) / 100.0f;

    gy25YawRawDeg = rawYaw;
    gy25PitchRawDeg = rawPitch;
    gy25RollRawDeg = rawRoll;

    const float alpha = clampUnit(imuLpfAlpha);
    const bool firstFrame = !gy25ReceivedOnce;

    if (firstFrame) {
        gy25YawDeg = rawYaw;
        gy25PitchDeg = rawPitch;
        gy25RollDeg = rawRoll;
        previousGy25YawDeg = rawYaw;
    } else {
        gy25YawDeg = lowPassFilterAngleDegrees(gy25YawDeg, rawYaw, alpha);
        gy25PitchDeg = lowPassFilter(gy25PitchDeg, rawPitch, alpha);
        gy25RollDeg = lowPassFilter(gy25RollDeg, rawRoll, alpha);
    }

    const unsigned long now = millis();
    if (!firstFrame && now != lastFrame) {
        const float dt = (now - lastFrame) / 1000.0f;
        const float rawYawRateDegPerSecond =
            wrapAngleDegrees(gy25YawDeg - previousGy25YawDeg) / dt;
        pushYawRateSample(rawYawRateDegPerSecond);
    } else if (firstFrame) {
        resetYawRateFilter();
    }

    previousGy25YawDeg = gy25YawDeg;
    lastFrame = now;
    gy25ReceivedOnce = true;

    if (!headingTargetValid) {
        headingTargetDeg = gy25YawDeg;
        headingTargetValid = true;
    }

    if (!baudLocked) {
        baudLocked = true;
        Serial.print("GY-25 detected at ");
        Serial.print(BAUD_RATES[baudIndex]);
        Serial.println(" baud");
    }

    warningPrinted = false;
}

void readGY25()
{
    while (GY25.available() > 0) {
        const uint8_t value = static_cast<uint8_t>(GY25.read());
        receivedBytes++;

        // Wait for frame header byte.
        if (frameIndex == 0 && value != 0xAA) {
            continue;
        }

        frame[frameIndex++] = value;

        if (frameIndex < FRAME_SIZE) {
            continue;
        }

        if (frame[0] == 0xAA && frame[7] == 0x55) {
            processGy25Frame(frame);
        }

        // If tail byte is AA, reuse as next frame start.
        if (value == 0xAA) {
            frame[0] = value;
            frameIndex = 1;
        } else {
            frameIndex = 0;
        }
    }
}

void updateGy25()
{
    const uint32_t now = millis();

    if (now - lastRequest >= GY25_REQUEST_INTERVAL_MS) {
        lastRequest = now;
        requestAngles();
    }

    readGY25();

    // If no valid frame found yet, keep trying both baud rates.
    if (!baudLocked && now - baudStarted >= GY25_BAUD_TRY_MS) {
        Serial.print("Bytes received at this baud: ");
        Serial.println(receivedBytes);

        baudIndex =
            (baudIndex + 1) %
            (sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]));

        startGY25Baud(BAUD_RATES[baudIndex]);
    }

    if (!warningPrinted && !baudLocked && now > GY25_WARNING_AFTER_MS) {
        Serial.println(
            "No valid GY-25 frame. Check TX/RX, GND, power, and UART/I2C selector."
        );

        warningPrinted = true;
    }
}

float getHeadingControlledWz(float requestedWz, float vx, float vy)
{
    if (!isGy25Active()) {
        headingTargetValid = false;
        rotationWasCommanded = false;
        return requestedWz;
    }

    const bool rotationCommanded =
        fabsf(requestedWz) > ROTATION_COMMAND_DEADBAND;

    if (rotationCommanded) {
        // Manual rotation always has priority. Remember the latest heading so it
        // becomes the new target as soon as the joystick rotation is released.
        headingTargetDeg = gy25YawDeg;
        headingTargetValid = true;
        rotationWasCommanded = true;
        return requestedWz;
    }

    if (rotationWasCommanded || !headingTargetValid) {
        headingTargetDeg = gy25YawDeg;
        headingTargetValid = true;
        rotationWasCommanded = false;
    }

    const bool translating =
        hypotf(vx, vy) > TRANSLATION_COMMAND_DEADBAND;
    if (!translating) {
        // Do not make the stationary robot hunt around the target heading.
        headingTargetDeg = gy25YawDeg;
        return 0.0f;
    }

    const float headingErrorRad =
        wrapAngleDegrees(headingTargetDeg - gy25YawDeg) * DEG_TO_RAD;
    const float yawRateRadPerSecond = gy25YawRateDegPerSecond * DEG_TO_RAD;
    const float correction =
        (headingHoldKp * headingErrorRad) -
        (headingHoldKd * yawRateRadPerSecond);

    return clampFloat(
        correction,
        -HEADING_HOLD_MAX_WZ,
        HEADING_HOLD_MAX_WZ
    );
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

    if (fabsf(speed) < 0.01f) {
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
    /*
     * Wheel 1 = 150 degrees
     * Wheel 2 =  30 degrees
     * Wheel 3 = 270 degrees
     *
     * W1 = -0.866Vx + 0.5Vy + LxWz
     * W2 =  0.866Vx + 0.5Vy + LxWz
     * W3 =         -1.0Vy + LxWz
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

    // Preserve commanded direction while respecting max wheel linear speed.
    const float maximumWheel = max(fabsf(wheel1), max(fabsf(wheel2), fabsf(wheel3)));
    if (maximumWheel > MAX_WHEEL_LINEAR_SPEED) {
        const float scale = MAX_WHEEL_LINEAR_SPEED / maximumWheel;
        wheel1 *= scale;
        wheel2 *= scale;
        wheel3 *= scale;
    }

    targetWheelMps[0] = wheel1;
    targetWheelMps[1] = wheel2;
    targetWheelMps[2] = wheel3;
}

void applyWheelPidControl()
{
    const unsigned long now = millis();
    if (now - lastPidControlTime < PID_CONTROL_INTERVAL_MS) {
        return;
    }

    const float dt =
        (lastPidControlTime == 0)
            ? (PID_CONTROL_INTERVAL_MS / 1000.0f)
            : ((now - lastPidControlTime) / 1000.0f);
    lastPidControlTime = now;

    if (dt <= 0.0f) {
        return;
    }

    // Sample encoders in the same time base as the PID. Previously these
    // speeds were refreshed by the slower odometry publisher, which produced
    // alternating stale samples and large derivative spikes.
    int32_t tick1;
    int32_t tick2;
    int32_t tick3;
    noInterrupts();
    tick1 = encoder1Ticks;
    tick2 = encoder2Ticks;
    tick3 = encoder3Ticks;
    interrupts();

    const float meterPerTick[3] = {
        (2.0f * PI * odomWheelRadiusM[0]) / encoderTicksPerRev,
        (2.0f * PI * odomWheelRadiusM[1]) / encoderTicksPerRev,
        (2.0f * PI * odomWheelRadiusM[2]) / encoderTicksPerRev
    };
    measuredWheelMps[0] =
        ((tick1 - lastPidEncoder1Ticks) * ENCODER1_DIRECTION * meterPerTick[0]) / dt;
    measuredWheelMps[1] =
        ((tick2 - lastPidEncoder2Ticks) * ENCODER2_DIRECTION * meterPerTick[1]) / dt;
    measuredWheelMps[2] =
        ((tick3 - lastPidEncoder3Ticks) * ENCODER3_DIRECTION * meterPerTick[2]) / dt;
    lastPidEncoder1Ticks = tick1;
    lastPidEncoder2Ticks = tick2;
    lastPidEncoder3Ticks = tick3;

    float pidCommandMps[3];
    for (size_t i = 0; i < 3; i++) {
        const float error = targetWheelMps[i] - measuredWheelMps[i];
        wheelPidIntegral[i] += error * dt;
        wheelPidIntegral[i] = clampFloat(
            wheelPidIntegral[i],
            -WHEEL_PID_INTEGRAL_LIMIT,
            WHEEL_PID_INTEGRAL_LIMIT
        );

        const float derivative =
            (error - wheelPidPrevError[i]) / dt;
        wheelPidPrevError[i] = error;

        pidCommandMps[i] =
            targetWheelMps[i] +
            (wheelPidKp * error) +
            (wheelPidKi * wheelPidIntegral[i]) +
            (wheelPidKd * derivative);
    }

    const float wheel1Norm = clampFloat(
        pidCommandMps[0] / MAX_WHEEL_LINEAR_SPEED,
        -1.0f,
        1.0f
    );
    const float wheel2Norm = clampFloat(
        pidCommandMps[1] / MAX_WHEEL_LINEAR_SPEED,
        -1.0f,
        1.0f
    );
    const float wheel3Norm = clampFloat(
        pidCommandMps[2] / MAX_WHEEL_LINEAR_SPEED,
        -1.0f,
        1.0f
    );

    setMotor(
        MOTOR1_PWM_PIN,
        MOTOR1_DIR_PIN,
        wheel1Norm,
        MOTOR1_DIRECTION
    );

    setMotor(
        MOTOR2_PWM_PIN,
        MOTOR2_DIR_PIN,
        wheel2Norm,
        MOTOR2_DIRECTION
    );

    setMotor(
        MOTOR3_PWM_PIN,
        MOTOR3_DIR_PIN,
        wheel3Norm,
        MOTOR3_DIRECTION
    );
}

// ============================================================
// micro-ROS callback and setup
// ============================================================

void cmdVelCallback(const void* messageIn)
{
    const geometry_msgs__msg__Twist* message =
        static_cast<const geometry_msgs__msg__Twist*>(messageIn);

    cmdVelVx = clampFloat(
        static_cast<float>(message->linear.x),
        -MAX_CMD_VX,
        MAX_CMD_VX
    );

    cmdVelVy = clampFloat(
        static_cast<float>(message->linear.y),
        -MAX_CMD_VY,
        MAX_CMD_VY
    );

    cmdVelWz = clampFloat(
        static_cast<float>(message->angular.z),
        -MAX_CMD_WZ,
        MAX_CMD_WZ
    );

    lastCmdVelUpdate = millis();
    cmdVelReceivedOnce = true;
}

void resetHeadingCallback(const void* messageIn)
{
    const std_msgs__msg__Bool* message =
        static_cast<const std_msgs__msg__Bool*>(messageIn);

    if (!message->data) {
        return;
    }

    if (isGy25Active()) {
        headingTargetDeg = gy25YawDeg;
        headingTargetValid = true;
        rotationWasCommanded = false;
        Serial.println("Heading target reset from ROS2 command");
    } else {
        headingTargetValid = false;
        Serial.println("Heading reset requested, but GY-25 is not active");
    }
}

void setFrameIdCallback(const void* messageIn)
{
    const std_msgs__msg__String* message =
        static_cast<const std_msgs__msg__String*>(messageIn);

    if (message == nullptr || message->data.data == nullptr) {
        return;
    }

    if (setImuFrameId(message->data.data)) {
        Serial.print("IMU frame_id updated to: ");
        Serial.println(imuFrameId);
    } else {
        Serial.println("WARN: invalid IMU frame_id (empty or too long)");
    }
}

void resetImuCovCallback(const void* messageIn)
{
    const std_msgs__msg__Bool* message =
        static_cast<const std_msgs__msg__Bool*>(messageIn);

    if (!message->data) {
        return;
    }

    resetCovarianceToDefault();
    syncCovarianceParametersToServer();
    Serial.println("IMU covariance reset to default preset (MEDIUM)");
}

void dumpParamsCallback(const void* messageIn)
{
    const std_msgs__msg__Bool* message =
        static_cast<const std_msgs__msg__Bool*>(messageIn);

    if (!message->data) {
        return;
    }

    dumpCurrentParameters();
}

void resetOdomCallback(const void* messageIn)
{
    const std_msgs__msg__Bool* message =
        static_cast<const std_msgs__msg__Bool*>(messageIn);

    if (!message->data) {
        return;
    }

    resetOdomPose();
    Serial.println("Odometry pose reset to origin");
}

#if HAS_RCLC_PARAMETER
bool onParameterChanged(
    const Parameter* oldParam,
    const Parameter* newParam,
    void* context)
{
    (void)oldParam;
    (void)context;

    if (newParam == nullptr || newParam->name.data == nullptr) {
        return false;
    }

    if (strcmp(newParam->name.data, PARAM_HEADING_KP) == 0) {
        return newParam->value.double_value >= 0.0 &&
            newParam->value.double_value <= 20.0;
    }

    if (strcmp(newParam->name.data, PARAM_HEADING_KD) == 0) {
        return newParam->value.double_value >= 0.0 &&
            newParam->value.double_value <= 10.0;
    }

    if (strcmp(newParam->name.data, PARAM_YAW_RATE_WINDOW) == 0) {
        return newParam->value.integer_value >= 1 &&
            newParam->value.integer_value <=
                static_cast<int64_t>(MAX_YAW_RATE_FILTER_WINDOW);
    }

    if (strcmp(newParam->name.data, PARAM_IMU_LPF_ALPHA) == 0) {
        return newParam->value.double_value >= 0.0 &&
            newParam->value.double_value <= 1.0;
    }

    if (strcmp(newParam->name.data, PARAM_ENABLE_TIMESTAMP_SYNC) == 0) {
        return true;
    }

    if (strcmp(newParam->name.data, PARAM_IMU_COV_PRESET_MODE) == 0) {
        return newParam->value.integer_value >= static_cast<int64_t>(ImuCovPresetMode::LOW_NOISE) &&
            newParam->value.integer_value <= static_cast<int64_t>(ImuCovPresetMode::AGGRESSIVE);
    }

    if (strcmp(newParam->name.data, PARAM_ODOM_WHEEL_RADIUS_M) == 0) {
        return newParam->value.double_value >= 0.001 &&
            newParam->value.double_value <= 0.300;
    }

    if (strcmp(newParam->name.data, PARAM_ENCODER_TICKS_PER_REV) == 0) {
        return newParam->value.double_value >= 10.0 &&
            newParam->value.double_value <= 200000.0;
    }

    size_t covIndex = 0;
    if (
        tryParseCovarianceParamName(newParam->name.data, PARAM_IMU_ORI_COV_PREFIX, &covIndex) ||
        tryParseCovarianceParamName(newParam->name.data, PARAM_IMU_ANG_VEL_COV_PREFIX, &covIndex) ||
        tryParseCovarianceParamName(newParam->name.data, PARAM_IMU_LIN_ACC_COV_PREFIX, &covIndex)
    ) {
        (void)covIndex;
        return newParam->value.double_value >= -10.0 &&
            newParam->value.double_value <= 10.0;
    }

    return true;
}

void updateTunableParametersFromServer()
{
    if (!parameterServerReady) {
        return;
    }

    double kp = static_cast<double>(headingHoldKp);
    double kd = static_cast<double>(headingHoldKd);
    int64_t window = static_cast<int64_t>(yawRateFilterWindow);
    double lpfAlpha = static_cast<double>(imuLpfAlpha);
    bool timestampSync = enableTimestampSync;
    int64_t covPresetMode = static_cast<int64_t>(imuCovPresetMode);
    // The legacy ROS parameter sets all three radii together. Individual
    // calibration remains available through the three constants above.
    double odomRadius = static_cast<double>(
        (odomWheelRadiusM[0] + odomWheelRadiusM[1] + odomWheelRadiusM[2]) / 3.0f
    );
    double ticksPerRev = static_cast<double>(encoderTicksPerRev);

    if (rclc_parameter_get_double(&parameterServer, PARAM_HEADING_KP, &kp) == RCL_RET_OK) {
        headingHoldKp = static_cast<float>(kp);
    }

    if (rclc_parameter_get_double(&parameterServer, PARAM_HEADING_KD, &kd) == RCL_RET_OK) {
        headingHoldKd = static_cast<float>(kd);
    }

    if (rclc_parameter_get_int(&parameterServer, PARAM_YAW_RATE_WINDOW, &window) == RCL_RET_OK) {
        const size_t newWindow = clampYawRateWindowSize(window);
        if (newWindow != yawRateFilterWindow) {
            yawRateFilterWindow = newWindow;
            resetYawRateFilter();
        }
    }

    if (rclc_parameter_get_double(&parameterServer, PARAM_IMU_LPF_ALPHA, &lpfAlpha) == RCL_RET_OK) {
        imuLpfAlpha = clampUnit(static_cast<float>(lpfAlpha));
    }

    if (rclc_parameter_get_bool(&parameterServer, PARAM_ENABLE_TIMESTAMP_SYNC, &timestampSync) == RCL_RET_OK) {
        enableTimestampSync = timestampSync;
    }

    if (rclc_parameter_get_int(&parameterServer, PARAM_IMU_COV_PRESET_MODE, &covPresetMode) == RCL_RET_OK) {
        const int32_t mode = static_cast<int32_t>(covPresetMode);
        if (mode != imuCovPresetMode) {
            imuCovPresetMode = mode;
            applyCovariancePreset(static_cast<ImuCovPresetMode>(imuCovPresetMode));
        }
    }

    if (rclc_parameter_get_double(&parameterServer, PARAM_ODOM_WHEEL_RADIUS_M, &odomRadius) == RCL_RET_OK) {
        const float commonRadius = clampFloat(
            static_cast<float>(odomRadius),
            0.001f,
            0.300f
        );
        odomWheelRadiusM[0] = commonRadius;
        odomWheelRadiusM[1] = commonRadius;
        odomWheelRadiusM[2] = commonRadius;
    }

    if (rclc_parameter_get_double(&parameterServer, PARAM_ENCODER_TICKS_PER_REV, &ticksPerRev) == RCL_RET_OK) {
        encoderTicksPerRev = clampFloat(static_cast<float>(ticksPerRev), 10.0f, 200000.0f);
    }

    for (size_t i = 0; i < 9; i++) {
        char paramName[48];
        double value = 0.0;

        if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ORI_COV_PREFIX, i) &&
            rclc_parameter_get_double(&parameterServer, paramName, &value) == RCL_RET_OK) {
            imuOrientationCovariance[i] = clampFloat(static_cast<float>(value), -10.0f, 10.0f);
        }

        if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ANG_VEL_COV_PREFIX, i) &&
            rclc_parameter_get_double(&parameterServer, paramName, &value) == RCL_RET_OK) {
            imuAngularVelocityCovariance[i] = clampFloat(static_cast<float>(value), -10.0f, 10.0f);
        }

        if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_LIN_ACC_COV_PREFIX, i) &&
            rclc_parameter_get_double(&parameterServer, paramName, &value) == RCL_RET_OK) {
            imuLinearAccelerationCovariance[i] = clampFloat(static_cast<float>(value), -10.0f, 10.0f);
        }
    }
}
#else
void updateTunableParametersFromServer()
{
    // Parameter server library unavailable; keep compile-time defaults.
}
#endif

void eulerDegreesToQuaternion(
    float yawDeg,
    float pitchDeg,
    float rollDeg,
    float* qx,
    float* qy,
    float* qz,
    float* qw)
{
    const float yaw = yawDeg * DEG_TO_RAD;
    const float pitch = pitchDeg * DEG_TO_RAD;
    const float roll = rollDeg * DEG_TO_RAD;

    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);

    *qw = (cr * cp * cy) + (sr * sp * sy);
    *qx = (sr * cp * cy) - (cr * sp * sy);
    *qy = (cr * sp * cy) + (sr * cp * sy);
    *qz = (cr * cp * sy) - (sr * sp * cy);
}

void publishImuEuler()
{
    if (connectionState != ConnectionState::CONNECTED) {
        return;
    }

    if (!isGy25Active()) {
        return;
    }

    const unsigned long now = millis();
    if (now - lastImuPublishTime < IMU_PUBLISH_INTERVAL_MS) {
        return;
    }

    lastImuPublishTime = now;

    getRosTimeFromSystem(
        &imuMsg.header.stamp.sec,
        &imuMsg.header.stamp.nanosec
    );

    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;

    eulerDegreesToQuaternion(
        gy25YawDeg,
        gy25PitchDeg,
        gy25RollDeg,
        &qx,
        &qy,
        &qz,
        &qw
    );

    imuMsg.orientation.x = qx;
    imuMsg.orientation.y = qy;
    imuMsg.orientation.z = qz;
    imuMsg.orientation.w = qw;

    for (size_t i = 0; i < 9; i++) {
        imuMsg.orientation_covariance[i] = imuOrientationCovariance[i];
        imuMsg.angular_velocity_covariance[i] = imuAngularVelocityCovariance[i];
        imuMsg.linear_acceleration_covariance[i] = imuLinearAccelerationCovariance[i];
    }

    imuMsg.angular_velocity.x = 0.0;
    imuMsg.angular_velocity.y = 0.0;
    imuMsg.angular_velocity.z = gy25YawRateDegPerSecond * DEG_TO_RAD;

    imuMsg.linear_acceleration.x = 0.0;
    imuMsg.linear_acceleration.y = 0.0;
    imuMsg.linear_acceleration.z = 0.0;

    const rcl_ret_t publishRet = rcl_publish(&imuEulerPublisher, &imuMsg, nullptr);
    if (publishRet != RCL_RET_OK) {
        Serial.println("WARN: failed to publish /imu/data");
    }
}

void publishOdometryAndTf()
{
    if (connectionState != ConnectionState::CONNECTED) {
        return;
    }

    const unsigned long now = millis();
    if (now - lastOdomPublishTime < ODOM_PUBLISH_INTERVAL_MS) {
        return;
    }

    const float dt =
        (lastOdomPublishTime == 0)
            ? (ODOM_PUBLISH_INTERVAL_MS / 1000.0f)
            : ((now - lastOdomPublishTime) / 1000.0f);

    lastOdomPublishTime = now;

    int32_t tick1;
    int32_t tick2;
    int32_t tick3;

    noInterrupts();
    tick1 = encoder1Ticks;
    tick2 = encoder2Ticks;
    tick3 = encoder3Ticks;
    interrupts();

    const int32_t deltaTick1 = (tick1 - lastEncoder1Ticks) * ENCODER1_DIRECTION;
    const int32_t deltaTick2 = (tick2 - lastEncoder2Ticks) * ENCODER2_DIRECTION;
    const int32_t deltaTick3 = (tick3 - lastEncoder3Ticks) * ENCODER3_DIRECTION;

    lastEncoder1Ticks = tick1;
    lastEncoder2Ticks = tick2;
    lastEncoder3Ticks = tick3;

    const float meterPerTick1 =
        (2.0f * PI * odomWheelRadiusM[0]) / encoderTicksPerRev;
    const float meterPerTick2 =
        (2.0f * PI * odomWheelRadiusM[1]) / encoderTicksPerRev;
    const float meterPerTick3 =
        (2.0f * PI * odomWheelRadiusM[2]) / encoderTicksPerRev;

    const float wheel1 = (deltaTick1 * meterPerTick1) / dt;
    const float wheel2 = (deltaTick2 * meterPerTick2) / dt;
    const float wheel3 = (deltaTick3 * meterPerTick3) / dt;

    // Inverse kinematics for the same wheel model used in moveRobot().
    const float wz = (wheel1 + wheel2 + wheel3) / (3.0f * ROBOT_RADIUS);
    const float vy = (ROBOT_RADIUS * wz) - wheel3;
    const float vx = (wheel2 - wheel1) / 1.7320508f;

    odomYaw = wrapAngleDegrees(odomYaw + (wz * dt * RAD_TO_DEG));

    const float yawRad = odomYaw * DEG_TO_RAD;
    const float cosYaw = cosf(yawRad);
    const float sinYaw = sinf(yawRad);

    odomX += ((vx * cosYaw) - (vy * sinYaw)) * dt;
    odomY += ((vx * sinYaw) + (vy * cosYaw)) * dt;

    int32_t stampSec = 0;
    uint32_t stampNsec = 0;
    getRosTimeFromSystem(&stampSec, &stampNsec);

    odomMsg.header.stamp.sec = stampSec;
    odomMsg.header.stamp.nanosec = stampNsec;
    odomMsg.pose.pose.position.x = odomX;
    odomMsg.pose.pose.position.y = odomY;
    odomMsg.pose.pose.position.z = 0.0;

    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    eulerDegreesToQuaternion(odomYaw, 0.0f, 0.0f, &qx, &qy, &qz, &qw);

    odomMsg.pose.pose.orientation.x = qx;
    odomMsg.pose.pose.orientation.y = qy;
    odomMsg.pose.pose.orientation.z = qz;
    odomMsg.pose.pose.orientation.w = qw;

    odomMsg.twist.twist.linear.x = vx;
    odomMsg.twist.twist.linear.y = vy;
    odomMsg.twist.twist.linear.z = 0.0;
    odomMsg.twist.twist.angular.x = 0.0;
    odomMsg.twist.twist.angular.y = 0.0;
    odomMsg.twist.twist.angular.z = wz;

    const rcl_ret_t odomRet = rcl_publish(&odomPublisher, &odomMsg, nullptr);
    if (odomRet != RCL_RET_OK) {
        Serial.println("WARN: failed to publish /odom");
    }

    if (tfMsg.transforms.size > 0) {
        geometry_msgs__msg__TransformStamped* tf = &tfMsg.transforms.data[0];
        tf->header.stamp.sec = stampSec;
        tf->header.stamp.nanosec = stampNsec;

        tf->transform.translation.x = odomX;
        tf->transform.translation.y = odomY;
        tf->transform.translation.z = 0.0;

        tf->transform.rotation.x = qx;
        tf->transform.rotation.y = qy;
        tf->transform.rotation.z = qz;
        tf->transform.rotation.w = qw;

        const rcl_ret_t tfRet = rcl_publish(&tfPublisher, &tfMsg, nullptr);
        if (tfRet != RCL_RET_OK) {
            Serial.println("WARN: failed to publish /tf");
        }
    }

    wheelTicksMsg.x = static_cast<float>(tick1);
    wheelTicksMsg.y = static_cast<float>(tick2);
    wheelTicksMsg.z = static_cast<float>(tick3);
    rcl_publish(&wheelTicksPublisher, &wheelTicksMsg, nullptr);

    wheelVelMsg.x = wheel1;
    wheelVelMsg.y = wheel2;
    wheelVelMsg.z = wheel3;
    rcl_publish(&wheelVelPublisher, &wheelVelMsg, nullptr);
}

bool createMicroRosEntities()
{
    rosAllocator = rcl_get_default_allocator();
    parameterServerReady = false;

    if (
        rclc_support_init(
            &rosSupport,
            0,
            nullptr,
            &rosAllocator
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: micro-ROS support init failed");
        return false;
    }

    if (
        rclc_node_init_default(
            &rosNode,
            "omni_micro_ros_node",
            "",
            &rosSupport
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: micro-ROS node init failed");
        return false;
    }

    if (
        rclc_subscription_init_best_effort(
            &cmdVelSubscriber,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                geometry_msgs,
                msg,
                Twist
            ),
            "/cmd_vel"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /cmd_vel subscription init failed");
        return false;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_subscription_init_best_effort(
            &resetHeadingSubscriber,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                Bool
            ),
            "/reset_heading"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /reset_heading subscription init failed");
        return false;
    }

    if (!frameIdMsgInitialized) {
        std_msgs__msg__String__init(&setFrameIdMsg);
        frameIdMsgInitialized = true;
    }

    if (!boolMsgsInitialized) {
        std_msgs__msg__Bool__init(&resetImuCovMsg);
        std_msgs__msg__Bool__init(&dumpParamsMsg);
        boolMsgsInitialized = true;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_subscription_init_best_effort(
            &setFrameIdSubscriber,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                String
            ),
            "/set_imu_frame_id"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /set_imu_frame_id subscription init failed");
        return false;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_subscription_init_best_effort(
            &resetImuCovSubscriber,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                Bool
            ),
            "/reset_imu_cov"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /reset_imu_cov subscription init failed");
        return false;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_subscription_init_best_effort(
            &dumpParamsSubscriber,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                Bool
            ),
            "/dump_params"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /dump_params subscription init failed");
        return false;
    }

    if (
        rclc_subscription_init_best_effort(
            &resetOdomSubscriber,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                Bool
            ),
            "/reset_odom"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /reset_odom subscription init failed");
        return false;
    }

    if (
        rclc_publisher_init_best_effort(
            &imuEulerPublisher,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                sensor_msgs,
                msg,
                Imu
            ),
            "/imu/data"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /imu/data publisher init failed");
        return false;
    }

    if (
        // Nav2/tf2 normally subscribes to /odom and /tf with reliable QoS.
        rclc_publisher_init_default(
            &odomPublisher,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                nav_msgs,
                msg,
                Odometry
            ),
            "/odom"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /odom publisher init failed");
        return false;
    }

    if (
        rclc_publisher_init_default(
            &tfPublisher,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                tf2_msgs,
                msg,
                TFMessage
            ),
            "/tf"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /tf publisher init failed");
        return false;
    }

    if (
        rclc_publisher_init_best_effort(
            &wheelTicksPublisher,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                geometry_msgs,
                msg,
                Vector3
            ),
            "/debug/wheel_ticks"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /debug/wheel_ticks publisher init failed");
        return false;
    }

    if (
        rclc_publisher_init_best_effort(
            &wheelVelPublisher,
            &rosNode,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                geometry_msgs,
                msg,
                Vector3
            ),
            "/debug/wheel_vel_mps"
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: /debug/wheel_vel_mps publisher init failed");
        return false;
    }

    if (!setImuFrameId(DEFAULT_IMU_FRAME_ID)) {
        Serial.println("ERROR: imu frame_id assign failed");
        return false;
    }

    if (!rosidl_runtime_c__String__assign(&odomMsg.header.frame_id, ODOM_FRAME_ID)) {
        Serial.println("ERROR: odom header frame_id assign failed");
        return false;
    }

    if (!rosidl_runtime_c__String__assign(&odomMsg.child_frame_id, BASE_LINK_FRAME_ID)) {
        Serial.println("ERROR: odom child_frame_id assign failed");
        return false;
    }

    // Non-holonomic axes are intentionally assigned very high variance. The
    // planar x/y/yaw values remain usable by robot_localization and Nav2.
    const double poseVariance[6] = {
        0.02, 0.02, 1.0e6, 1.0e6, 1.0e6, 0.05
    };
    const double twistVariance[6] = {
        0.05, 0.05, 1.0e6, 1.0e6, 1.0e6, 0.10
    };
    for (size_t row = 0; row < 6; row++) {
        for (size_t col = 0; col < 6; col++) {
            const size_t index = (row * 6) + col;
            odomMsg.pose.covariance[index] =
                (row == col) ? poseVariance[row] : 0.0;
            odomMsg.twist.covariance[index] =
                (row == col) ? twistVariance[row] : 0.0;
        }
    }

    if (!odomTfMsgsInitialized) {
        tf2_msgs__msg__TFMessage__init(&tfMsg);
        tfMsg.transforms.data = static_cast<geometry_msgs__msg__TransformStamped*>(
            malloc(sizeof(geometry_msgs__msg__TransformStamped))
        );

        if (tfMsg.transforms.data == nullptr) {
            Serial.println("ERROR: TF message allocation failed");
            return false;
        }

        tfMsg.transforms.size = 1;
        tfMsg.transforms.capacity = 1;
        geometry_msgs__msg__TransformStamped__init(&tfMsg.transforms.data[0]);

        if (!rosidl_runtime_c__String__assign(&tfMsg.transforms.data[0].header.frame_id, ODOM_FRAME_ID)) {
            Serial.println("ERROR: TF header frame_id assign failed");
            return false;
        }

        if (!rosidl_runtime_c__String__assign(&tfMsg.transforms.data[0].child_frame_id, BASE_LINK_FRAME_ID)) {
            Serial.println("ERROR: TF child_frame_id assign failed");
            return false;
        }

        odomTfMsgsInitialized = true;
    }

#if HAS_RCLC_PARAMETER
    rclc_parameter_options_t paramOptions = {
        .notify_changed_over_dds = false,
        .max_params = 40,
        .allow_undeclared_parameters = false,
        .low_mem_mode = true
    };

    if (
        ENABLE_ROS_PARAMETER_SERVER &&
        rclc_parameter_server_init_with_option(&parameterServer, &rosNode, &paramOptions) == RCL_RET_OK
    ) {
        parameterServerReady = true;
    } else if (ENABLE_ROS_PARAMETER_SERVER) {
        Serial.println("WARN: parameter server init failed, continue without ROS2 parameters");
    }

    if (parameterServerReady) {
        if (rclc_add_parameter(&parameterServer, PARAM_HEADING_KP, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK) {
            Serial.println("WARN: add parameter heading_kp failed");
            parameterServerReady = false;
        }

        if (parameterServerReady && rclc_add_parameter(&parameterServer, PARAM_HEADING_KD, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK) {
            Serial.println("WARN: add parameter heading_kd failed");
            parameterServerReady = false;
        }

        if (parameterServerReady && rclc_add_parameter(&parameterServer, PARAM_YAW_RATE_WINDOW, RCLC_PARAMETER_INT) != RCL_RET_OK) {
            Serial.println("WARN: add parameter yaw_rate_window failed");
            parameterServerReady = false;
        }

        if (parameterServerReady && rclc_add_parameter(&parameterServer, PARAM_IMU_LPF_ALPHA, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK) {
            Serial.println("WARN: add parameter imu_lpf_alpha failed");
            parameterServerReady = false;
        }

        if (parameterServerReady && rclc_add_parameter(&parameterServer, PARAM_ENABLE_TIMESTAMP_SYNC, RCLC_PARAMETER_BOOL) != RCL_RET_OK) {
            Serial.println("WARN: add parameter enable_timestamp_sync failed");
            parameterServerReady = false;
        }

        if (parameterServerReady && rclc_add_parameter(&parameterServer, PARAM_IMU_COV_PRESET_MODE, RCLC_PARAMETER_INT) != RCL_RET_OK) {
            Serial.println("WARN: add parameter imu_cov_preset_mode failed");
            parameterServerReady = false;
        }

        if (parameterServerReady && rclc_add_parameter(&parameterServer, PARAM_ODOM_WHEEL_RADIUS_M, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK) {
            Serial.println("WARN: add parameter odom_wheel_radius_m failed");
            parameterServerReady = false;
        }

        if (parameterServerReady && rclc_add_parameter(&parameterServer, PARAM_ENCODER_TICKS_PER_REV, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK) {
            Serial.println("WARN: add parameter encoder_ticks_per_rev failed");
            parameterServerReady = false;
        }

        for (size_t i = 0; parameterServerReady && i < 9; i++) {
            char paramName[48];

            if (
                !buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ORI_COV_PREFIX, i) ||
                rclc_add_parameter(&parameterServer, paramName, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK
            ) {
                Serial.println("WARN: add imu_orientation_cov matrix parameter failed");
                parameterServerReady = false;
                break;
            }

            if (
                !buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ANG_VEL_COV_PREFIX, i) ||
                rclc_add_parameter(&parameterServer, paramName, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK
            ) {
                Serial.println("WARN: add imu_angular_velocity_cov matrix parameter failed");
                parameterServerReady = false;
                break;
            }

            if (
                !buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_LIN_ACC_COV_PREFIX, i) ||
                rclc_add_parameter(&parameterServer, paramName, RCLC_PARAMETER_DOUBLE) != RCL_RET_OK
            ) {
                Serial.println("WARN: add imu_linear_acceleration_cov matrix parameter failed");
                parameterServerReady = false;
                break;
            }
        }

        if (!parameterServerReady) {
            rclc_parameter_server_fini(&parameterServer, &rosNode);
        }
    }

    if (parameterServerReady) {
        rclc_parameter_set_double(&parameterServer, PARAM_HEADING_KP, static_cast<double>(headingHoldKp));
        rclc_parameter_set_double(&parameterServer, PARAM_HEADING_KD, static_cast<double>(headingHoldKd));
        rclc_parameter_set_int(&parameterServer, PARAM_YAW_RATE_WINDOW, static_cast<int64_t>(yawRateFilterWindow));
        rclc_parameter_set_double(&parameterServer, PARAM_IMU_LPF_ALPHA, static_cast<double>(imuLpfAlpha));
        rclc_parameter_set_bool(&parameterServer, PARAM_ENABLE_TIMESTAMP_SYNC, enableTimestampSync);
        rclc_parameter_set_int(&parameterServer, PARAM_IMU_COV_PRESET_MODE, static_cast<int64_t>(imuCovPresetMode));
        const double averageWheelRadius = static_cast<double>(
            (odomWheelRadiusM[0] + odomWheelRadiusM[1] + odomWheelRadiusM[2]) / 3.0f
        );
        rclc_parameter_set_double(&parameterServer, PARAM_ODOM_WHEEL_RADIUS_M, averageWheelRadius);
        rclc_parameter_set_double(&parameterServer, PARAM_ENCODER_TICKS_PER_REV, static_cast<double>(encoderTicksPerRev));

        for (size_t i = 0; i < 9; i++) {
            char paramName[48];

            if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ORI_COV_PREFIX, i)) {
                rclc_parameter_set_double(&parameterServer, paramName, static_cast<double>(imuOrientationCovariance[i]));
            }

            if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_ANG_VEL_COV_PREFIX, i)) {
                rclc_parameter_set_double(&parameterServer, paramName, static_cast<double>(imuAngularVelocityCovariance[i]));
            }

            if (buildCovarianceParamName(paramName, sizeof(paramName), PARAM_IMU_LIN_ACC_COV_PREFIX, i)) {
                rclc_parameter_set_double(&parameterServer, paramName, static_cast<double>(imuLinearAccelerationCovariance[i]));
            }
        }
    }
#endif

    // Essential subscriptions: /cmd_vel and /reset_odom.
    size_t executorHandles = 2;
    if (ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES) {
        executorHandles += 4;
    }
#if HAS_RCLC_PARAMETER
    if (parameterServerReady) {
        executorHandles += RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES;
    }
#endif
    if (
        rclc_executor_init(
            &rosExecutor,
            &rosSupport.context,
            executorHandles,
            &rosAllocator
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: micro-ROS executor init failed");
        return false;
    }

    if (
        rclc_executor_add_subscription(
            &rosExecutor,
            &cmdVelSubscriber,
            &cmdVelMsg,
            &cmdVelCallback,
            ON_NEW_DATA
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: add /cmd_vel subscription failed");
        return false;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_executor_add_subscription(
            &rosExecutor,
            &resetHeadingSubscriber,
            &resetHeadingMsg,
            &resetHeadingCallback,
            ON_NEW_DATA
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: add /reset_heading subscription failed");
        return false;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_executor_add_subscription(
            &rosExecutor,
            &setFrameIdSubscriber,
            &setFrameIdMsg,
            &setFrameIdCallback,
            ON_NEW_DATA
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: add /set_imu_frame_id subscription failed");
        return false;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_executor_add_subscription(
            &rosExecutor,
            &resetImuCovSubscriber,
            &resetImuCovMsg,
            &resetImuCovCallback,
            ON_NEW_DATA
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: add /reset_imu_cov subscription failed");
        return false;
    }

    if (
        ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES &&
        rclc_executor_add_subscription(
            &rosExecutor,
            &dumpParamsSubscriber,
            &dumpParamsMsg,
            &dumpParamsCallback,
            ON_NEW_DATA
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: add /dump_params subscription failed");
        return false;
    }

    if (
        rclc_executor_add_subscription(
            &rosExecutor,
            &resetOdomSubscriber,
            &resetOdomMsg,
            &resetOdomCallback,
            ON_NEW_DATA
        ) != RCL_RET_OK
    ) {
        Serial.println("ERROR: add /reset_odom subscription failed");
        return false;
    }

#if HAS_RCLC_PARAMETER
    if (
        parameterServerReady &&
        rclc_executor_add_parameter_server_with_context(
            &rosExecutor,
            &parameterServer,
            onParameterChanged,
            nullptr
        ) != RCL_RET_OK
    ) {
        Serial.println("WARN: add parameter server to executor failed, continue without ROS2 parameters");
        parameterServerReady = false;
    }
#endif

    microRosReady = true;
    if (parameterServerReady) {
        Serial.println("micro-ROS ready: Nav2 core topics, params enabled");
    } else {
        Serial.println("micro-ROS ready: /cmd_vel, /odom, /tf, /imu/data");
    }
    return true;
}

void destroyMicroRosEntities()
{
    stopRobot();
    cmdVelReceivedOnce = false;
    microRosReady = false;

    // Do not wait for a dead agent while releasing local entities.
    rmw_context_t* rmwContext = rcl_context_get_rmw_context(&rosSupport.context);
    if (rmwContext != nullptr) {
        rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0);
    }

    rclc_executor_fini(&rosExecutor);
    rcl_subscription_fini(&cmdVelSubscriber, &rosNode);
    rcl_subscription_fini(&resetHeadingSubscriber, &rosNode);
    rcl_subscription_fini(&setFrameIdSubscriber, &rosNode);
    rcl_subscription_fini(&resetImuCovSubscriber, &rosNode);
    rcl_subscription_fini(&dumpParamsSubscriber, &rosNode);
    rcl_subscription_fini(&resetOdomSubscriber, &rosNode);
    rcl_publisher_fini(&imuEulerPublisher, &rosNode);
    rcl_publisher_fini(&odomPublisher, &rosNode);
    rcl_publisher_fini(&tfPublisher, &rosNode);
    rcl_publisher_fini(&wheelTicksPublisher, &rosNode);
    rcl_publisher_fini(&wheelVelPublisher, &rosNode);
#if HAS_RCLC_PARAMETER
    if (parameterServerReady) {
        rclc_parameter_server_fini(&parameterServer, &rosNode);
    }
#endif
    rcl_node_fini(&rosNode);
    rclc_support_fini(&rosSupport);

    rosNode = rcl_get_zero_initialized_node();
    cmdVelSubscriber = rcl_get_zero_initialized_subscription();
    resetHeadingSubscriber = rcl_get_zero_initialized_subscription();
    setFrameIdSubscriber = rcl_get_zero_initialized_subscription();
    resetImuCovSubscriber = rcl_get_zero_initialized_subscription();
    dumpParamsSubscriber = rcl_get_zero_initialized_subscription();
    resetOdomSubscriber = rcl_get_zero_initialized_subscription();
    imuEulerPublisher = rcl_get_zero_initialized_publisher();
    odomPublisher = rcl_get_zero_initialized_publisher();
    tfPublisher = rcl_get_zero_initialized_publisher();
    wheelTicksPublisher = rcl_get_zero_initialized_publisher();
    wheelVelPublisher = rcl_get_zero_initialized_publisher();
    rosExecutor = rclc_executor_get_zero_initialized_executor();
    parameterServerReady = false;

    if (frameIdMsgInitialized) {
        std_msgs__msg__String__fini(&setFrameIdMsg);
        frameIdMsgInitialized = false;
    }

    if (boolMsgsInitialized) {
        std_msgs__msg__Bool__fini(&resetImuCovMsg);
        std_msgs__msg__Bool__fini(&dumpParamsMsg);
        std_msgs__msg__Bool__fini(&resetOdomMsg);
        boolMsgsInitialized = false;
    }

    rosidl_runtime_c__String__assign(&imuMsg.header.frame_id, "");

    if (odomTfMsgsInitialized) {
        if (tfMsg.transforms.data != nullptr && tfMsg.transforms.size > 0) {
            geometry_msgs__msg__TransformStamped__fini(&tfMsg.transforms.data[0]);
            free(tfMsg.transforms.data);
            tfMsg.transforms.data = nullptr;
            tfMsg.transforms.size = 0;
            tfMsg.transforms.capacity = 0;
        }

        tf2_msgs__msg__TFMessage__fini(&tfMsg);
        odomTfMsgsInitialized = false;
    }
}

void updateMicroRosConnection()
{
    const unsigned long now = millis();

    switch (connectionState) {
        case ConnectionState::WAITING_FOR_AGENT:
            if (now - lastAgentPingTime >= AGENT_SEARCH_INTERVAL_MS) {
                lastAgentPingTime = now;
                if (RMW_RET_OK == rmw_uros_ping_agent(150, 2)) {
                    consecutiveAgentPingFailures = 0;
                    connectionState = ConnectionState::AGENT_AVAILABLE;
                }
            }
            break;

        case ConnectionState::AGENT_AVAILABLE:
            if (createMicroRosEntities()) {
                connectionState = ConnectionState::CONNECTED;
            } else {
                destroyMicroRosEntities();
                connectionState = ConnectionState::WAITING_FOR_AGENT;
            }
            break;

        case ConnectionState::CONNECTED:
            rclc_executor_spin_some(&rosExecutor, RCL_MS_TO_NS(5));
            if (now - lastAgentPingTime >= AGENT_HEALTH_CHECK_INTERVAL_MS) {
                lastAgentPingTime = now;
                if (RMW_RET_OK == rmw_uros_ping_agent(150, 2)) {
                    consecutiveAgentPingFailures = 0;
                } else if (++consecutiveAgentPingFailures >= AGENT_MAX_CONSECUTIVE_FAILURES) {
                    Serial.printf(
                        "micro-ROS agent missed %u consecutive health checks\n",
                        consecutiveAgentPingFailures
                    );
                    connectionState = ConnectionState::DISCONNECTED;
                }
            }
            break;

        case ConnectionState::DISCONNECTED:
            destroyMicroRosEntities();
            Serial.println("micro-ROS agent disconnected; motors stopped");
            consecutiveAgentPingFailures = 0;
            connectionState = ConnectionState::WAITING_FOR_AGENT;
            break;
    }
}


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
        "cmd:%d | vx:%0.2f vy:%0.2f wz:%0.2f | IMU:%d yaw:%0.2f pitch:%0.2f roll:%0.2f yawRate:%0.2f target:%0.2f | kp:%0.2f kd:%0.2f win:%u alpha:%0.2f ts_sync:%d frame:%s | odomR:[%0.3f,%0.3f,%0.3f] tpr:%0.1f\n",
        isCmdVelActive(),
        cmdVelVx,
        cmdVelVy,
        cmdVelWz,
        isGy25Active(),
        gy25YawDeg,
        gy25PitchDeg,
        gy25RollDeg,
        gy25YawRateDegPerSecond,
        headingTargetDeg,
        headingHoldKp,
        headingHoldKd,
        static_cast<unsigned int>(yawRateFilterWindow),
        imuLpfAlpha,
        static_cast<int>(enableTimestampSync),
        imuFrameId,
        odomWheelRadiusM[0],
        odomWheelRadiusM[1],
        odomWheelRadiusM[2],
        encoderTicksPerRev
    );
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);
    delay(3000);

    Serial.println("GY-25 automatic UART detection");
    startGY25Baud(BAUD_RATES[baudIndex]);

    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR3_DIR_PIN, OUTPUT);

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
        Serial.println("ERROR: PWM configuration failed");
    }

    stopRobot();

    set_microros_wifi_transports(
        WIFI_SSID,
        WIFI_PASSWORD,
        AGENT_IP,
        AGENT_PORT
    );

    // ESP32 modem sleep can delay/drop short UDP exchanges such as XRCE-DDS
    // heartbeats and agent pings. Keep the radio awake for stable discovery.
    WiFi.setSleep(false);

    Serial.println();
    Serial.println("======================================");
    Serial.println("3-Wheel Omni micro-ROS Ready");
    Serial.println("Subscribe topic: /cmd_vel");
    Serial.println("Subscribe topic: /reset_odom (std_msgs/Bool)");
    if (ENABLE_OPTIONAL_ROS_CONFIG_INTERFACES) {
        Serial.println("Optional config subscriptions enabled");
    }
    Serial.println("Publish topic: /imu/data (sensor_msgs/Imu)");
    Serial.println("Publish topic: /odom (nav_msgs/Odometry)");
    Serial.println("Publish topic: /tf (tf2_msgs/TFMessage)");
    Serial.println("Publish topic: /debug/wheel_ticks (geometry_msgs/Vector3)");
    Serial.println("Publish topic: /debug/wheel_vel_mps (geometry_msgs/Vector3)");
    Serial.println("Control source: teleop_twist_keyboard");
    Serial.println("GY-25 heading hold: UART2 RX=16 TX=17 (query mode)");
#if HAS_RCLC_PARAMETER
    if (ENABLE_ROS_PARAMETER_SERVER) {
    Serial.println("ROS2 parameters: heading_kp, heading_kd, yaw_rate_window, imu_lpf_alpha,");
    Serial.println("                 enable_timestamp_sync,");
    Serial.println("                 imu_cov_preset_mode (0=low_noise, 1=medium, 2=aggressive),");
    Serial.println("                 odom_wheel_radius_m, encoder_ticks_per_rev,");
    Serial.println("                 imu_orientation_cov_00..22,");
    Serial.println("                 imu_angular_velocity_cov_00..22,");
    Serial.println("                 imu_linear_acceleration_cov_00..22");
    Serial.println("NOTE: string parameter is not supported in rclc_parameter, use /set_imu_frame_id");
    }
#else
    Serial.println("WARN: rclc_parameter not found, ROS2 parameters disabled");
#endif
    Serial.printf("Agent: %s:%u\n", AGENT_IP, AGENT_PORT);
    Serial.println("======================================");

    printOdomWheelRadii();

    initEncoders();
    resetOdomPose();

    if (!boolMsgsInitialized) {
        std_msgs__msg__Bool__init(&resetImuCovMsg);
        std_msgs__msg__Bool__init(&dumpParamsMsg);
        std_msgs__msg__Bool__init(&resetOdomMsg);
        boolMsgsInitialized = true;
    }
}

// ============================================================
// Main loop
// ============================================================

void loop()
{
    updateGy25();
    updateMicroRosConnection();
    updateTunableParametersFromServer();
    publishImuEuler();
    publishOdometryAndTf();

    if (connectionState == ConnectionState::CONNECTED && isCmdVelActive()) {
        const float controlledWz = getHeadingControlledWz(
            cmdVelWz,
            cmdVelVx,
            cmdVelVy
        );
        moveRobot(cmdVelVx, cmdVelVy, controlledWz);
        applyWheelPidControl();
    } else {
        // Stop robot if command stream is lost.
        stopRobot();
        resetWheelPidState();

        // If the robot is turned by hand while stopped, use that direction as
        // the next heading target instead of trying to return to an old angle.
        if (isGy25Active()) {
            headingTargetDeg = gy25YawDeg;
            headingTargetValid = true;
            rotationWasCommanded = false;
        }
    }

    printRobotStatus();
    delay(2);
}
