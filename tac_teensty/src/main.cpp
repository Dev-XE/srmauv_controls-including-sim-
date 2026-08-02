//editted on jun 13th
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32.h>

#include <MS5837.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error Serial transport required
#endif

// ============================================================
// Configuration & Hardware Pins
// ============================================================
#define NUM_THRUSTERS 8
#define PWM_MIN 1200
#define PWM_MAX 1800
#define PWM_NEUTRAL 1500
#define CMD_TIMEOUT_MS 1200
#define CONTROL_PERIOD_MS 100

const uint8_t thruster_pins[NUM_THRUSTERS] = {5, 3, 4, 2, 6, 9, 8, 7};
Servo thrusters[NUM_THRUSTERS];

const uint8_t PIN_ARM_GRIPPER = 14;
const uint8_t PIN_ARM_ROLL    = 15;
const uint8_t PIN_ARM_PITCH   = 22;
Servo arm_gripper;
Servo arm_roll;
Servo arm_pitch;

const uint8_t PIN_LIGHT1_EN = 23; 
const uint8_t PIN_LIGHT2_EN = 24;

MS5837 bar02;
bool bar02_connected = false;

// ============================================================
// 12-Direction Independent Thruster Matrix
// ============================================================
// Columns: T1, T2, T3, T4, T5, T6, T7, T8
// Adjust these weights (0.0 to 1.0) to fine-tune your specific AUV chassis.
const float thrust_matrix[12][NUM_THRUSTERS] = {
  // --- HORIZONTAL COMMANDS ---
  // Surge uses ONLY T3 and T4. (IITM logic applies opposite signs to counter prop spin)
  { -1.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0}, // 0: +Surge (Forward)
  { 1.0,  0.0,  1.0, 0.0,  0.0,  0.0,  0.0,  0.0}, // 1: -Surge (Backward)
  
  // Sway uses ONLY T1 and T2. (IITM logic applies opposite signs)
  { 0.0,  -1.0,  0.0,  -1.0,  0.0,  0.0,  0.0,  0.0}, // 2: +Sway (Right)
  { 0.0, 1.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0}, // 3: -Sway (Left)

  // --- VERTICAL COMMANDS ---
  // Heave pushes all 4 vertical thrusters in the same direction (IITM logic)
  { 0.0,  0.0,  0.0,  0.0,  1.0,  -1.0,  1.0,  -1.0}, // 4: +Heave (Up)
  { 0.0,  0.0,  0.0,  0.0, -1.0, 1.0, -1.0, 1.0}, // 5: -Heave (Down)
  
  // Roll dips one side and raises the other (T5/T7 vs T6/T8)
  { 0.0,  0.0,  0.0,  0.0,  1.0, -1.0,  1.0, -1.0}, // 6: +Roll (Right)
  { 0.0,  0.0,  0.0,  0.0, -1.0,  1.0, -1.0,  1.0}, // 7: -Roll (Left)
  
  // Pitch dips the front and raises the back (T5/T6 vs T7/T8)
  { 0.0,  0.0,  0.0,  0.0,  1.0,  1.0, -1.0, -1.0}, // 8: +Pitch (Nose Up)
  { 0.0,  0.0,  0.0,  0.0, -1.0, -1.0,  1.0,  1.0}, // 9: -Pitch (Nose Down)

  // --- ROTATIONAL COMMANDS ---
  // Yaw uses Sway thrusters T1 and T2. (IITM logic uses matching signs to spin)
  {0.0, 1.0,  0.0, -1.0,  0.0,  0.0,  0.0,  0.0}, // 10: +Yaw (Clockwise)
  { 0.0,  -1.0,  0.0,  1.0,  0.0,  0.0,  0.0,  0.0}  // 11: -Yaw (Counter-Clockwise)
};

// ============================================================
// micro-ROS entities & State
// ============================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t control_timer;

rcl_subscription_t cmd_sub, arm_sub, manip_sub, light_sub;
rcl_publisher_t    depth_pub;

geometry_msgs__msg__Twist          cmd_msg;
std_msgs__msg__Bool                arm_msg;
std_msgs__msg__Int16MultiArray     manip_msg;
std_msgs__msg__Int16MultiArray     light_msg;
std_msgs__msg__Float32             depth_msg;

int16_t manip_data_buffer[3];
std_msgs__msg__MultiArrayDimension manip_dim_buffer[1];
int16_t light_data_buffer[2];
std_msgs__msg__MultiArrayDimension light_dim_buffer[1];

static bool ros_connected = false;
static bool entities_created = false;
static uint32_t last_cmd_ms = 0;
volatile bool armed = false;

volatile float cmd_surge = 0.0f, cmd_sway = 0.0f, cmd_heave = 0.0f;
volatile float cmd_roll = 0.0f, cmd_pitch = 0.0f, cmd_yaw = 0.0f;

// ============================================================
// Utility & Mixer
// ============================================================
static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }
static inline int clamp_pwm(int v) { return (v < PWM_MIN) ? PWM_MIN : ((v > PWM_MAX) ? PWM_MAX : v); }
static inline int norm_to_pwm(float x) { return clamp_pwm(PWM_NEUTRAL + (int)(clampf(x, -1.0f, 1.0f) * (PWM_MAX - PWM_NEUTRAL))); }

static inline void neutral_all() {
  for (int i = 0; i < NUM_THRUSTERS; i++) thrusters[i].writeMicroseconds(PWM_NEUTRAL);
}

void null_axis(float *value, float deadband) { if (fabsf(*value) < deadband) *value = 0.0f; }

void compute_12dof_mix(float out[NUM_THRUSTERS]) {
  float surge = cmd_surge, sway = cmd_sway, heave = cmd_heave;
  float roll = cmd_roll, pitch = cmd_pitch, yaw = cmd_yaw;

  // Apply deadbands
  null_axis(&surge, 0.05f); null_axis(&sway, 0.1f); null_axis(&heave, 0.05f);
  null_axis(&roll, 0.1f); null_axis(&pitch, 0.1f); null_axis(&yaw, 0.1f);

  // Determine active matrix rows based on command polarity
  int srg_idx = (surge >= 0) ? 0 : 1; float srg_val = fabsf(surge);
  int swy_idx = (sway >= 0)  ? 2 : 3; float swy_val = fabsf(sway);
  int hve_idx = (heave >= 0) ? 4 : 5; float hve_val = fabsf(heave);
  int rol_idx = (roll >= 0)  ? 6 : 7; float rol_val = fabsf(roll);
  int pit_idx = (pitch >= 0) ? 8 : 9; float pit_val = fabsf(pitch);
  int yaw_idx = (yaw >= 0)   ? 10 : 11; float yaw_val = fabsf(yaw);

  float max_mag = 0.0f;

  for (int i = 0; i < NUM_THRUSTERS; i++) {
    out[i] = (thrust_matrix[srg_idx][i] * srg_val) +
             (thrust_matrix[swy_idx][i] * swy_val) +
             (thrust_matrix[hve_idx][i] * hve_val) +
             (thrust_matrix[rol_idx][i] * rol_val) +
             (thrust_matrix[pit_idx][i] * pit_val) +
             (thrust_matrix[yaw_idx][i] * yaw_val);

    if (fabsf(out[i]) > max_mag) max_mag = fabsf(out[i]);
  }

  // Normalize outputs if combined commands exceed thruster maximum limits
  if (max_mag > 1.0f) {
    for (int i = 0; i < NUM_THRUSTERS; i++) out[i] /= max_mag;
  }
}

// ============================================================
// Callbacks
// ============================================================
void cmd_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_surge = msg->linear.x; cmd_sway  = msg->linear.y; cmd_heave = msg->linear.z;
  cmd_roll  = msg->angular.x; cmd_pitch = msg->angular.y; cmd_yaw   = msg->angular.z;
  last_cmd_ms = millis();
}

void arm_cb(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  armed = msg->data;
  if (!armed) neutral_all();
}

void manip_cb(const void *msgin) {
  const std_msgs__msg__Int16MultiArray *msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  if (msg->data.size == 3) {
    arm_gripper.writeMicroseconds(msg->data.data[0]);
    arm_roll.writeMicroseconds(msg->data.data[1]);
    arm_pitch.writeMicroseconds(msg->data.data[2]);
  }
}

void light_cb(const void *msgin) {
  const std_msgs__msg__Int16MultiArray *msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  if (msg->data.size == 2) {
    analogWrite(PIN_LIGHT1_EN, constrain(msg->data.data[0], 0, 255));
    analogWrite(PIN_LIGHT2_EN, constrain(msg->data.data[1], 0, 255));
  }
}

void control_timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;

  if (bar02_connected) {
    bar02.read();
    depth_msg.data = bar02.depth(); 
    rcl_publish(&depth_pub, &depth_msg, NULL);
  }

  if (!armed || (millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
    neutral_all();
    return;
  }

  float outputs[NUM_THRUSTERS] = {0};
  compute_12dof_mix(outputs);
  for (int i = 0; i < NUM_THRUSTERS; i++) thrusters[i].writeMicroseconds(norm_to_pwm(outputs[i]));
}

// ============================================================
// Entity creation / destruction
// ============================================================

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "basic_auv_controller", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "nemo_auv/cmd_vel") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&arm_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "nemo_auv/arm") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&manip_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "nemo_auv/manipulator") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&light_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "nemo_auv/lights") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&depth_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "nemo_auv/depth") != RCL_RET_OK) return false;

  if (rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(CONTROL_PERIOD_MS), control_timer_cb) != RCL_RET_OK) return false;
  if (rclc_executor_init(&executor, &support.context, 5, &allocator) != RCL_RET_OK) return false;

  if (rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_cb, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &arm_sub, &arm_msg, &arm_cb, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &manip_sub, &manip_msg, &manip_cb, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &light_sub, &light_msg, &light_cb, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &control_timer) != RCL_RET_OK) return false;

  last_cmd_ms = millis();
  entities_created = true;
  return true;
}

void destroy_entities()
{
  if (!entities_created) return;

  entities_created = false;
  neutral_all();

  rcl_subscription_fini(&cmd_sub,   &node);
  rcl_subscription_fini(&arm_sub,   &node);
  rcl_subscription_fini(&manip_sub, &node);
  rcl_subscription_fini(&light_sub, &node);
  rcl_publisher_fini(&depth_pub,    &node);

  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ============================================================
// Setup & Loop
// ============================================================
void setup() {
  Serial.begin(115200);

  // I2C Setup with Anti-Crash Hardware Timeouts
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();
  Wire.setClock(100000);
  
  // FIX 1: Set timeout to 50,000 microseconds (50ms) to allow sensor conversions
  delay(100);

  bar02.setModel(MS5837::MS5837_02BA);
  if (bar02.init()) {
    bar02_connected = true;
    bar02.setFluidDensity(997);
  }

  set_microros_serial_transports(Serial);
  delay(1500);

  for (int i = 0; i < NUM_THRUSTERS; i++) thrusters[i].attach(thruster_pins[i]);
  
  arm_gripper.attach(PIN_ARM_GRIPPER); arm_roll.attach(PIN_ARM_ROLL); arm_pitch.attach(PIN_ARM_PITCH);
  arm_gripper.writeMicroseconds(PWM_NEUTRAL); arm_roll.writeMicroseconds(PWM_NEUTRAL); arm_pitch.writeMicroseconds(PWM_NEUTRAL);

  pinMode(PIN_LIGHT1_EN, OUTPUT); pinMode(PIN_LIGHT2_EN, OUTPUT);

  // FIX 2: Re-allocate memory buffers for micro-ROS arrays
  manip_msg.data.capacity       = 3;
  manip_msg.data.size           = 0;
  manip_msg.data.data           = manip_data_buffer;
  manip_msg.layout.dim.capacity = 1;
  manip_msg.layout.dim.size     = 0;
  manip_msg.layout.dim.data     = manip_dim_buffer;

  light_msg.data.capacity       = 2;
  light_msg.data.size           = 0;
  light_msg.data.data           = light_data_buffer;
  light_msg.layout.dim.capacity = 1;
  light_msg.layout.dim.size     = 0;
  light_msg.layout.dim.data     = light_dim_buffer;

  neutral_all();
  delay(2000);
  last_cmd_ms = millis();
}
void loop() {
  // Clear the I2C timeout flag if a hardware reset was triggered during the loop
  if (!ros_connected) {
    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
      if (create_entities()) ros_connected = true;
    delay(200);
    return;
  }

  if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    destroy_entities();
    ros_connected = false;
    neutral_all();
    delay(200);
    return;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}