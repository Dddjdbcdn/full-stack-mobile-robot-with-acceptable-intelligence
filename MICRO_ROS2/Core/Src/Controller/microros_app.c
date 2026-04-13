#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "microros_app.h"
#include "imu.h"
#include "motor.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/point32.h>

extern void I2C1_Clear_Busy_Flag(void);
extern void MX_I2C1_Init(void);

#define PI 3.14159265358979323846f
#define WHEEL_RADIUS 0.0325
#define WHEEL_SEPERATION 0.396

// --- TRANSPORT DECLARATIONS ---
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// ==========================================
// GLOBAL MICRO-ROS VARIABLES 
// ==========================================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// --- PUBLISHERS ---
rcl_publisher_t debug_publisher;
std_msgs__msg__String debug_msg;

rcl_publisher_t imu_publisher;
geometry_msgs__msg__Point32 imu_msg;

rcl_publisher_t wheel_state_publisher;
sensor_msgs__msg__JointState wheel_state_msg;

// Publisher Memory Buffers
rosidl_runtime_c__String pub_names[2];
double pub_positions[2];
double pub_velocities[2];
char pub_name_L[] = "left_wheel_joint";
char pub_name_R[] = "right_wheel_joint";

// --- SUBSCRIBER ---
rcl_subscription_t wheel_cmd_subscriber;
sensor_msgs__msg__JointState wheel_cmd_msg;

// Subscriber Memory Buffers
rosidl_runtime_c__String sub_names[2];
char sub_name_string_L[30]; 
char sub_name_string_R[30];
double sub_positions[2];   
double sub_velocities[2];   
double sub_efforts[2];   

// Params

rclc_parameter_server_t param_server;

// ==========================================
// CALLBACKS & FUNCTIONS
// ==========================================
void debug_print(const char *format, ...)
{
    static uint count = 0;
    if (++count >= 1)
    {
        static char buffer[128];

        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        debug_msg.data.data = buffer;
        debug_msg.data.size = strlen(buffer);
        debug_msg.data.capacity = sizeof(buffer);

        rcl_ret_t ret;

        ret=rcl_publish(&debug_publisher, &debug_msg, NULL);
        (void)ret;

        count = 0;
    }
}

void wheel_cmd_callback(const void * msgin)
{
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

    if (msg->velocity.size >= 2)
    {
        float target_rad_s_L = (float)msg->velocity.data[0];
        float target_rad_s_R = (float)msg->velocity.data[1];

        target_rpm_L = target_rad_s_L * (60.0f / (2.0f * M_PI));
        target_rpm_R = target_rad_s_R * (60.0f / (2.0f * M_PI));
    }
}

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context) {
  (void) context;
  if (new_param == NULL) return false;

  if (strcmp(new_param->name.data, "kf") == 0) {
    Kf = (float)new_param->value.double_value;
  } else if (strcmp(new_param->name.data, "kp") == 0) {
    Kp = (float)new_param->value.double_value;
  } else if (strcmp(new_param->name.data, "ki") == 0) {
    Ki = (float)new_param->value.double_value;
  } else if (strcmp(new_param->name.data, "kd") == 0) {
    Kd = (float)new_param->value.double_value;
  }

  return true;
}

// ==========================================
// MAIN TASK
// ==========================================
void run_microros_app()
{
    // --- 1. Hardware Initialization ---
    bool init_status = MPU6050_Init(&hi2c1);

    if (init_status) {
        MPU6050_Calibrate(&hi2c1);
    }

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Base_Start_IT(&htim4);

    // --- 2. Micro-ROS Transport Setup ---
    rmw_uros_set_custom_transport(
      true,
      (void *) &huart2,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "cubemx_node", "", &support);

    // --- 3. WIRING MEMORY ---
    // Publisher Memory
    wheel_state_msg.name.data = pub_names;
    wheel_state_msg.name.size = 2;
    wheel_state_msg.name.capacity = 2;

    wheel_state_msg.name.data[0].data = pub_name_L;
    wheel_state_msg.name.data[0].size = strlen(pub_name_L);
    wheel_state_msg.name.data[0].capacity = strlen(pub_name_L) + 1;

    wheel_state_msg.name.data[1].data = pub_name_R;
    wheel_state_msg.name.data[1].size = strlen(pub_name_R);
    wheel_state_msg.name.data[1].capacity = strlen(pub_name_R) + 1;

    wheel_state_msg.position.data = pub_positions;
    wheel_state_msg.position.size = 2;
    wheel_state_msg.position.capacity = 2;

    wheel_state_msg.velocity.data = pub_velocities;
    wheel_state_msg.velocity.size = 2;
    wheel_state_msg.velocity.capacity = 2;

    // Subscriber Memory
    wheel_cmd_msg.name.data = sub_names;
    wheel_cmd_msg.name.capacity = 2;
    wheel_cmd_msg.name.size = 0;

    wheel_cmd_msg.name.data[0].data = sub_name_string_L;
    wheel_cmd_msg.name.data[0].capacity = 30;
    wheel_cmd_msg.name.data[0].size = 0;

    wheel_cmd_msg.name.data[1].data = sub_name_string_R;
    wheel_cmd_msg.name.data[1].capacity = 30;
    wheel_cmd_msg.name.data[1].size = 0;

    wheel_cmd_msg.velocity.data = sub_velocities;
    wheel_cmd_msg.velocity.capacity = 2;
    wheel_cmd_msg.velocity.size = 0;

    wheel_cmd_msg.position.data = sub_positions;
    wheel_cmd_msg.position.capacity = 2;
    wheel_cmd_msg.position.size = 0;

    wheel_cmd_msg.effort.data = sub_efforts;
    wheel_cmd_msg.effort.capacity = 2;
    wheel_cmd_msg.effort.size = 0;

    // --- 4. Initialize ROS 2 Entities ---
    rclc_publisher_init_best_effort(
      &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "imu_msg");

      rclc_publisher_init_best_effort(
      &debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "debug_msg");

    rclc_publisher_init_best_effort(
      &wheel_state_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "stm32/wheel_states");

    rclc_subscription_init_best_effort(
      &wheel_cmd_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "stm32/wheel_commands");

        // 1. Initialize the parameter server
    rclc_parameter_server_init_default(&param_server, &node);

    // 2. Add parameters
    rclc_add_parameter(&param_server, "kf", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "kf", (double)Kf);

    rclc_add_parameter(&param_server, "kp", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "kp", (double)Kp);

    rclc_add_parameter(&param_server, "ki", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "ki", (double)Ki);

    rclc_add_parameter(&param_server, "kd", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&param_server, "kd", (double)Kd);

    rclc_executor_init(&executor, &support.context, 15, &allocator); 
    rclc_executor_add_subscription(&executor, &wheel_cmd_subscriber, &wheel_cmd_msg, &wheel_cmd_callback, ON_NEW_DATA);
    rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);

    uint32_t last_reconnect_attempt = 0;
    const uint32_t RECONNECT_INTERVAL = 2000;

    // --- 5. Main Execution Loop ---
    for(;;)
    {
      rcl_ret_t ret;

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

      if (init_status == true) {
            bool accel_ok = MPU6050_Read_Accel(&hi2c1);
            bool gyro_ok = MPU6050_Read_Gyro(&hi2c1);

            if (!accel_ok || !gyro_ok) {
              init_status = false; 
              debug_print("[ERROR] IMU I2C Read Failed! Bus offline.");
            }
            else {
              imu_msg.x = Ax;
              imu_msg.y = Ay;
              imu_msg.z = Gz;

              ret=rcl_publish(&imu_publisher, &imu_msg, NULL);
              (void)ret;
            }
      }
      else {
        if (HAL_GetTick() - last_reconnect_attempt >= RECONNECT_INTERVAL) {
          last_reconnect_attempt = HAL_GetTick();
          debug_print("[WARN] Attempting IMU I2C bus recovery...");

          HAL_I2C_DeInit(&hi2c1);
          I2C1_Clear_Busy_Flag();
          __HAL_RCC_I2C1_FORCE_RESET();
          osDelay(2); 
          __HAL_RCC_I2C1_RELEASE_RESET();
          MX_I2C1_Init(); 
          
          init_status = MPU6050_Init(&hi2c1);
          
          if (init_status) {
              debug_print("[SUCCESS] IMU Reconnected!");
          } else {
              debug_print("[FAILED] IMU Reconnect failed, retrying in 2s...");
          }
        }
      }



      wheel_state_msg.position.data[0] = (position_L == 0.0f) ? 1e-6 : (double)position_L;
      wheel_state_msg.position.data[1] = (position_R == 0.0f) ? 1e-6 : (double)position_R;

      wheel_state_msg.velocity.data[0] = (double)(motor_rpm_L * 2.0f * PI / 60.0f);
      wheel_state_msg.velocity.data[1] = (double)(motor_rpm_R * 2.0f * PI / 60.0f);


      ret=rcl_publish(&wheel_state_publisher, &wheel_state_msg, NULL);
      (void)ret;

      float v_L = (motor_rpm_L * 2.0f * PI / 60.0f) * WHEEL_RADIUS;
      float v_R = (motor_rpm_R * 2.0f * PI / 60.0f) * WHEEL_RADIUS;

      osDelay(1);
    }
}

