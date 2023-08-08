/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "rosidl_runtime_c/message_type_support_struct.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include "geometry_msgs/msg/twist.h"
#include <geometry_msgs/msg/vector3.h>

#include "usart.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARRAY_LEN 200

#define gear_ratio 10
#define wheel_separation 0.50
#define wheel_radius 0.1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float angular_velocity_left_con = 0.0;
float angular_velocity_right_con = 0.0;

float angular_velocity_left = 0.0;
float angular_velocity_right = 0.0;

int direction_right;
int direction_left;

extern float wheel_angular_velocity_right;
extern float wheel_angular_velocity_left;

rcl_ret_t rc;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

geometry_msgs__msg__Vector3 vector3_subscribe;
geometry_msgs__msg__Vector3 vector3_publish;
geometry_msgs__msg__Twist twist_msg;

/* USER CODE END Variables */
/* Definitions for microros */
osThreadId_t microrosHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t microros_attributes = {
  .name = "microros",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for control_task */
osThreadId_t control_taskHandle;
const osThreadAttr_t control_task_attributes = {
  .name = "control_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Twist_queue */
osMessageQueueId_t Twist_queueHandle;
const osMessageQueueAttr_t Twist_queue_attributes = {
  .name = "Twist_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;

    vector3_subscribe.x = twist_msg->linear.x;
    vector3_subscribe.z = twist_msg->angular.z;

    osMessageQueuePut(Twist_queueHandle, &vector3_subscribe, 0, 0);
}

void twist_publish_callback(rcl_timer_t * timer, int64_t last_call_time){
	(void) last_call_time;
	if (timer != NULL){
		// Update the robot position and orientation based on the wheel movements:
		vector3_publish.x = (wheel_angular_velocity_right + wheel_angular_velocity_left) / 2;
		vector3_publish.z = (wheel_angular_velocity_right - wheel_angular_velocity_left) / wheel_separation;
		rc = rcl_publish(&publisher, &vector3_publish, NULL);
	}
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControl_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Twist_queue */
  Twist_queueHandle = osMessageQueueNew (16, 24, &Twist_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of microros */
  microrosHandle = osThreadNew(StartDefaultTask, NULL, &microros_attributes);

  /* creation of control_task */
  control_taskHandle = osThreadNew(StartControl_task, NULL, &control_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

	  // micro-ROS configuration
	  char test_array[ARRAY_LEN];
	  memset(test_array,'z',ARRAY_LEN);

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

	  // micro-ROS App //
	  // Initialize micro-ROS allocator
	  rcl_allocator_t allocator;
	  allocator = rcl_get_default_allocator();

	  // Initialize support object
	  rclc_support_t support;
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // Create node object
	  rcl_node_t node;
	  rclc_node_init_default(&node, "stm32f446re_node", "", &support);

	  // Create publisher
	  const char * pub_topic_name = "/publish_twist";
	  const rosidl_message_type_support_t * pub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
	  rclc_publisher_init_default(&publisher, &node, pub_type_support, pub_topic_name);

	  // Create timer
	  rcl_timer_t timer;
	  rclc_timer_init_default(
			&timer,
			&support,
			RCL_MS_TO_NS(100),
			twist_publish_callback);

	  // Create subscriber
	  const char * sub_topic_name = "/cmd_vel";
	  const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
	  rclc_subscription_init_default(&subscriber, &node, sub_type_support, sub_topic_name);

	  // Create executor
	  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	  rclc_executor_init(&executor, &support.context, 3, &allocator);
	  rclc_executor_add_timer(&executor, &timer);
	  rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);

	  // Spin executor to receive messages
	  rclc_executor_prepare(&executor);
	  rclc_executor_spin(&executor);

	  // cleaning Up
	  rc += rcl_subscription_fini(&subscriber, &node);
	  rc += rcl_node_fini(&node);

  /* Infinite loop */
//  for(;;)
//  {
//
//  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControl_task */
/**
* @brief Function implementing the Control_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControl_task */
void StartControl_task(void *argument)
{
  /* USER CODE BEGIN StartControl_task */
	geometry_msgs__msg__Vector3 twist_msg;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(Twist_queueHandle, &twist_msg, NULL, osWaitForever);
	  angular_velocity_left_con = (twist_msg.x -  (twist_msg.z * wheel_separation) / 2.0) / wheel_radius;
	  angular_velocity_right_con = (twist_msg.x +  (twist_msg.z * wheel_separation) / 2.0) / wheel_radius;

	// Direction of stepper motor left
	if (angular_velocity_left_con >= 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
		direction_left = 1;
		angular_velocity_left = gear_ratio * angular_velocity_left_con;

	}else if (angular_velocity_left_con < 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
		direction_left = 0;
		angular_velocity_left = -1.0 * gear_ratio * angular_velocity_left_con;
	}

	// Direction of stepper motor right
	if (angular_velocity_right_con >= 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		direction_right = 1;
		angular_velocity_right = gear_ratio * angular_velocity_right_con;

	}else if (angular_velocity_right_con < 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		direction_right = 0;
		angular_velocity_right = -1.0 * gear_ratio * angular_velocity_right_con;
	}
	osDelay(1);
  }
  /* USER CODE END StartControl_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

