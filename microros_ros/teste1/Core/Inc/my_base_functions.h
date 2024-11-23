/*
 * my_base_functions.h
 *
 *  Created on: Nov 23, 2024
 *      Author: ivan
 */

#ifndef INC_MY_BASE_FUNCTIONS_H_
#define INC_MY_BASE_FUNCTIONS_H_


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string.h>

#include <time.h>

#include <sensor_msgs/msg/imu.h>             // for the encoder msg
#include <geometry_msgs/msg/twist.h>                 // for the motors control
#include <std_msgs/msg/int32.h>
#include <actuator_msgs/msg/actuators.h>

extern osThreadId_t EscreverSetpoinHandle;
extern UART_HandleTypeDef hlpuart1;
/* Subscriber declaration */
rcl_subscription_t imu_sub;

/* Publisher declaration */
rcl_publisher_t velocity_pub;

/* ROS timer declaration */
rcl_timer_t timer;

/* Messages declaration */
actuator_msgs__msg__Actuators velocity_msg;
sensor_msgs__msg__Imu imu_msg;

/* Microros global variables*/
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Minhas funções.
void vSetActuatorMsg(float *);
void vSendActuatorMsg();
void vImuCallback(const void * msgin);
void vMicrorosConfiguration();
void vCreateNode();
void vCreatePublisher();
void vCreateSubscriber();
void vCreateExecutor();


// Funções nativas.
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

/*
 * Função responsável por definir o formato da mensagem do atuator.
 * Essa mensagem tem um header composoto por
	 * header : { stamp: {sec:0,nanosec:0}, frame_id:''}
	 * position: []
	 * velocity: []
	 * normalized: []
 * Só nos interessa a velocidade. Então o argumento da função é um ponteiro de float com 4 posições
 */

void vSetActuatorMsg(float *fpVelocity){
	velocity_msg.header.frame_id.capacity = 20;
	velocity_msg.header.frame_id.data = (char*) pvPortMalloc(velocity_msg.header.frame_id.capacity  * sizeof(char));
	velocity_msg.header.frame_id.size = strlen(velocity_msg.header.frame_id.data);

	velocity_msg.velocity.capacity = 5;
	velocity_msg.velocity.data = (double*) pvPortMalloc(velocity_msg.velocity.capacity * sizeof(double));
	velocity_msg.velocity.data[0] = fpVelocity[0];
	velocity_msg.velocity.data[1] = fpVelocity[1];
	velocity_msg.velocity.data[2] = fpVelocity[2];
	velocity_msg.velocity.data[3] = fpVelocity[3];
	velocity_msg.velocity.data[4] = imu_msg.linear_acceleration.z;
	velocity_msg.velocity.size = 5;
}

void vSendActuatorMsg(){
	rcl_ret_t ret = rcl_publish(&velocity_pub, &velocity_msg, NULL);
}

void vImuCallback(const void * msgin)
{
	//const sensor_msgs__msg__Imu * minha_msg;
	if (msgin != NULL)
	{

		// Indica que houve leitura da IMU para a tarefa escrever setpoint
		osThreadFlagsSet(EscreverSetpoinHandle, 0x01);
	}

}

void vMicrorosConfiguration(){
	rmw_uros_set_custom_transport(
	true,
	(void *) &hlpuart1,
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
	// Initialize and modify options (Set DOMAIN ID to 25)
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_init_options_init(&init_options, allocator);
	rcl_init_options_set_domain_id(&init_options, 25);
	// Initialize rclc support object with custom options
	rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
}

void vCreateNode(){
	// Create node object

	const char * node_name = "test_node";
	// Node namespace (Can remain empty "")
	const char * namespace = "";
	// Init node with configured support object
	rcl_ret_t rc2 = rclc_node_init_default(&node, node_name, namespace, &support);
	while (rc2 != RCL_RET_OK) {
		for(int i=0;i<10;i++){
			HAL_GPIO_TogglePin(LD2_GPIO_Port , LD2_Pin);
			osDelay(500);
		}
		rc2 = rclc_node_init_default(&node, node_name, namespace, &support);
	}
}

void vCreatePublisher(){
	rclc_publisher_init_default(
			&velocity_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs,msg,Actuators),
			"/X3/gazebo/command/motor_speed");
	 float a_velocity[] = {500,500,500,500};
	 vSetActuatorMsg(a_velocity);
}
void vCreateSubscriber(){
	 const char * imu_topic_name = "/drone/imu";
	// Get message type support
	const rosidl_message_type_support_t * imu_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu);
	// Initialize a reliable subscriber
	rcl_ret_t rc_imu = rclc_subscription_init_default(
	  &imu_sub, &node,
	  imu_type_support, imu_topic_name);
	if (RCL_RET_OK != rc_imu) {
		for(int i=0;i<50;i++){
			HAL_GPIO_TogglePin(LD2_GPIO_Port , LD2_Pin);
			osDelay(500);
		}
	}
}
void vCreateExecutor(){
	 // Create a timer
	const unsigned int timer_timeout = 1000;
	rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), vSendActuatorMsg,true);
	// Create executor
	rclc_executor_init(&executor, &support.context, 2, &allocator);
	rclc_executor_add_subscription(&executor, &imu_sub, &imu_msg,
	  &vImuCallback, ON_NEW_DATA); // ON_NEW_DATA does not work properly
	rclc_executor_add_timer(&executor, &timer);
}



#endif /* INC_MY_BASE_FUNCTIONS_H_ */
