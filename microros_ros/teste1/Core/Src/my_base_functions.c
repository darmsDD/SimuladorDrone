///*
// * my_base_functions.c
// *
// *  Created on: Nov 23, 2024
// *      Author: ivan
// */
//
//
#include "my_base_functions.h"


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


const int iNumberOfTries = 1;
rcl_ret_t i32PubMessageState = RCL_RET_OK;


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
	velocity_msg.velocity.data[0] = fpVelocity[0];
	velocity_msg.velocity.data[1] = fpVelocity[1];
	velocity_msg.velocity.data[2] = fpVelocity[2];
	velocity_msg.velocity.data[3] = fpVelocity[3];
	velocity_msg.velocity.data[4] = imu_msg.linear_acceleration.z;
}


void vFirstSetActuatorMsg(float *fpVelocity){
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
	i32PubMessageState = rcl_publish(&velocity_pub, &velocity_msg, NULL);
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


void vMyMicroros(){
	if(i32MicrorosConfiguration()==RCL_RET_OK && i32CreateNode()==RCL_RET_OK && rmw_uros_sync_session(1000) == RMW_RET_OK &&
			i32CreatePublisher()==RCL_RET_OK && i32CreateSubscriber()==RCL_RET_OK && i32CreateExecutor()==RCL_RET_OK){
		// Run executor
		rcl_ret_t ret = rclc_executor_spin_some(&executor,1000*(1000*1000));
		//rclc_executor_spin(&executor);
		while(RCL_RET_OK==ret){

			if(i32PubMessageState!=RCL_RET_OK){
				break;
			}
			ret = rclc_executor_spin_some(&executor,1000*(500*1000));
		}

	}
	rcl_ret_t rc;
	rc = rclc_executor_fini(&executor);
	rc += rcl_publisher_fini(&velocity_pub, &node);
	rc += rcl_timer_fini(&timer);
	rc += rcl_subscription_fini(&imu_sub, &node);
	rc += rcl_node_fini(&node);
	rc += rclc_support_fini(&support);
	i32PubMessageState=RCL_RET_OK;
	vFastBlinkOnError();
	return;
}


rcl_ret_t i32MicrorosConfiguration(){
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

	int i=0;
	rcl_ret_t ret;
	ret = rcl_init_options_init(&init_options, allocator);
	while (ret != RCL_RET_OK && i<iNumberOfTries){
		ret = rcl_init_options_init(&init_options, allocator);
		i++;
	}
	if(ret != RCL_RET_OK){return ret;}


	i=0;
	ret = rcl_init_options_set_domain_id(&init_options, 25);
	while (ret != RCL_RET_OK && i<iNumberOfTries){
		ret = rcl_init_options_set_domain_id(&init_options, 25);
		i++;
	}
	if(ret != RCL_RET_OK){return ret;}

	// Initialize rclc support object with custom options
	i=0;
	ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
	while (ret != RCL_RET_OK && i<iNumberOfTries){
		ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
		i++;
	};
	return ret;
}

rcl_ret_t i32CreateNode(){
	// Create node object

	const char * node_name = "test_node";
	// Node namespace (Can remain empty "")
	const char * namespace = "";
	int i=0;
	// Init node with configured support object
	rcl_ret_t rc2 = rclc_node_init_default(&node, node_name, namespace, &support);
	while(rc2 != RCL_RET_OK && i<iNumberOfTries){
		rc2 = rclc_node_init_default(&node, node_name, namespace, &support);
		i++;
	}
	return rc2;
}

rcl_ret_t i32CreatePublisher(){
	rcl_ret_t ret = rclc_publisher_init_default(
			&velocity_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs,msg,Actuators),
			"/X3/gazebo/command/motor_speed"
		);
	int i=0;
	while (ret != RCL_RET_OK && i<iNumberOfTries){
		ret = rclc_publisher_init_default(
					&velocity_pub,
					&node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs,msg,Actuators),
					"/X3/gazebo/command/motor_speed"
			   );
		i++;
	};

	 float a_velocity[] = {500,500,500,500};
	 vFirstSetActuatorMsg(a_velocity);
	 return ret;
}

rcl_ret_t i32CreateSubscriber(){
	 const char * imu_topic_name = "/drone/imu";
	// Get message type support
	const rosidl_message_type_support_t * imu_type_support =
	  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu);
	// Initialize a reliable subscriber
	rcl_ret_t rc_imu = rclc_subscription_init_default(
			&imu_sub, &node,
			imu_type_support, imu_topic_name
		);
	int i=0;
	while(RCL_RET_OK != rc_imu && i<iNumberOfTries){
		rc_imu = rclc_subscription_init_default(
			&imu_sub, &node,
			imu_type_support, imu_topic_name
		);
		i++;

	}
	return rc_imu;
}

rcl_ret_t i32CreateExecutor(){
	 // Create a timer
	const unsigned int timer_timeout = 1000;
	rcl_ret_t ret;
	ret = rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), vSendActuatorMsg,true);
	if(ret != RCL_RET_OK){ return ret;}

	// Create executor
	ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
	if(ret != RCL_RET_OK){ return ret;}

	ret = rclc_executor_add_subscription(&executor, &imu_sub, &imu_msg,
			  &vImuCallback, ON_NEW_DATA); // ON_NEW_DATA does not work properly
	if(ret != RCL_RET_OK){ return ret;}


	ret = rclc_executor_add_timer(&executor, &timer);

	return ret;
}

void vFastBlinkOnError(){
	for(int i=0;i<50;i++){
		HAL_GPIO_TogglePin(LD2_GPIO_Port , LD2_Pin);
		osDelay(100);
	}
}
