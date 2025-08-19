/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : red_test [ifname1] [ifname2] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 500
 *
 * This is a redundancy test.
 *
 * (c)Arthur Ketels 2008
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <inttypes.h>

#include "ethercat.h"

#include <sched.h>
#include <signal.h>
#include "chry_ringbuffer.h"
chry_ringbuffer_t rb;
uint8_t mempool[8192];

#include "udptools.h"


sockfdc fd;

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2,thread3;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int opMode = 0;
int running = 1;

int testMode = 0;


#define ENCODER_17BIT 20860.76 // 2^17 / 2pi
#define ENCODER_17BIT_rpm  2184.53  // 2^17 / 60
float kp=0.5,kd=5, posE,velE;
int sinCnt = 0;
 

#define oversample 1

typedef struct PACKED
{
   uint16 Ch1_CycleCount;
   int16 Ch1_Sample[oversample];
   uint16 Ch2_CycleCount;
   int16 Ch2_Sample[oversample];
   uint32 StartTimeNextLatch;
} el3702_1_in_pdo;

// rxPDO
typedef struct PACKED
{
   uint16 status; //0x60400010
   int16 tTorque; //0x60710010
   int32 tPos;    //0x607A0020
   int32 tVel;    //0x60FF0020
   uint8 opMode;  //0x60600008
  //  uint8 complement;
} youda_out_pdo;
// txPDO
typedef struct PACKED
{
   uint16 status;  //0x60410010
   uint16 error;   //0x603F0010
   int32 Pos;      //0x60640020
   int32 vel;      //0x606C0020
   int16 Torque;   //0x60770010
   uint8 opMode;   //0x60610008
} youda_in_pdo;

 typedef struct PACKED
{
   uint16 status;
   uint8 opMode;
   int32 tPos;
   int32 tVel;
   int16 tTorque;
   int32 Voffset;
   uint8 complement;
} elmo_out_pdo;

typedef struct PACKED
{
   uint16 status;
   uint8 opMode;
   int32 Pos;
   int32 vel;
   int16 Torque;
   uint32 dcV;
   uint16_t error;
   uint8 complement;
} elmo_in_pdo;


uint16_t command[20];
youda_in_pdo *youda_val[20];
youda_out_pdo *youda_target[20];
youda_in_pdo  youda_state[20];
youda_out_pdo youda_CMD[20];

size_t youda_cnt = 0;

elmo_in_pdo *elmo_val[20];
elmo_out_pdo *elmo_target[20];
size_t elmo_cnt = 0;

el3702_1_in_pdo *el3702_1_in_val[20];


uint32 buf32;
uint16 buf16;
uint8 buf8;
 

uint32 set_cycletime = 500*1000;


///////////////////////////////////ros   ///////////////////////////////////
#include <std_msgs/msg/int32.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>


#include "custom_interfaces/msg/driving_motors.h"
#include "custom_interfaces/msg/driving_command.h"
#include "custom_interfaces/msg/basic_command.h"
#include "custom_interfaces/msg/motor_state.h"

rcl_publisher_t my_pub;
rcl_publisher_t motorState_pub;
rcl_subscription_t my_sub;
rcl_subscription_t motorCMD_sub;
std_msgs__msg__Int32 pub_msg;
std_msgs__msg__Int32 sub_msg;
custom_interfaces__msg__DrivingCommand motorCMD_msg;
unsigned int short_timer_counter = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_ret_t rc;
rcl_node_t my_node;
rclc_executor_t executor;
  rcl_timer_t my_timer;

custom_interfaces__msg__DrivingMotors message;

void my_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback: I heard: %d\n", msg->data);
  }
  if (msg->data % 2)
  {
    rclc_sleep_ms(900);
  }
}

void motorCMD_subscriber_callback(const void * msgin)
{
//   const custom_interfaces__msg__DrivingCommand * msg = (const custom_interfaces__msg__DrivingCommand *)msgin;
printf("motorCMD_subscriber_callback\n");
      uint32_t len = chry_ringbuffer_write(&rb, msgin, sizeof(custom_interfaces__msg__DrivingCommand));
         if(len == sizeof(custom_interfaces__msg__DrivingCommand)){
            printf("[P] write %d bytes success\r\n",len);
        }else{
            printf("[P] write faild, only write %d byte\r\n", len);
        }
//   if (msg->data % 2)
//   {
//     rclc_sleep_ms(900);
//   }
}

void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    rc = rcl_publish(&my_pub, &pub_msg, NULL);

    if (rc == RCL_RET_OK) {
      // printf("Published message %d\n", pub_msg.data);
    } else {
      printf("timer_callback: Error publishing message %d\n", pub_msg.data);
    }
    pub_msg.data++;
  } else {
    printf("timer_callback Error: timer parameter is NULL\n");
  }
}

void short_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(timer);
  RCLC_UNUSED(last_call_time);
  printf("shorttimer %d\n",short_timer_counter++);
}

void getMotorState(int motorId, const char * motorName)
{
    // 模拟电机状态数据
    int state = 0;
    if (motorId == 1)
    {
        state = 0x0F;
    }
    else if (motorId == 2)
    {
        state = 0x0F;
    }
    else
    {
        state = 0x06;
    }
    return state;
}

void motorStatePub() {

    //motor pointer
    custom_interfaces__msg__MotorState *motorPtr[4];

    motorPtr[0] = &message.motor1;
    motorPtr[1] = &message.motor2;
    motorPtr[2] = &message.motor3;
    motorPtr[3] = &message.motor4;

    for (size_t i = 0; i < youda_cnt; i++)
    {
        motorPtr[i]->id = i + 1;
        motorPtr[i]->position = youda_state[i].Pos/ENCODER_17BIT;
        motorPtr[i]->velocity = youda_state[i].vel/ENCODER_17BIT;
        motorPtr[i]->torque = youda_state[i].Torque;
        motorPtr[i]->temperature = 25.0f;
        motorPtr[i]->err = youda_state[i].error;
    }
    // // print youda_state
    // for(size_t i = 0; i < youda_cnt; i++)
    // {
    //     printf("slave: %d  state: %5d pos: %9d  vel: %9d  torque: %9d opMode:%d error:%4x \n", i, youda_state[i].status, youda_state[i].Pos, youda_state[i].vel, youda_state[i].Torque, youda_state[i].opMode, youda_state[i].error);
    // }
 

    // 发布电机状态
    // printf("Publishing motor status...\n");

    rc = rcl_publish(&motorState_pub, &message, NULL);
    if (rc != RCL_RET_OK) {
        printf("Error while publishing motor state\n");
    }
}

 
int stopRos()
{
      // clean up
  rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_timer_fini(&my_timer);
  rc += rcl_subscription_fini(&my_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);

  std_msgs__msg__Int32__fini(&pub_msg);
  std_msgs__msg__Int32__fini(&sub_msg);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}
///////////////////////////////////ros   ///////////////////////////////////
 


 
uint16_t getCW2(int32_t nSevoOn , uint16_t u16Sate, uint16_t *u16Ctrol){

	if(u16Sate & 0x08){
		if(*u16Ctrol == 0x80)
			*u16Ctrol = 0;
		else
			*u16Ctrol = 0x80;
		return 0;
	}
	if(nSevoOn==1){
		switch(u16Sate & 0x6F){
		case 0x40:
			*u16Ctrol = 0x06;
			break;
		case 0x21:
			*u16Ctrol = 0x07;
			break;
		case 0x23:
			*u16Ctrol = 0x0F;
			break;
		case 0x27:
			*u16Ctrol = 0x0F;
			// nret = 1;
			break;
		default:
			*u16Ctrol = 0x06;
			break;
		}
	}else{
        *u16Ctrol = 0x00;
    }
}

uint32 buf32;
uint16 buf16;
uint8 buf8;

#define READ(slaveId, idx, sub, buf, comment)                                                                      \
   {                                                                                                               \
      buf = 0;                                                                                                     \
      int __s = sizeof(buf);                                                                                       \
      int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                 \
      printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, \
             (unsigned int)buf, (unsigned int)buf, comment);                                                       \
   }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                          \
   {                                                                                                           \
      int __s = sizeof(buf);                                                                                   \
      buf = value;                                                                                             \
      int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                             \
      printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, \
             (unsigned int)buf, comment);                                                                      \
   }

#define CHECKERROR(slaveId)                                                                                   \
   {                                                                                                          \
      ec_readstate();                                                                                         \
      printf(" EC> \"%s\" %x - %x [%s] ", (char *)ec_elist2string(), ec_slave[slaveId].state,                 \
             ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode)); \
   }

static int zeroError_config(uint16 slave)
{

   ec_dcsync0(slave, TRUE, 500 * 1000, 0);

   WRITE(slave, 0x1C12, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1C13, 0, buf8, 0x00, "disable PDO");

   WRITE(slave, 0x1600, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1600, 1, buf32, 0x60400010, "1 rPDO");
   WRITE(slave, 0x1600, 2, buf32, 0x60600008, "2 rPDO");
   WRITE(slave, 0x1600, 3, buf32, 0x607A0020, "3 rPDO");
   WRITE(slave, 0x1600, 4, buf32, 0x60FF0020, "4 rPDO");
   WRITE(slave, 0x1600, 5, buf32, 0x60710010, "5 rPDO");
   WRITE(slave, 0x1600, 6, buf32, 0x00000008, "6 rPDO");
   WRITE(slave, 0x1600, 0, buf8, 0x06, "rxPDO#");

   WRITE(slave, 0x1A00, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1A00, 1, buf32, 0x60410010, "1 tPDO");
   WRITE(slave, 0x1A00, 2, buf32, 0x60640020, "2 tPDO");
   WRITE(slave, 0x1A00, 3, buf32, 0x606C0020, "3 tPDO");
   WRITE(slave, 0x1A00, 4, buf32, 0x60770010, "4 tPDO");
   WRITE(slave, 0x1A00, 5, buf32, 0x3B690020, "5 tPDO");
   WRITE(slave, 0x1A00, 6, buf32, 0x603F0010, "6 tPDO");
   WRITE(slave, 0x1A00, 0, buf8, 0x06, "txPDO#");

   WRITE(slave, 0x1C12, 1, buf16, 0x1600, "rxPDO");
   WRITE(slave, 0x1C12, 0, buf8, 0x01, "enable PDO");

   WRITE(slave, 0x1C13, 1, buf16, 0x1A00, "txPDO");
   WRITE(slave, 0x1C13, 0, buf8, 0x01, "enable PDO");

   WRITE(slave, 0x6060, 0, buf8, 10, " op mode");
}

static int youda_dc_config(uint16 slave)
{

   ec_dcsync0(slave, TRUE, set_cycletime, 0);
   // ec_dcsync01(slave, TRUE, 1000 * 1000, 000 * 1000, 0); // SYNC0/1 on slave 1

   printf("ec_dcsync0 called on slave %u\n", slave);

   WRITE(slave, 0x1C12, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1C13, 0, buf8, 0x00, "disable PDO");

   WRITE(slave, 0x1601, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1601, 1, buf32, 0x60400010, "1 rPDO");
   WRITE(slave, 0x1601, 2, buf32, 0x60600008, "2 rPDO");
   WRITE(slave, 0x1601, 3, buf32, 0x607A0020, "3 rPDO");
   WRITE(slave, 0x1601, 4, buf32, 0x60FF0020, "4 rPDO");
   WRITE(slave, 0x1601, 5, buf32, 0x60710010, "5 rPDO");
   // WRITE(slave, 0x1601, 6, buf32, 0x00000008, "6 rPDO");
   WRITE(slave, 0x1601, 0, buf8, 0x05, "rxPDO#");
   // WRITE(slave, 0x1608, 0, buf8, 0x00, "rxPDO#");

   WRITE(slave, 0x1A01, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1A01, 1, buf32, 0x60410010, "1 tPDO");
   WRITE(slave, 0x1A01, 2, buf32, 0x60610008, "2 tPDO");
   WRITE(slave, 0x1A01, 3, buf32, 0x60640020, "3 tPDO");
   WRITE(slave, 0x1A01, 4, buf32, 0x606C0020, "4 tPDO");
   WRITE(slave, 0x1A01, 5, buf32, 0x60770010, "5 tPDO");
   WRITE(slave, 0x1A01, 6, buf32, 0x603F0010, "6 tPDO");
   WRITE(slave, 0x1A01, 0, buf8, 0x06, "txPDO#");
   // WRITE(slave, 0x1A08, 0, buf8, 0x00, "txPDO#");

   WRITE(slave, 0x1C12, 1, buf16, 0x1601, "rxPDO");
   WRITE(slave, 0x1C12, 0, buf8, 0x01, "enable PDO");

   WRITE(slave, 0x1C13, 1, buf16, 0x1A01, "txPDO");
   WRITE(slave, 0x1C13, 0, buf8, 0x01, "enable PDO");

   WRITE(slave, 0x6060, 0, buf8, 10, " op mode");

   WRITE(slave, 0x60c2, 1, buf8, 5, "cyc time");
   WRITE(slave, 0x60c2, 2, buf8, -4, "cyc time index");


   WRITE(slave, 0x607f, 0, buf32, 131072*5000, "Max.Profile Velocity");

   // WRITE(slave, 0x10F1, 2, buf32, 2, "Heartbeat");

   return 0;
}



static int youda_dc_config2(uint16 slave)
{

   ec_dcsync0(slave, TRUE, set_cycletime, 0);
   // ec_dcsync01(slave, TRUE, 1000 * 1000, 000 * 1000, 0); // SYNC0/1 on slave 1

   printf("ec_dcsync0 called on slave %u\n", slave);

   WRITE(slave, 0x1C12, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1C13, 0, buf8, 0x00, "disable PDO");

   WRITE(slave, 0x1601, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1601, 1, buf32, 0x60400010, "1 rPDO");
   WRITE(slave, 0x1601, 2, buf32, 0x60710010, "2 rPDO");   // 0x60600008
   WRITE(slave, 0x1601, 3, buf32, 0x607A0020, "3 rPDO");
   WRITE(slave, 0x1601, 4, buf32, 0x60FF0020, "4 rPDO");
   WRITE(slave, 0x1601, 5, buf32, 0x60600008, "5 rPDO");   // 0x60710010
   // WRITE(slave, 0x1601, 6, buf32, 0x00000008, "6 rPDO");
   WRITE(slave, 0x1601, 0, buf8, 0x05, "rxPDO#");
   // WRITE(slave, 0x1608, 0, buf8, 0x00, "rxPDO#");

   WRITE(slave, 0x1A01, 0, buf8, 0x00, "disable PDO");
   WRITE(slave, 0x1A01, 1, buf32, 0x60410010, "1 tPDO");
   WRITE(slave, 0x1A01, 2, buf32, 0x603F0010, "2 tPDO");    // 0x60610008
   WRITE(slave, 0x1A01, 3, buf32, 0x60640020, "3 tPDO");
   WRITE(slave, 0x1A01, 4, buf32, 0x606C0020, "4 tPDO");
   WRITE(slave, 0x1A01, 5, buf32, 0x60770010, "5 tPDO");
   WRITE(slave, 0x1A01, 6, buf32, 0x60610008, "6 tPDO");    //  0x603F0010
   WRITE(slave, 0x1A01, 0, buf8, 0x06, "txPDO#");
   // WRITE(slave, 0x1A08, 0, buf8, 0x00, "txPDO#");

   WRITE(slave, 0x1C12, 1, buf16, 0x1601, "rxPDO");
   WRITE(slave, 0x1C12, 0, buf8, 0x01, "enable PDO");

   WRITE(slave, 0x1C13, 1, buf16, 0x1A01, "txPDO");
   WRITE(slave, 0x1C13, 0, buf8, 0x01, "enable PDO");

   WRITE(slave, 0x6060, 0, buf8, 10, " op mode");

   // WRITE(slave, 0x60c2, 1, buf8, 5, "cyc time");
   // WRITE(slave, 0x60c2, 2, buf8, -4, "cyc time index");


   WRITE(slave, 0x607f, 0, buf32, 131072*5000, "Max.Profile Velocity");

   // WRITE(slave, 0x10F1, 2, buf32, 2, "Heartbeat");

   return 0;
}
 

 
int printCnt = 0;
volatile int received = 0;

void redtest(char *ifname, int argc, const char * argv[])
{


    ////////////////////////////////////////////////////


  allocator = rcl_get_default_allocator();
 

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }

  // create rcl_node
  my_node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&my_node, "ecm_node", "", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }

  // create a publisher to publish topic 'topic_0' with type std_msg::msg::String
  // my_pub is global, so that the timer callback can access this publisher.
 

  const rosidl_message_type_support_t * my_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  const char * topic_name = "DrivingPublisher";
  const rosidl_message_type_support_t * motorState_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, MotorState);

//   rc = rclc_publisher_init_default(
//     &motorState_pub,
//     &my_node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, DrivingMotors),
//     "motorState");

rc = rclc_publisher_init_default(
    &motorState_pub,
    &my_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, DrivingMotors),
    topic_name
);


  if (RCL_RET_OK != rc) {
    printf("Error in rclc_publisher_init_default %s.\n", topic_name);
    return -1;
  }

  // create a timer, which will call the publisher with period=`timer_timeout` ms in the 'my_timer_callback'
//    my_timer = rcl_get_zero_initialized_timer();
//   const unsigned int timer_timeout = 1000;
//   rc = rclc_timer_init_default2(
//     &my_timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     my_timer_callback,
//     true);
//   if (rc != RCL_RET_OK) {
//     printf("Error in rclc_timer_init_default2.\n");
//     return -1;
//   } else {
//     printf("Created timer with timeout %d ms.\n", timer_timeout);
//   }

//   rcl_timer_t short_timer = rcl_get_zero_initialized_timer();
//   const unsigned int short_timer_timeout = 100;
//   rc = rclc_timer_init_default2(
//     &short_timer,
//     &support,
//     RCL_MS_TO_NS(short_timer_timeout),
//     short_timer_callback,
//     true);
//   if (rc != RCL_RET_OK) {
//     printf("Error in rclc_timer_init_default2.\n");
//     return -1;
//   } else {
//     printf("Created timer with timeout %d ms.\n", short_timer_timeout);
//   }

  // assign message to publisher
//   pub_msg.data = 1;

const char * topic_name2 = "DrivingController";
  // create subscription
    motorCMD_sub = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(
    &motorCMD_sub,
    &my_node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(custom_interfaces, msg, DrivingCommand),
    topic_name2);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name2);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name2);
  }

  // initialize subscription message
//   std_msgs__msg__Int32__init(&sub_msg);
  custom_interfaces__msg__DrivingCommand__init(&motorCMD_msg);


  ////////////////////////////////////////////////////////////////////////////
  // Configuration of RCL Executor
  ////////////////////////////////////////////////////////////////////////////
   executor = rclc_executor_get_zero_initialized_executor();
  // total number of handles = #subscriptions + #timers
  // check also xrce-dds configuration for maximum number of publisher, 
  // subscribers, timers etc. 
  // Note:
  // If you need more than the default number of publisher/subscribers, etc., you
  // need to configure the micro-ROS middleware also!
  // See documentation in the executor.h at the function rclc_executor_init()
  // for more details.
  unsigned int num_handles = 1 + 0;
  printf("Debug: number of DDS handles: %u\n", num_handles);
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

  // add subscription to executor
  rc = rclc_executor_add_subscription(
    &executor, &motorCMD_sub, &motorCMD_msg, &motorCMD_subscriber_callback,
    ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_subscription. \n");
  }

//   rclc_executor_add_timer(&executor, &my_timer);
//   if (rc != RCL_RET_OK) {
//     printf("Error in rclc_executor_add_timer.\n");
//   }

//   rclc_executor_add_timer(&executor, &short_timer);
//   if (rc != RCL_RET_OK) {
//     printf("Error in rclc_executor_add_timer.\n");
//   }



    // 创建并填充电机状态消息
    

    // 初始化并分配内存给每个 MotorState
    custom_interfaces__msg__MotorState__init(&message.motor1);
    custom_interfaces__msg__MotorState__init(&message.motor2);
    custom_interfaces__msg__MotorState__init(&message.motor3);
    custom_interfaces__msg__MotorState__init(&message.motor4);

    // 初始化字符串字段并分配内存
    const size_t motor_name_size = 50; // 假设最大长度为50

    // 为 name 字段分配内存，并设置 capacity 大于 size
    message.motor1.name.data = (char *)malloc(motor_name_size);
    message.motor1.name.capacity = motor_name_size;  // 设置 capacity
    message.motor1.name.size = 0;  // 初始化 size 为 0
    snprintf(message.motor1.name.data, motor_name_size, "Motor 1");
    message.motor1.name.size = strlen(message.motor1.name.data);  // 更新 size 为实际大小

    message.motor2.name.data = (char *)malloc(motor_name_size);
    message.motor2.name.capacity = motor_name_size;  // 设置 capacity
    message.motor2.name.size = 0;
    snprintf(message.motor2.name.data, motor_name_size, "Motor 2");
    message.motor2.name.size = strlen(message.motor2.name.data);

    message.motor3.name.data = (char *)malloc(motor_name_size);
    message.motor3.name.capacity = motor_name_size;  // 设置 capacity
    message.motor3.name.size = 0;
    snprintf(message.motor3.name.data, motor_name_size, "Motor 3");
    message.motor3.name.size = strlen(message.motor3.name.data);

    message.motor4.name.data = (char *)malloc(motor_name_size);
    message.motor4.name.capacity = motor_name_size;  // 设置 capacity
    message.motor4.name.size = 0;
    snprintf(message.motor4.name.data, motor_name_size, "Motor 4");
    message.motor4.name.size = strlen(message.motor4.name.data);





    ///////////////////////////////////////////////////

   int cnt, i, j;

   printf("Starting Ecm Motor node\n");

   /* initialise SOEM, bind socket to ifname */
 
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */
      if (ec_config_init(FALSE) > 0)
      {
         printf("ec_slavecount %d \n", ec_slavecount);
         for (cnt = 1; cnt <= ec_slavecount; cnt++)
         {
            // /* Detect slave man:00000002 ID: 0e763052 Name:TC200E TR CoE Drive  */
            // if ((ec_slave[cnt].eep_man == 0x00000002) && (ec_slave[cnt].eep_id == 0x0e763052))
            // {
            //    printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);

            //    /* link slave specific setup to preop->safeop hook */
            //    ec_slave[cnt].PO2SOconfigx = &el3704_dc_config;
            // }

            /* Detect slave man:00075500 ID: 00000001 Name:TC200E TR CoE Drive  */
            if ((ec_slave[cnt].eep_man == 0x00075500) && (ec_slave[cnt].eep_id == 0x00000001))
            {
               printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
               /* link slave specific setup to preop->safeop hook */
               ec_slave[cnt].PO2SOconfig = &youda_dc_config2;
            }

            /* Detect slave man:0x0000005a ID: 00000001 Name:YD100 Servo CoE Drive  */
            if ((ec_slave[cnt].eep_man == 0x0000005a) && (ec_slave[cnt].eep_id == 0x00000001))
            {
               printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
               /* link slave specific setup to preop->safeop hook */
               ec_slave[cnt].PO2SOconfig = &youda_dc_config2;
            }

            /* Detect slave man:0000005a ID: 00000002 Name:YD300 Servo CoE Drive  */   
            if ((ec_slave[cnt].eep_man == 0x0000005a) && (ec_slave[cnt].eep_id == 0x00000002))
            {
               printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
               /* link slave specific setup to preop->safeop hook */
               ec_slave[cnt].PO2SOconfig = &youda_dc_config2;

               printf("Found !!!!\n");
            }

            /* Detect slave man:0x0000005a ID: 00000001 Name:YD300 Servo CoE Drive  PD version*/
            if ((ec_slave[cnt].eep_man == 0x0000005a) && (ec_slave[cnt].eep_id == 0x00000100))
            {
               printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
               /* link slave specific setup to preop->safeop hook */
               ec_slave[cnt].PO2SOconfig = &youda_dc_config2;

               printf("Found !!!!\n");
            }


            /* Detect slave man:0000009a ID: 00030924 Name:elmo Driver  */
            if ((ec_slave[cnt].eep_man == 0x0000009a) && (ec_slave[cnt].eep_id == 0x00030924))
            {
               printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
               /* link slave specific setup to preop->safeop hook */
               ec_slave[cnt].PO2SOconfig = &elmo_dc_config;
            }
         }

         ec_configdc();

         ec_config_map(&IOmap);
         printf("%d slaves found and configured.\n", ec_slavecount);
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

          /* activate cyclic process data */
         dorun = 1;


         /* read indevidual slave state and store in ec_slave[] */
         ec_readstate();
         for (cnt = 1; cnt <= ec_slavecount; cnt++)
         {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                   ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            printf("         Out:%p,%4d In:%p,%4d\n",
                   ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
            /* check for EL2004 or EL2008 */
            if (!digout && ((ec_slave[cnt].eep_id == 0x0af83052) || (ec_slave[cnt].eep_id == 0x07d83052)))
            {
            
               digout = ec_slave[cnt].outputs;
            }
         }
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);

         printf("Request operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* request OP state for all slaves */
         ec_writestate(0);

         // /* activate cyclic process data */
         // dorun = 1;

         /* wait for all slaves to reach OP state */
         ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);

         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");

            for (cnt = 1; cnt <= ec_slavecount; cnt++)
            {
               /* Detect slave man:00000002 ID: 0e763052 Name:EL3702   */
               if ((ec_slave[cnt].eep_man == 0x00000002) && (ec_slave[cnt].eep_id == 0x0e763052))
               {
                  printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
                   el3702_1_in_val[0] = (el3702_1_in_pdo *)(ec_slave[cnt].inputs);
               }

               /* Detect slave man:00075500 ID: 00000001 Name:TC200E TR CoE Drive  */
               if ((ec_slave[cnt].eep_man == 0x00075500) && (ec_slave[cnt].eep_id == 0x00000001))
               {
                  printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
                  youda_val[youda_cnt] = (youda_in_pdo *)(ec_slave[cnt].inputs);
                  youda_target[youda_cnt] = (youda_out_pdo *)(ec_slave[cnt].outputs);
                  youda_cnt++;
               }

               /* Detect slave man:0x0000005a ID: 00000001 Name:YD100 Servo CoE Drive  */
               if ((ec_slave[cnt].eep_man == 0x0000005a) && (ec_slave[cnt].eep_id == 0x00000001))
               {
                  printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
                  youda_val[youda_cnt] = (youda_in_pdo *)(ec_slave[cnt].inputs);
                  youda_target[youda_cnt] = (youda_out_pdo *)(ec_slave[cnt].outputs);
                  youda_cnt++;
               }
               
               /* Detect slave man:0000005a ID: 00000002 Name:YD300 Servo CoE Drive  */   
               if ((ec_slave[cnt].eep_man == 0x0000005a) && (ec_slave[cnt].eep_id == 0x00000002))
               {
                  printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
                  youda_val[youda_cnt] = (youda_in_pdo *)(ec_slave[cnt].inputs);
                  youda_target[youda_cnt] = (youda_out_pdo *)(ec_slave[cnt].outputs);
                  youda_cnt++;
               }

               /* Detect slave man:0000005a ID: 00000002 Name:YD300 Servo CoE Drive  PD version*/   
               if ((ec_slave[cnt].eep_man == 0x0000005a) && (ec_slave[cnt].eep_id == 0x00000100))
               {
                  printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
                  youda_val[youda_cnt] = (youda_in_pdo *)(ec_slave[cnt].inputs);
                  youda_target[youda_cnt] = (youda_out_pdo *)(ec_slave[cnt].outputs);
                  youda_cnt++;
               }



               /* Detect slave man:0000009a ID: 00030924 Name:elmo Driver  */
               if ((ec_slave[cnt].eep_man == 0x0000009a) && (ec_slave[cnt].eep_id == 0x00030924))
               {
                  printf("Found %s at pos %d \n", ec_slave[cnt].name, cnt);
                  elmo_val[elmo_cnt] = (elmo_in_pdo *)(ec_slave[cnt].inputs);
                  elmo_target[elmo_cnt] = (elmo_out_pdo *)(ec_slave[cnt].outputs);
                  elmo_cnt++;

               }
            }
            

            // while (youda_cnt!=4)   // target motor number
            // {
            //     printf("Erroe motors number: %d  ! \n",youda_cnt);
            //     osal_usleep(10000);
            // }
            

            inOP = TRUE;
            /* acyclic loop 5000 x 20ms = 10s */
            float txData[20];
 
            
            while (running == 1)
            {
               if (received == 1)
               {
                 motorStatePub();
                  received = 0;

                  printCnt++;
                  if (printCnt % 100 == 0)
                  {
                     printf("cycle# %5d , Wck %3d, dt %12" PRId64 ", ",
                            dorun, wkc, gl_delta);
                     // print youda_target and youda_val
                     for (size_t i = 0; i < 1; i++)
                     {
                        printf("slave: %d cw: %5d state: %5d pos: %9d vel: %9d taget_torque:%9d || torque: %9d error:%4x ", i, youda_target[i]->status, youda_val[i]->status, youda_val[i]->Pos, youda_val[i]->vel, youda_target[i]->tTorque, youda_val[i]->Torque,youda_val[i]->error);
                     }
     
                     printf("\r");

                     fflush(stdout);
                  }

                  // osal_usleep(20000);

                  // log data
                  txData[0] = gl_delta;
                  txData[1] = toff;
                  // txData[2] = el3702_1_in_val[0]->Ch1_Sample[0] / (32767.0) *500;


                  
                  txData[3] = youda_val[0]->Pos; // 131072.0;
                  txData[4] = youda_val[0]->vel/ENCODER_17BIT_rpm;// ENCODER_17BIT_rpm ;
                  txData[5] = youda_val[0]->Torque; ///1000.0 * 40
                  txData[6] = youda_target[0]->tPos; // 131072.0;
                  txData[7] = youda_target[0]->tVel;// ENCODER_17BIT_rpm ;
                  txData[8] = youda_target[0]->tTorque; ///1000.0 * 40

                  txData[2] = youda_val[0]->status;      // status word
                  txData[9] = youda_target[0]->status;  ///control word

    
 

                  updSendArray(&fd, txData, sizeof(txData) / 4);
               }
                // osal_usleep(1);
                rc = rclc_executor_spin_some(&executor, 1000  );
            }
            printf("\r\n");
            fflush(stdout);
            printf(" stop running \n");
            for (int cnt = 0; cnt < ec_slavecount; cnt++)
            {
               youda_target[cnt]->opMode = 0;
               youda_target[cnt]->status = 0;
               youda_target[cnt]->tTorque = 0;
            }
            osal_usleep(20000);
            dorun = 0;
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (i = 1; i <= ec_slavecount; i++)
            {
               if (ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
         printf("Request safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End redundant test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", ifname);
   }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if (ts->tv_nsec >= NSEC_PER_SEC)
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}
void precise_sleep(struct timespec *target_time) {
    struct timespec current_time, sleep_time;
    long sleep_ns;

    // Get current time
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    // Calculate the time to sleep (target_time - 20us)
    sleep_time.tv_sec = target_time->tv_sec;
    sleep_ns = target_time->tv_nsec - 20000; // 20us = 20000ns
    if (sleep_ns < 0) {
        sleep_time.tv_sec -= 1;
        sleep_ns += 1000000000;
    }
    sleep_time.tv_nsec = sleep_ns;

    // Sleep until the calculated time
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sleep_time, NULL);

    // Busy wait until the target time
    do {
        clock_gettime(CLOCK_MONOTONIC, &current_time);
    } while ((current_time.tv_sec < target_time->tv_sec) ||
             (current_time.tv_sec == target_time->tv_sec && current_time.tv_nsec < target_time->tv_nsec));
}
/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if (delta > (cycletime / 2))
   {
      delta = delta - cycletime;
   }
   if (delta > 0)
   {
      integral++;
   }
   if (delta < 0)
   {
      integral--;
   }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
}


int16 currentLimit = 0;
float freq = 0;
float direction = 1;

char readBuffer[1024];
/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(0, &cpuset); // Set CPU core

   int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
   if (rc != 0)
   {
      // Handle error
   }

   struct timespec ts, tleft;
   int ht;
   int64 cycletime;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   if (ts.tv_nsec >= NSEC_PER_SEC)
   {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }
   cycletime = *(int *)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   int i = 0;
   //   ec_send_processdata();
   float cycleCnt = 1e9 / (float)cycletime;
   printf("cycleCnt %f \r\n",cycleCnt);
 
   
   while (1)
   {
      if (dorun > 0)
      {
          if (inOP )
         {
             
            uint32_t len = chry_ringbuffer_read(&rb, readBuffer, sizeof(custom_interfaces__msg__DrivingCommand));
            if(len){
                custom_interfaces__msg__DrivingCommand *MotorCMDPtr=readBuffer;
                printf("Received %d bytes from ringbuffer\n", len);
                for (size_t i = 0; i < youda_cnt; i++) {
                        getCW2(1, youda_val[i]->status, &youda_target[i]->status);

                        youda_target[i]->opMode = MotorCMDPtr->drive_mode;
                        youda_target[i]->tPos = MotorCMDPtr->actuator_command[i].position*ENCODER_17BIT;
                        youda_target[i]->tVel = MotorCMDPtr->actuator_command[i].velocity*ENCODER_17BIT;
                        youda_target[i]->tTorque = MotorCMDPtr->actuator_command[i].effort;
                    }
            }
 
         }

         ec_send_processdata();
         wkc = ec_receive_processdata(EC_TIMEOUTRET);

         dorun++;
         if (inOP)
         {
            for (size_t i = 0; i < youda_cnt; i++) {
             memcpy(&youda_state[i], youda_val[i], sizeof(youda_in_pdo));
            }

            if (printCnt > 10000)
            {
               i++;
            }
            
           
  
            // for (size_t cnt = 0; cnt < youda_cnt; cnt++)
            // {
            //    youda_target[cnt]->opMode = 9;
              
            //    getCW2(1, youda_val[cnt]->status, &youda_target[cnt]->status);
       

            //     youda_target[cnt]->tTorque =  sin(i/200.0)*currentLimit/2*direction;

            //     youda_target[cnt]->tPos =  sin(i/2000.0)*3.1415*ENCODER_17BIT*5;

            //     youda_target[cnt]->tVel = 200.0*cos(i / 4000.)*ENCODER_17BIT_rpm;

            //     youda_target[cnt]->tTorque = 50;
 

            //       if (youda_target[cnt]->tTorque > currentLimit)
            //       {
            //          youda_target[cnt]->tTorque = currentLimit;
            //          direction = -direction;
            //       }

            //       if (youda_target[cnt]->tTorque < -currentLimit)
            //       {
            //          youda_target[cnt]->tTorque = -currentLimit;
            //          direction = -direction;
            //       }

            // }


            received = 1;
         }

         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
         }
         // ec_send_processdata();
      }

      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      // clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      precise_sleep(&ts);
   }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
   int slave;

   (void)ptr;

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
         if (needlf)
         {
            needlf = FALSE;
            printf("\n");
         }
         /* one ore more slaves are not responding */
         ec_group[currentgroup].docheckstate = FALSE;
         ec_readstate();
         for (slave = 1; slave <= ec_slavecount; slave++)
         {
            if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
            {
               ec_group[currentgroup].docheckstate = TRUE;
               if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                  ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         if (!ec_group[currentgroup].docheckstate)
            printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
   }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
   cpu_set_t mask;
   CPU_ZERO(&mask);
   CPU_SET(0, &mask);

   int result = sched_setaffinity(0, sizeof(mask), &mask);

   updinit(&fd, "10.16.35.210", 8888); // init udp tools to record data

    if(0 == chry_ringbuffer_init(&rb, mempool, 8192)){
        printf("chry_ringbuffer_init success\r\n");
    }else{
        printf("error\r\n");
    }


   int ctime;

   
 
      dorun = 0;
      ctime = 500;
      set_cycletime = ctime*1000;
  
      testMode = 1;
      freq = 10;
      currentLimit = 1000;
  

      
      /* create RT thread */
      osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *)&ctime);

      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

       //osal_thread_create(&thread3, stack64k * 4, &ecatcheck, NULL);

      /* start acyclic part */
      char *rtif="enp3s0";
      redtest(rtif,  argc,  argv);
//   rosInit( argc,  argv);

   printf("End program\n");

   return (0);
}
