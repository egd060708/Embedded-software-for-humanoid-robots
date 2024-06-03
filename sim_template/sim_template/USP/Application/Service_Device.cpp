 /**
  ******************************************************************************
  * @file   task.cpp
  * @brief  freertos task running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>
  * </table>
  *
 */

/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include <Middlewares/UpperMonitor/UpperMonitor.h>
#include "MF9025_v2.h"
/* Private define ------------------------------------------------------------*/
TaskHandle_t DjiMotor_Handle;		
TaskHandle_t IMU_Handle;		
TaskHandle_t DR16_Handle;
TaskHandle_t Upper_Handle;
Motor_C610 motor2006(3);
Motor_C620 motor35081(1);
Motor_C620 motor35082(2);
Motor_GM6020 motor6020(1); 
myPID speedPID1;
myPID speedPID2;
float debugSpeed_T1 = 0.f;
float debugSpeed_C1 = 0.f;
float debugSpeed_T2 = 0.f;
float debugSpeed_C2 = 0.f;
MotorMF9025v2Classdef mf9025_motor(1);
uint8_t run_count = 0;
/* Private function declarations ---------------------------------------------*/
void tskDjiMotor(void *arg);
void tskIMU(void *arg);
void tskDR16(void *arg);
void tskUpper(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
  xTaskCreate(tskDjiMotor, 	"App.Motor",   Small_Stack_Size, NULL, PriorityAboveNormal, &DjiMotor_Handle);
	#if  USE_SRML_MPU6050
  xTaskCreate(tskIMU,				"App.IMU",	   Small_Stack_Size, NULL, PriorityNormal,      &IMU_Handle);
	#endif
  xTaskCreate(tskDR16, 			"App.DR16",    Small_Stack_Size, NULL, PriorityAboveNormal, &DR16_Handle);
	xTaskCreate(tskUpper,     "App.Upper",		Small_Stack_Size,NULL, PriorityNormal , &Upper_Handle);
}

void tskUpper(void *arg)
{
	
	for(;;)
	{
		vTaskDelay(5);
		Sent_Contorl(&huart1);
	}
}

/**
 * @brief <freertos> 大疆电机控制任务
 */
void tskDjiMotor(void *arg)
{
	/*	pre load for task	*/
	static Motor_CAN_COB Tx_Buff;
	speedPID1.SetPIDParam(5,0,0,0,10000);
	speedPID2.SetPIDParam(5,0,0,0,10000);
	mf9025_motor.init(CAN1_TxPort);
	for(;;){
		/* wait for next circle */
		vTaskDelay(2);
		run_count ++;
		if(run_count > 100){
			run_count = 0;
		}
		/*	电机控制	*/
//		speedPID1.Target = debugSpeed_T1;
//		speedPID1.Current = motor2006.getSpeed();
//		debugSpeed_C1 = motor2006.getSpeed();
//		speedPID1.Adjust();
//		motor2006.Out = speedPID1.Out;
//		
//		speedPID2.Target = debugSpeed_T2;
//		speedPID2.Current = motor35082.getSpeed();
//		debugSpeed_C2 = motor35082.getSpeed();
//		speedPID2.Adjust();
//		motor35082.Out = speedPID2.Out;
//		
//		/*	将电机输出数据打包成can消息队列	*/
		//Tx_Buff = MotorMsgPack(Tx_Buff,motor35082);
//		Tx_Buff = MotorMsgPack(Tx_Buff,motor2006);
////		Tx_Buff =  MotorMsgPack(Tx_Buff,pitchmotor,		//	pitch轴电机
////										yawmotor);		//	yaw轴电机

//		//	发送can队列，根据电机的发射帧id选择需要发送的数据包
		//xQueueSend(CAN1_TxPort,&Tx_Buff.Id200,0);
		//xQueueSend(CAN1_TxPort,&Tx_Buff.Id1ff,0);

		mf9025_motor.speedControl(debugSpeed_T1);
		//Sent_Contorl(&huart1);
	}
}

#if  USE_SRML_MPU6050
/**
 * @brief MPU6050读取数据
 */
void tskIMU(void *arg)
{
	/* Pre-Load for task */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	for(;;){
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);
		/*	读取MPU6050数据	*/
		dmp_read_data(&mpu_receive);
	}
}
#endif

/**
	*	@brief	Dr16 data receive task
	*/
void tskDR16(void *arg)
{
  /* Cache for Task */
  static USART_COB Rx_Package;
  /* Pre-Load for task */
  DR16.Check_Link(xTaskGetTickCount());
  /* Infinite loop */
  for (;;)
  {
    /* Enter critical */
    xSemaphoreTake(DR16_mutex, portMAX_DELAY);
    /*	等待数据	*/
	if (xQueueReceive(DR16_QueueHandle, &Rx_Package, 100) == pdPASS)
	{
	  // Read Message
	  DR16.DataCapture((DR16_DataPack_Typedef*) Rx_Package.address);
	}
	/*	检测遥控器连接 */
    DR16.Check_Link(xTaskGetTickCount());
    /*	判断是否连接 	 */
    if(DR16.GetStatus() != DR16_ESTABLISHED )
    {
    	/**
		 * lost the remote control
		 */

    	/* Leave critical */
    	xSemaphoreGive(DR16_mutex);
		continue;
    }
    /*	更新遥控器控制	*/

    /* Leave critical */
    xSemaphoreGive(DR16_mutex);
  }
}


