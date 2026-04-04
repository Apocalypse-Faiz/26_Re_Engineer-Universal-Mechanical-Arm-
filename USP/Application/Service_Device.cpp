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
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t DjiMotor_Handle;
TaskHandle_t IMU_Handle;
TaskHandle_t DR16_Handle;
TaskHandle_t VT13_Handle; // VT13遥控器任务，如果不用了可以删除掉
TaskHandle_t FS_I6X_Handle;
TaskHandle_t Rx_Referee_Handle;
TaskHandle_t Log_Handle;
/* Private function declarations ---------------------------------------------*/
void tskDjiMotor(void *arg);
void tskFS_I6X(void *arg);
void tskIMU(void *arg);
void tskDR16(void *arg);
void tskVT13(void *arg); // VT13遥控器任务，如果不用了可以删除掉
void Rx_Referee(void *arg);
void tskLog(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialization of device management service
 * @param  None.
 * @return None.
 */
void Service_Devices_Init(void)
{
	xTaskCreate(tskDjiMotor, "App.Motor", Small_Stack_Size, NULL, PriorityNormal, &DjiMotor_Handle);

#if USE_SRML_MPU6050
	xTaskCreate(tskIMU, "App.IMU", Normal_Stack_Size, NULL, PrioritySuperHigh, &IMU_Handle);
#endif

#if USE_SRML_DR16
	xTaskCreate(tskDR16, "App.DR16", Small_Stack_Size, NULL, PrioritySuperHigh, &DR16_Handle);
#endif

#if USE_SRML_VT13
	xTaskCreate(tskVT13, "App.VT13", Normal_Stack_Size, NULL, PrioritySuperHigh, &VT13_Handle);
#endif

#if USE_SRML_LOG
	xTaskCreate(tskLog, "APP.Log", Normal_Stack_Size, NULL, PriorityNormal, &Log_Handle);
#endif

#if USE_SRML_FS_I6X
	xTaskCreate(tskFS_I6X, "App.FS_I6X", Small_Stack_Size, NULL, PrioritySuperHigh, &FS_I6X_Handle);
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
	xTaskCreate(Rx_Referee, "Rx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Rx_Referee_Handle);
#endif
}

/**
 * @brief <freertos> 大疆电机控制任务
 */
void tskDjiMotor(void *arg)
{
	/*	pre load for task	*/
	Motor_CAN_COB Tx_Buff = {};

	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 1);

		/* dji电机库使用示例 */
		// 1. 电机控制
		m2006.Out = 1000;
		// 2. 将电机输出数据打包成can消息队列
		motor_dji::MotorMsgPack(Tx_Buff, m2006, m3508, gm6020);
		// 3. 发送can队列，根据电机的发射帧id选择需要发送的数据包
		xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);
		// 也可以2、3合并用MotorCanSend(),但只能发送一个can包,不同can包的电机不能一次性全丢进去，注意加命名空间
		motor_dji::MotorMsgSend(CAN1_TxPort, gm6020);

		/* 抽象电机库使用示例 */
		// 1. 电机控制
		absM2006.setMotorCurrentOut(1000);
		absM3508.setMotorCurrentOut(1000);
		absM6020[0].setMotorCurrentOut(1000);
		absM6020[0].setMotorCurrentOut(1000);
		// 2. 将电机输出数据打包成can消息队列
		MotorMsgPack(Tx_Buff, absM2006, absM3508, absM6020);
		// 3. 发送can队列，根据电机的发射帧id选择需要发送的数据包
		xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);
		// 也可以2、3合并用MotorCanSend(),但只能发送一个can包，且抽象电机无法用数组发送
		MotorMsgSend(CAN1_TxPort, absM2006);
	}
}

#if USE_SRML_MPU6050
/**
 * @brief MPU6050读取数据
 */
void tskIMU(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 2);
		/*	读取MPU6050数据	*/
		vTaskSuspendAll();		  // 挂起其他任务，防止被打断
		taskDISABLE_INTERRUPTS(); // 关闭中断，若使用中断关闭，请确保SRML定时器的中断不受FreeRTOS管辖
		dmp_read_data(&mpu_receive);
		taskENABLE_INTERRUPTS();
		xTaskResumeAll();
	}
}
#endif

#if USE_SRML_LOG
/**
 *	@brief	Log data transmit task
 */
void tskLog(void *arg)
{
	/*Pre-Load for task*/
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, (pdMS_TO_TICKS(100)));
		SRML_LOGI("MCU", "test log from [%s]", __func__); // 测试Log功能，你也可以在其他任务中使用Log
	}
}
#endif

#if USE_SRML_DR16
/**
 *	@brief	Dr16 data receive task
 */
void tskDR16(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
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
			DR16.DataCapture((DR16_DataPack_Typedef *)Rx_Package.address);
		}
		/*	检测遥控器连接 */
		DR16.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (DR16.GetStatus() != DR16_ESTABLISHED)
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
#endif

#if USE_SRML_VT13
/**
 * @brief  VT13接收任务
 * @param  None.
 * @return None.
 */
void tskVT13(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	VT13.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		// 若要设置进入临界区，请确认好各个遥控器（倘若同时使用两个遥控器）的优先级，防止进入死锁
		xSemaphoreTake(VT13_mutex, portMAX_DELAY);
		/*	等待数据	*/
		if (xQueueReceive(VT13_QueueHandle, &Rx_Package, 100) == pdPASS)
		{
			// Receive Message
			if (Rx_Package.len < 30) // 图传链路下有不同的包，包长不一，做好处理
			{
				VT13.Receive(Rx_Package.address, Rx_Package.len);
			}
		}
		/*	检测遥控器连接 */
		VT13.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (VT13.GetStatus() != VT13_ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */
			xSemaphoreGive(VT13_mutex);
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
		xSemaphoreGive(VT13_mutex);
	}
}
#endif

#if USE_SRML_REFEREE
/**
 * @brief  接受裁判系统数据
 * @param  None.
 * @return None.
 */
void Rx_Referee(void *arg)
{
	/* Preoad for task */
	USART_COB *referee_pack;
	TickType_t xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *)&referee_pack, portMAX_DELAY) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t *)referee_pack->address, referee_pack->len);
		}
	}
}
#endif

#if USE_SRML_FS_I6X
/**
 *	@brief	FS_I6X data receive task
 */
void tskFS_I6X(void *arg)
{
	/* Cache for Task */
	/* Pre-Load for task */
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		/*	等待数据	*/
		if (ulTaskNotifyTake(pdTRUE, 100) != 0)
		{
			// Read Message
			remote.DataProcess();
		}
		/*	检测遥控器连接 */
		remote.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (remote.GetStatus() != ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */;
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
	}
}
#endif // !USE_SRML_FS_I6X

#if USE_SRML_BMI088
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case BMI088_ACC_INT_Pin:
		/** TODO:  25赛季完全删除注释部分，停止对老版BMI解算的支持*/
		// BMI088.AccReadOut();
		// BMI088_AHRS.feed_acc(BMI088.AccData);
		// BMI088_AHRS.doAcc();
		BMI088_AccHandler();
		break;
	case BMI088_GYRO_INT_Pin:
		/** TODO:  25赛季完全删除注释部分，停止对老版BMI解算的支持*/
		// BMI088.GyroReadout();
		// BMI088_AHRS.feed_gyro(BMI088.GyroData);
		// BMI088_AHRS.doGYro();
		BMI088_GyroHandler();
		break;
	default:
		break;
	}
}
#endif