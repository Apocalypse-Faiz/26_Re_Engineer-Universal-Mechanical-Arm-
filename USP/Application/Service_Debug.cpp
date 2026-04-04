/**
 **********************************************************************************
 * @file   : Service_Debug.cpp
 * @brief  : Debug support file.This file provides access ports to debug.
 **********************************************************************************
 *
 **/
/* Includes ------------------------------------------------------------------*/
#include "internal.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t UpperMonitor_Handle;
TaskHandle_t VofaMonitor_Handle;
TaskHandle_t AsuwaveMonitor_Handle;
/* Private function declarations ---------------------------------------------*/
void Task_UpperMonitor(void *arg);
void Task_VofaMonitor(void *arg);
void Task_AsuwaveMonitor(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialize debug service based on Asuwave.
 */
void Service_Debug_Init(void)
{
#if USE_SRML_UPPER_MONITOR
	xTaskCreate(Task_UpperMonitor, "tskUpperMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &UpperMonitor_Handle);
#endif

#if USE_SRML_VOFA_MONITOR
	xTaskCreate(Task_VofaMonitor, "tskVofaMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &VofaMonitor_Handle);
#endif

#if USE_SRML_ASUWAVE_MONITOR
	xTaskCreate(Task_AsuwaveMonitor, "tskAsuwaveMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &AsuwaveMonitor_Handle);		
#endif
}

#if USE_SRML_UPPER_MONITOR
void Task_UpperMonitor(void *arg)
{
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	UpperMonitor::init(4);// 开启上位机修改变量功能，与发送无关，不使用也能照常发送
	// 分配编号给被修改的变量
	// UpperMonitor::bind_Modified_Var(0, ...);
	// UpperMonitor::bind_Modified_Var(1, ...);
	// UpperMonitor::bind_Modified_Var(2, ...);
	// UpperMonitor::bind_Modified_Var(3, ...);
	// UpperMonitor::bind_Modified_Var(4, ...);

	/* Infinite loop */
	for (;;)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		/* 在此处传入需要观察的变量，第一个参数为通道的起始编号 */
		// UpperMonitor::setDatas(0, mpu_receive.pitch, mpu_receive.yaw, mpu_receive.roll);
		//UpperMonitor::setDatas(3, data3, data4, data5);
		//UpperMonitor::setDatas(6, data6, data7, data8, data9);
		/* 选择串口id */
		UpperMonitor::send(4);
	}
}
#endif /* USE_SRML_UPEER_MONITOR */

#if USE_SRML_VOFA_MONITOR
void Task_VofaMonitor(void *arg){
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	while(1)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		/* 在此处传入需要观察的变量，第一个参数为通道的起始编号 */
		VofaMonitor::setDatas(0, mpu_receive.pitch, mpu_receive.yaw, mpu_receive.roll);
		//VofaMonitor::setDatas(3, data3, data4, data5);
		//VofaMonitor::setDatas(6, data6, data7, data8, data9);
		/* 选择串口id */
		VofaMonitor::send(4);
	}
}
#endif	/* USE_SRML_VOFA_MONITOR */

#if USE_SRML_ASUWAVE_MONITOR	
void Task_AsuwaveMonitor(void *arg){
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	AsuwaveMonitor::init(1, xTaskGetTickCount);

	/* Infinite loop */
	while(1)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		AsuwaveMonitor::send(); // 发送数据给上位机
	}
}
#endif /* USE_SRML_ASUWAVE_MONITOR */
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
