 /*
 * internal.h
 *
 *  Created on: Apr 7, 2021
 *      Author: M3chD09
 */

#ifndef _INTERNAL_H_
#define _INTERNAL_H_

#include "can.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"

#include "SRML.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#if USE_SRML_LOG
#include "Drivers/Devices/Log/inc/srml_log.h"
#include "Drivers/Devices/Log/inc/srml_utils.h"
#include "Drivers/Devices/Log/utils/log_send.h"
#endif

#if USE_SRML_BMI088  
/** TODO:  待BMI088库测试完善，加入srml.h目录里*/
/** TODO:  25赛季完全删除注释部分，停止对老版BMI解算的支持*/
//#include "../USP/BMIDriver/Mahony.hpp"
//#include "../USP/BMIDriver/Drv_BMI088.hpp"
#endif
#ifdef __cplusplus
extern "C" {
#endif

/* Macro Definitions ---------------------------------------------------------*/
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
	
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8
/* HAL Handlers --------------------------------------------------------------*/
/* RTOS Resources ------------------------------------------------------------*/
/* Task */
extern TaskHandle_t Rx_Referee_Handle;
#if USE_SRML_FS_I6X
extern TaskHandle_t FS_I6X_Handle;
#endif // !USE_SRML_FS_I6X
/* Queues */
extern QueueHandle_t USART_TxPort;
extern QueueHandle_t USART_RxPort;
extern QueueHandle_t CAN1_TxPort;				
extern QueueHandle_t CAN1_RxPort;		
extern QueueHandle_t CAN2_TxPort;
extern QueueHandle_t CAN2_RxPort;

#if USE_SRML_DR16
extern QueueHandle_t DR16_QueueHandle;
#endif

#if USE_SRML_VT13
    extern QueueHandle_t VT13_QueueHandle;
#endif
/* Semaphores */
/* mutex */
#if USE_SRML_DR16
extern SemaphoreHandle_t DR16_mutex;
#endif

#if USE_SRML_VT13
    extern SemaphoreHandle_t VT13_mutex;
#endif
/* Mutexes */
/* Notifications */
/* Other Resources -----------------------------------------------------------*/
#if USE_SRML_VIRTUAL_COM
extern uint8_t VirtualCom_Rx_Buff[VIRTUALCOM_RX_BUFFER_SIZE]; // 虚拟串口缓存区
#endif

#if  USE_SRML_MPU6050
extern mpu_rec_s mpu_receive; //mpu6050传感器
#endif

#if USE_SRML_BMI088
/** TODO:  25赛季完全删除，停止对老版BMI解算的支持*/
//extern BMI088SensorClassdef BMI088;
//extern MahonyFilterClassdef BMI088_AHRS;
#endif

#if  USE_SRML_DR16
extern DR16_Classdef DR16;		//遥控器DR16类
#endif

#if USE_SRML_VT13
    extern VT13_Classdef VT13; // VT13遥控器类
#endif

#if USE_SRML_FS_I6X
extern FS_I6X_Classdef remote;
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
extern referee_Classdef Referee;//裁判系统类
#endif

extern Motor_C610 m2006;
extern Motor_C620 m3508;
extern Motor_GM6020 gm6020[2];

extern abstractMotor<Motor_C610> absM2006;
extern abstractMotor<Motor_C620> absM3508;
extern abstractMotor<Motor_GM6020> absM6020[2];
/* Exported function declarations --------------------------------------------*/
void Service_Debug_Init(void);
void Service_Communication_Init(void);
void Service_Devices_Init(void);

void User_CAN1_RxCpltCallback(CAN_COB *CAN_RxCOB);
void User_CAN2_RxCpltCallback(CAN_COB *CAN_RxCOB);

extern User_Uart_Callback UART1_RxCpltCallback;
extern User_Uart_Callback UART2_RxCpltCallback;
extern User_Uart_Callback UART3_RxCpltCallback;
extern User_Uart_Callback UART4_RxCpltCallback;
extern User_Uart_Callback UART5_RxCpltCallback;
extern User_Uart_Callback UART6_RxCpltCallback;

#if USE_SRML_DR16
uint32_t DR16_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#if USE_SRML_VT13
    uint32_t VT13_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif // !USE_SRML_VT13

#if USE_SRML_FS_I6X
uint32_t FS_I6X_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
uint32_t Referee_recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#if USE_SRML_VIRTUAL_COM
void User_VirtualComRecCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INTERNAL_H_ */
