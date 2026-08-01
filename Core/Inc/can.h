/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stm32f4xx_hal_can.h"



/* USER CODE BEGIN Prototypes */
typedef struct{
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

typedef struct {
	float 		p_des;  // (rad)
	float 		v_des;  // (rad/s)
	float 		kp;     // (N.m/rad)
	float 		kd;     // (N.m.s/rad)
	float 		t_ff;   // (N.m)
	uint32_t	can_id; // ID 

} JointCommand_t;

typedef struct {
	uint32_t	can_id;   // ID của động cơ trên bus CAN
	float 		p_act;    // (rad)
	float 		v_act;    // (rad/s)
	float 		t_act;    // (N.m)
	float 		v_batt;   // (Volts)
} JointState_t;

typedef struct {
	const char*	name; 		// (name URDF on ROS 2)
	double 		position; // (rad)
	double 		velocity; // (rad/s)
	double 		effort;   // (N.m)
} JointROS_t;


typedef struct {
	JointCommand_t	cmd;      
	JointState_t    state;    
	JointROS_t 		ros; 
} JointRobot_t;

extern CAN_HandleTypeDef hcan2;
extern CANTxMessage can_tx;
extern CANRxMessage can_rx;
extern JointCommand_t joint_cmd;
extern JointState_t joint_state;


#define KT			        1.0f                   // Torque Constant (N-m/A)
#define GR			        1.0f           // Gear ratio

#define CAN_SID         0x02               // CAN bus ID
#define CAN_MID         0x01               // CAN bus "master" ID
#define CAN_TIMEOUT     1000               // CAN bus timeout period

#define P_MAX		        12.5f             // Position setupoint upper bound (rad)
#define V_MAX		        270.0f              // Velocity setpoint upper bound (rad/s)
#define T_MAX           18.0f	


#define KP_MAX		        100.0f              // Max position gain (N-m/rad)

#define KD_MAX		        2.0f               // Max velocity gain (N-m/rad/s)


#define VB_MIN					0.0f				
#define VB_MAX					50.0f				


void can_rx_init(CANRxMessage *msg);
void can_tx_init(CANTxMessage *msg);
void MX_CAN2_Init(void);
void Master_Pack_Cmd(CANTxMessage *msg, JointCommand_t *cmd);
void Master_Unpack_State(CANRxMessage *msg, JointState_t *state);
void DataCmdJoint(JointCommand_t *cmd, float angle_rad, float rads, float tor);
void MasterSendData(CAN_HandleTypeDef *hcan,CANTxMessage *msg, JointCommand_t *cmd);

void Master_Pack_Cmd_2(CANTxMessage *msg, JointCommand_t *cmd);
void Master_Unpack_State_2(CANRxMessage *msg, JointState_t *state);
void MasterSendData_2(CAN_HandleTypeDef *hcan,CANTxMessage *msg, JointCommand_t *cmd);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/