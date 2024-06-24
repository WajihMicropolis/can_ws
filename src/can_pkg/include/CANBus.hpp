/*
 * CANBus.h
 *
 *  Created on: Feb 26, 2024
 *      Author: najee
 */

#ifndef __CANBUS_HPP__
#define __CANBUS_HPP__

#include "stdint.h"
#include "CAN_Interface.hpp"
#include "PrjTypedef.hpp"
#include "string.h"
#include "stdbool.h"
#include "math.h"
#include <stdio.h>

class CANBus
{
private:
	// CAN_Interface *_CANInterface;

public:
uint8_t CANAknowledgeFlag = 0;

/*CAN MESSAGES********************************************/

/*CAN BUS Library Version***********************************/
#define MAJOR_VERSION 0
#define MINOR_VERSION 4
#define ROBOT_TYPE 0x01 //! 0x01 for M2Police, 0x02 for M2Qss Delivery, 0x03 for M2Qss Drone#define ROBOT_VERSION 0x01 // 0x01 for delivery | 0x00 for drone
/*CAN MESSAGES***********************************************/

/*Main Board*************************************************/
#define MRCU_ACKNOWLEDGE 0x01

#define MRCU_WHEELS_VELOCITY 0X02
#define MRCU_WHEELS_TORQUE 0X03

#define MRCU_IMU_ACCERLERATION 0X04
#define MRCU_IMU_VELOCITY 0X05
#define MRCU_IMU_ANGULAR_VELOCITY 0X06
#define MRCU_IMU_ANGLE 0X07

#define MRCU_STEERING_COMMAND 0X08
#define MRCU_BRAKING_COMMAND 0X09

#define MRCU_STEERING_ANGLE 0X0A
#define MRCU_STEERING_ANGULAR_VELOCITY 0X0B

#define MRCU_BRAKE_VALUE 0X0C
#define MRCU_ULTRASONIC 0X0D

#define MRCU_STATE_OF_CHARGE 0X0E
#define MRCU_POWER_STATUS_CAHNNEL_REQUEST 0X0F
#define MRCU_TEMP_HUMIDITY_FAN_REQUEST 0X10
#define MRCU_RELAYS_H_BRIGDE_DIGITAL_OUTPUTS_COMMAND 0X11
#define MRCU_DIGITAL_ANALOG_INPUT_REQUEST 0X12
#define MRCU_DOOR_AND_LIFTER_STATUS 0X13

#define MRCU_RESERVED_MESSAGE_1 0X13
#define MRCU_RESERVED_MESSAGE_2 0X14
#define MRCU_RESERVED_MESSAGE_3 0X15
#define MRCU_RESERVED_MESSAGE_4 0X16
#define MRCU_RESERVED_MESSAGE_5 0X17
#define MRCU_RESERVED_MESSAGE_6 0X18
#define MRCU_RESERVED_MESSAGE_7 0X19
#define MRCU_RESERVED_MESSAGE_8 0X1A

/*ORIN Board************************************************/
#define ORIN_SET_DRIVING_COMMAND 0x1B
#define ORIN_SET_STEERING_MODE 	 0x1C
#define ORIN_SET_LIGHTS 		 0x1D
#define ORIN_SET_EMERGENCY_BRAKE 0x1E
#define ORIN_SET_AUTONOMOUS_MODE 0x1F

#define ORIN_DOOR_LIFTER_COMMAND 0X20
#define ORIN_RESERVED_MESSAGE_2  0X21
#define ORIN_RESERVED_MESSAGE_3  0X22
#define ORIN_RESERVED_MESSAGE_4  0X23
#define ORIN_RESERVED_MESSAGE_5  0X24
#define ORIN_RESERVED_MESSAGE_6  0X25

/*STEERING BOARD*******************************************/
#define STEERING_ANGLE_FRONT 0x26
#define STEERING_ANGLE_REAR 0x27
#define STEERING_ANGULAR_VELOCITY_FRONT 0x28
#define STEERING_ANGULAR_VELOCITY_REAR 0x29
#define STEERING_STATUS_ERROR 0X2A

#define STEERING_RESERVED_MESSAGE_2 0X2B
#define STEERING_RESERVED_MESSAGE_3 0X2C
#define STEERING_RESERVED_MESSAGE_4 0X2D
#define STEERING_RESERVED_MESSAGE_5 0X2E
#define STEERING_RESERVED_MESSAGE_6 0X2F
#define STEERING_RESERVED_MESSAGE_7 0X30

/*BRAKE BOARD*/
#define BRAKE_APPLIED_TORQUE_FRONT 0X31
#define BRAKE_APPLIED_TORQUE_REAR 0X32
#define BRAKE_BRAKE_VALUE 0X33
#define BRAKE_MOTORS_SYSTEM_STATUS 0X34
#define BRAKE_MOTORS_CURRENT 0X35
#define BRAKE_MOTORS_TEMPERATURE 0X36

#define BRAKE_RESERVED_MESSAGE_4 0X37
#define BRAKE_RESERVED_MESSAGE_5 0X38
#define BRAKE_RESERVED_MESSAGE_6 0X39

/*PDU BOARD************************************************/
#define PDU_BATTERY_STATE_OF_CHARGE 0X3A
#define PDU_FAULT_MESSAGE 0X3B
#define PDU_POWER_STATUS_CAHNNEL_RESPONSE 0X3C
#define PDU_TEMP_HUMIDITY_FAN_RESPONSE 0X3D
#define PDU_DIGITAL_ANALOG_INPUT_RESPONSE 0X3E

#define PDU_RESERVED_MESSAGE_1 0X3F

#define CAST(__u16_DATA) *((uint16_t *)&__u16_DATA)
#define CONVERTERu16tou8(pData, value) (*((uint16_t *)(pData)) = (value))

	typedef struct
	{
		uint32_t CANMessgeID;
		uint8_t CANMessageLength;
		uint8_t CANMessageData[8];
	} CANMessage;

	typedef enum
	{
		Stream = 0x00,
		Command = 0x01,
		Inform = 0x02,
		Response = 0x03
	} CANMessageStatus;

	typedef enum
	{
		MRCU = 0x01,
		AutonomousBoard = 0x02,
		SteeringBoard = 0x03,
		BrakingBoard = 0x04,
		PDUBoard = 0x05
	} Board;

	STATUS_TypeDef SendCANMessage(CANMessage Message);
	STATUS_TypeDef SendCANMessageWithRetries(CANMessage Message, uint32_t timeout, uint8_t trials);
	STATUS_TypeDef ParserCANMessage(uint16_t ID, unsigned char *RxData);
	uint32_t EncodeCANID(uint8_t BoardNumber, uint8_t messageNumber, uint8_t status);
	void DecodeCANID(uint16_t canID, uint8_t *boardNumber, uint8_t *messageNumber, uint8_t *status);
	uint16_t float_encode(float _number);
	float float_decode(uint16_t _number);
	uint8_t GetSenderCANID(uint8_t MessageNumber);
	STATUS_TypeDef SendAcknowledgmentMessage(uint8_t MessageNumber);

	CANBus();
	~CANBus();
};

#endif /* __CANBUS_HPP__ */
