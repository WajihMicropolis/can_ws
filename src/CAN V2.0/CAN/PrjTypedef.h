/*
 * PrjTypedef.h
 *
 *  Created on: Feb 9, 2024
 *      Author: najee
 */

#ifndef INC_PRJTYPEDEF_H_
#define INC_PRJTYPEDEF_H_

#include "stdbool.h"
#include "stdint.h"

typedef enum
{
  STATUS_OK       = 0x00U,
  STATUS_ERROR    = 0x01U,
  STATUS_BUSY     = 0x02U,
  STATUS_TIMEOUT  = 0x03U
} STATUS_TypeDef;

/*
  Wheel Front Right  = 0x00
  Wheel Front Left   = 0x01
  Wheel Rear Left    = 0x02
  Wheel Rear Right   = 0x03
*/
// 
// Orin ID: 0x02
#define BOARDNUMBER 0x02 

typedef  enum
{
	Disable = 0,
	Front_ackerman,
	Double_ackerman,
	Carb_steer,
	Front_parallel,
	Double_parallel,
	Goto_home,
	Calibration,
}SteeringMode;

typedef struct
{
	float DrivingVelocity;
	uint16_t DrivingVelocitySampleTime;

	float   DrivingTorque;
	uint16_t DrivingTorqueSampleTime;
} WheelDrivingParameter;


typedef struct
{
	bool MotorOverTemperature;
	bool MotorOverCurrent;
	bool PositionSensorDemaged;
	bool PositionSensorResponse;
	bool MotorError;
	bool MotorConnectionLoss;
} WheelSteeringStatus;


typedef struct
{
	bool MotorOverTemperature;
	bool MotorOverCurrent;
	bool MotorError;
	bool MotorConnectionLoss;
} WheelBrakingStatus;

typedef struct {
    uint16_t value : 10;  // 10-bit wide bit field
} Limitswitches;

typedef struct
{
	bool MotorCANBusConnection;
	bool BoardCANBusConnection;
	bool SteeringSystemError;
	bool MotorsInitError;
	bool SteeringNeedCalibration;
	bool CalibrationTimout;
	bool CalibrationBusy;
	bool EEPROMError;
	bool SystemReady;
	bool SysteminCalibrationMode;
	bool SysteminOperatingMode;
	Limitswitches LimitswitchesStatus;
} SteeringSystemStatus;

typedef struct
{
	bool MotorCANBusConnection;
	bool SystemReady;
	bool SysteminCalibrationMode;
	bool SysteminCalibrationError;
	bool MotorInitERROR;
} BrakingSystemStatus;
typedef struct
{
	float 				  SteeringAngle;
	uint16_t 			  SteeringAngleSampleTime;

	float 				  SteeringAngularVelocity;
	uint16_t 			  SteeringAngularVelocitySampleTime;

	SteeringMode 		  SteeringMode;

	WheelSteeringStatus   WheelSteeringState;

} WheelSteeringParameter;

typedef struct
{
	float BrakeTorque;
	uint16_t BrakeTorqueSampleTime;
	uint8_t  BrakeValue;
	float BrakeCurrent;
	float BrakeTemperature;
	WheelBrakingStatus BrakingStatus;
} WheelBrakeParameter;

typedef struct
{
	bool Over_Voltage;
	bool Under_Voltage;
	bool Over_Temperature;
	bool Under_Temperature;
	bool Discharge_Over_Current;
	bool Charge_Over_Current;
	bool System_Error;
}Battery_Protection_Data;


typedef struct
{
	float BatteryVoltage;
	float BatteryCurrent;
	float BatteryTemperature;
	float Range;

	uint8_t BatteryChargePercentage;
	Battery_Protection_Data BatteryStatus;
} BatteryParameter;

typedef struct
{
	BatteryParameter BatteryStateOfCharge;
} VehiclePDU;

typedef struct
{
	WheelDrivingParameter   Driving;
	WheelSteeringParameter  Steering;
	WheelBrakeParameter 	Braking;
} Wheel;

typedef struct
{
float Acceleration_Without_gX;
float Acceleration_Without_gY;
float Acceleration_Without_gZ;
float Acceleration_X;
float Acceleration_Y;
float Acceleration_Z;
} Acceleration;

typedef struct
{
float Velocity_X;
float Velocity_Y;
float Velocity_Z;
float Velocity_abs;
} Velocity;

typedef struct
{
float AngularVelocity_Roll;
float AngularVelocity_Pitch;
float AngularVelocity_Yaw;
} AngularVelocity;

typedef struct
{
float Angle_Roll;
float Angle_Pitch;
float Angle_Yaw;
} Angle;


typedef struct
{
	Acceleration Acceleration;
	Velocity Velocity;
	AngularVelocity AngularVelocity;
	Angle Angle;
} IMUValues;

typedef struct
{
	float Angle;
	float AngularVelocity;
	SteeringMode SteeringMode;
} SteeringCommands;

typedef struct
{
	float BrakeTorqueFrontRight;
	float BrakeTorqueFrontLeft;
	float BrakeTorqueRearRight;
	float BrakeTorqueRearLeft;
} BrakeCommands;

typedef struct
{
	uint8_t FrontRight;
//	uint8_t FrontMiddleRight;
//	uint8_t FrontMiddleLeft;
	uint8_t FrontLeft;
	uint8_t RearRight;
//	uint8_t RearMiddleRight;
//	uint8_t RearMiddleLeft;
	uint8_t RearLeft;
	uint8_t Left;
	uint8_t Right;
} UltraSonic;

typedef struct
{
	float DesiredVelocity;
	float DesiredAcceleration;
	float DesiredSteeringAngle;
	float DesiredSteeringAngularVelocity;
	float DesiredBrake;
	SteeringMode DesiredSteeringMode;

} DesiredDrivingParameters;

typedef struct
{
	IMUValues IMUSensor;
	SteeringCommands SteeringCommand;
	BrakeCommands  BrakeCommand;
	UltraSonic UltraSonicSensors;
	Wheel WheelFrontRight;
	Wheel WheelFrontLeft;
	Wheel WheelRearRight;
	Wheel WheelRearLeft;
	VehiclePDU PDU;
	DesiredDrivingParameters DesiredCarParameters;
	bool AutonomousMode;
	BrakingSystemStatus   BrakingSystemState;
	SteeringSystemStatus  SteeringSystemState;


} Vehicle_Status;


extern Vehicle_Status Vehicle;

typedef struct
{
	float DesiredVelocity;
	float DesiredAcceleration;
} DrivingCommand;


#endif /* INC_PRJTYPEDEF_H_ */