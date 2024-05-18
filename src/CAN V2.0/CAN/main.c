#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <time.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>
#include <stdint.h>
#include "CAN.h"
#include "CANBus.h"
#include "PrjTypedef.h"

int lengthData = 0;
unsigned char RecivedData[21] = {0};
uint8_t CAN_Message_Length = 0;
uint16_t CAN_ID = 0;
uint8_t BoardNumber;
uint8_t Status;
uint8_t MessageNumber;
CANMessage cMessage;
int main(int argc, char *argv[])
{
  /* initialize random seed: */
  srand(time(NULL));

  CANInit();
  memset(RecivedData, 0, sizeof(RecivedData)); // Initialize buffer to zero before using it.
  int counter = 0;
  time_t t; // not a primitive datatype

  while (1)
  {
    // signal(SIGTERM, sigterm);
    // signal(SIGHUP, sigterm);
    // signal(SIGINT, sigterm);
    lengthData = frame_recv(RecivedData);
    if (lengthData != 0 && lengthData != -1)
    {
      if (RecivedData[0] == 0xaa && RecivedData[lengthData - 1] == 0x55)
      {
        CAN_ID = RecivedData[3] << 8 | RecivedData[2];
        DecodeCANID(CAN_ID, &BoardNumber, &MessageNumber, &Status);
        if (BoardNumber == BOARDNUMBER)
        {
          CAN_Message_Length = (uint8_t)(RecivedData[1] & 0x0F);
          ParserCANMessage(CAN_ID, &RecivedData[4]);
          // fprintf(stdout, "counter: %d\n", );

          fprintf(stdout, "Vehicle.WheelFrontRight.Car_Steering.Wheel_Steering_Angle=%.2f\n",
                  Vehicle.WheelFrontRight.Steering.SteeringAngle);
          // fprintf(stdout, "Vehicle.WheelFrontLeft.Car_Steering.Wheel_Steering_Angle=%.2f\n",
          //         Vehicle.WheelFrontLeft.Steering.SteeringAngle);
          // fprintf(stdout, "Vehicle.WheelRearRight.Car_Steering.Wheel_Steering_Angle=%.2f\n",
          //         Vehicle.WheelRearRight.Steering.SteeringAngle);
          // fprintf(stdout, "Vehicle.WheelRearLeft.Car_Steering.Wheel_Steering_Angle=%.2f\n",
          //         Vehicle.WheelRearLeft.Steering.SteeringAngle);
          // fprintf(stdout, "Vehicle.IMUSensor.Acceleration.ACC.X=%.2f\n",
          //         Vehicle.IMUSensor.Acceleration.Acceleration_X);
          // Vehicle.WheelRearRight.Steering.
          // Vehicle.UltraSonicSensors.FrontLeft
          // Vehicle.DesiredCarParameters.DesiredBrake

          fprintf(stdout, "-------------------------------------------------\n");
        }
      }
    }

    // cMessage.CANMessgeID = EncodeCANID(MRCU, ORIN_SET_DRIVING_COMMAND, Stream);                                               // receiber board id, message number (from CanBus.h), status of the message (from google form)
    // cMessage.CANMessageLength = 0x08;                                                                                         // from google form
    // // generate random number between 0 and 16?

    // Vehicle.DesiredCarParameters.DesiredVelocity = rand() %16;                                                              // from ros
    // Vehicle.DesiredCarParameters.DesiredAcceleration = rand() % 10;                                                                   // from ros
    // Vehicle.DesiredCarParameters.DesiredSteeringAngle = rand() % 10;                                                          // from ros in degrees +- 16
    // Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity = rand() % 10;                                                        // from ros
    // CONVERTERu16tou8(&cMessage.CANMessageData[0], float_encode(Vehicle.DesiredCarParameters.DesiredVelocity));                // velocity m/s
    // CONVERTERu16tou8(&cMessage.CANMessageData[2], float_encode(Vehicle.DesiredCarParameters.DesiredAcceleration));            // acceleration m/s^2
    // CONVERTERu16tou8(&cMessage.CANMessageData[4], float_encode(Vehicle.DesiredCarParameters.DesiredSteeringAngle));           // steering angle
    // CONVERTERu16tou8(&cMessage.CANMessageData[6], float_encode(Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity)); // steering angular velocity

    // //  cMessage.CANMessageData[0]=0x02;
    // printf("Vehicle.DesiredCarParameters.DesiredVelocity= %f \n", Vehicle.DesiredCarParameters.DesiredVelocity);
    // printf("Vehicle.DesiredCarParameters.DesiredAcceleration= %f \n", Vehicle.DesiredCarParameters.DesiredAcceleration);
    // printf("Vehicle.DesiredCarParameters.DesiredSteeringAngle= %f \n", Vehicle.DesiredCarParameters.DesiredSteeringAngle);
    // printf("Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity= %f \n", Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity);
    // SendCANMessage(cMessage);
  }
  return EXIT_SUCCESS;
}
