#ifndef __CAN_INTERFACE_HPP__
#define __CAN_INTERFACE_HPP__

#include <PrjTypedef.hpp>
// #include <CANBus.hpp>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <time.h>
#include <ctype.h>
#include <signal.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <stdint.h>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Range.h"
#include <tf2/LinearMath/Quaternion.h>

class CANBus;

using namespace std;
using namespace std_msgs;
using namespace sensor_msgs;
class CAN_Interface
{
private:
#define CANUSB_INJECT_SLEEP_GAP_DEFAULT 200 /* ms */
#define CANUSB_TTY_BAUD_RATE_DEFAULT 2000000
#define CAN_BInt16MultiArrayUS_SPEED 1000000
#define CAN_BUS_SPEED 1000000

   // TODO char
   char *USB_PORT;
   char CAN_IDENTIFER[3] = "5";

   typedef enum
   {
      CANUSB_SPEED_Default = 0,
      CANUSB_SPEED_1000000 = 0x01,
      CANUSB_SPEED_800000 = 0x02,
      CANUSB_SPEED_500000 = 0x03,
      CANUSB_SPEED_400000 = 0x04,
      CANUSB_SPEED_250000 = 0x05,
      CANUSB_SPEED_200000 = 0x06,
      CANUSB_SPEED_125000 = 0x07,
      CANUSB_SPEED_100000 = 0x08,
      CANUSB_SPEED_50000 = 0x09,
      CANUSB_SPEED_20000 = 0x0a,
      CANUSB_SPEED_10000 = 0x0b,
      CANUSB_SPEED_5000 = 0x0c,
   } CANUSB_SPEED;

   typedef enum
   {
      CANUSB_MODE_NORMAL = 0x00,
      CANUSB_MODE_LOOPBACK = 0x01,
      CANUSB_MODE_SILENT = 0x02,
      CANUSB_MODE_LOOPBACK_SILENT = 0x03,
   } CANUSB_MODE;

   typedef enum
   {
      CANUSB_FRAME_STANDARD = 0x01,
      CANUSB_FRAME_EXTENDED = 0x02,
   } CANUSB_FRAME;

   typedef enum
   {
      CANUSB_INJECT_PAYLOAD_MODE_RANDOM = 0,
      CANUSB_INJECT_PAYLOAD_MODE_INCREMENTAL = 1,
      CANUSB_INJECT_PAYLOAD_MODE_FIXED = 2,
   } CANUSB_PAYLOAD_MODE;

   int terminate_after,
       inject_payload_mode;
   int c,

       baudrate;

   float inject_sleep_gap,
       temp;

   char *inject_id,
       *inject_data;
   CANUSB_SPEED speed;

   // unsigned char frame[20];

   float current_velocity,
       current_steering,
       prev_velocity,
       prev_steering;

   int lengthData = 0;
   unsigned char RecivedData[21] = {0};
   uint8_t CAN_Message_Length = 0;
   uint16_t CAN_ID = 0;
   uint8_t BoardNumber;
   uint8_t Status;
   uint8_t MessageNumber;

   pair<float, float> _robotOrientation;
   pair<string, string> _steering_braking_status;

   CANBus *canBus;
   // CANMessage cMessage;
   string doorStateStr[6] = {"opened", "closed", "opening", "closing","stopped","error"};
   

   CANUSB_SPEED canusb_int_to_speed(int speed);
   struct timespec timespec_diff(struct timespec start, struct timespec end);
   int generate_checksum(const unsigned char *data, int data_len);
   int frame_is_complete(const unsigned char *frame, int frame_len);
   int frame_send(unsigned char *frame, int frame_len);
   int frame_recv(unsigned char *frame);
   int command_settings(CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame);
   int send_data_frame(CANUSB_FRAME frame, unsigned char id_lsb, unsigned char id_msb, const char *data, int data_length_code);
   int hex_value(int c);
   int convert_from_hex(const char *hex_string, unsigned char *bin_string, int bin_string_len);
   void dump_data_frames(void);
   int adapter_init(const char *tty_device, int baudrate);
   static void sigterm(int signo);

   float getRobotSpeed(Int16MultiArray motorsSpeed);
   string getSteeringError(WheelSteeringStatus &steeringError);
   string getBrakingError(WheelBrakingStatus &brakingError);
   pair<float, float> getRobotOrientation(float &yaw);
   pair<string, string> steeringBrakingStatus();
   String getDrivingMode(float &_robotSpeedFb, Int16MultiArray &_brakePercentageFb, float &_robotSpeedCommand);

public:
   typedef struct
   {
      Int16MultiArray motors_speed,
          steering_angle,
          brake_percentage,
          ultrasonic;
      Float32MultiArray rpy;

      BatteryState battery_state;
      Imu imu;
      Range front_right_ultrasonic,
          front_left_ultrasonic,
          back_right_ultrasonic,
          back_left_ultrasonic;
      Float32 robot_speed;

      String door_state;
      String lifter_state;
      String drone_base_state;
      String steering_status;
      String braking_status;
      String driving_mode;

      Bool steering_health_check;
      Bool braking_health_check;

   } CANFeedback;

   int inject_data_frame(const char *hex_id, const char *hex_data, int data_len);
   void getFeedback(CANFeedback &feedback);

   void sendEmergencyBrake(bool emergencyBrake);
   void sendCmdVel(float velocity, float steering);
   void sendDoorControl(uint8_t doorControl);

   CAN_Interface(char *port);
   ~CAN_Interface();
};

#endif //__CAN_INTERFACE_HPP__