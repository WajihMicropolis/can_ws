#include "CAN_Interface.hpp"
#include <CANBus.hpp>
static int program_running;
int tty_fd_new;
CANBus::CANMessage cMessage;
int print_traffic = 0;

CAN_Interface::CAN_Interface(char *port) : USB_PORT(port)
{

   this->inject_id = CAN_IDENTIFER;
   this->terminate_after = 1;
   program_running = 1;
   this->inject_payload_mode = CANUSB_INJECT_PAYLOAD_MODE_FIXED;
   this->inject_sleep_gap = CANUSB_INJECT_SLEEP_GAP_DEFAULT;
   this->speed = canusb_int_to_speed(CAN_BUS_SPEED);
   this->baudrate = CANUSB_TTY_BAUD_RATE_DEFAULT;

   // this->cMessage = new CANMessage();
   signal(SIGTERM, sigterm);
   signal(SIGHUP, sigterm);
   signal(SIGINT, sigterm);

   tty_fd_new = adapter_init(USB_PORT, baudrate);
   printf("ttyfd: %d\n", tty_fd_new);
   if (tty_fd_new == -1)
   {
      printf("Wrong in init adpater");
      exit(1);
   }
   command_settings(speed, CANUSB_MODE_NORMAL, CANUSB_FRAME_STANDARD);

   // srand(time(NULL));
   memset(RecivedData, 0, sizeof(RecivedData)); // Initialize buffer to zero before using it.

   canBus = new CANBus();
   printf(" CAN Interface init");
}

CAN_Interface::CANUSB_SPEED CAN_Interface::canusb_int_to_speed(int speed)
{
   switch (speed)
   {
   case 1000000:
      return CANUSB_SPEED_1000000;
   case 800000:
      return CANUSB_SPEED_800000;
   case 500000:
      return CANUSB_SPEED_500000;
   case 400000:
      return CANUSB_SPEED_400000;
   case 250000:
      return CANUSB_SPEED_250000;
   case 200000:
      return CANUSB_SPEED_200000;
   case 125000:
      return CANUSB_SPEED_125000;
   case 100000:
      return CANUSB_SPEED_100000;
   case 50000:
      return CANUSB_SPEED_50000;
   case 20000:
      return CANUSB_SPEED_20000;
   case 10000:
      return CANUSB_SPEED_10000;
   case 5000:
      return CANUSB_SPEED_5000;
   default:
      return CANUSB_SPEED_Default;
   }
}

int CAN_Interface::generate_checksum(const unsigned char *data, int data_len)
{
   int i, checksum;

   checksum = 0;
   for (i = 0; i < data_len; i++)
   {
      checksum += data[i];
   }

   return checksum & 0xff;
}

int CAN_Interface::frame_is_complete(const unsigned char *frame, int frame_len)
{
   if (frame_len > 0)
   {
      if (frame[0] != 0xaa)
      {
         /* Need to sync on 0xaa at start of frames, so just skip. */
         return 1;
      }
   }

   if (frame_len < 2)
   {
      return 0;
   }

   if (frame[1] == 0x55)
   { /* Command frame... */
      if (frame_len >= 20)
      { /* ...always 20 bytes. */
         return 1;
      }
      else
      {
         return 0;
      }
   }
   else if ((frame[1] >> 4) == 0xc)
   { /* Data frame... */
      if (frame_len >= (frame[1] & 0xf) + 5)
      { /* ...payload and 5 bytes. */
         return 1;
      }
      else
      {
         return 0;
      }
   }

   /* Unhandled frame type. */
   return 1;
}

int CAN_Interface::frame_send(unsigned char *frame, int frame_len)
{
   int result, i;
   if (print_traffic)
   {
      printf(">>> ");
      for (i = 0; i < frame_len; i++)
      {
         printf("%02x ", frame[i]);
      }
      if (print_traffic > 1)
      {
         printf("    '");
         for (i = 4; i < frame_len - 1; i++)
         {
            printf("%c", isalnum(frame[i]) ? frame[i] : '.');
         }
         printf("'");
      }
      printf("\n");
   }
   // printf("ttyfd: %d\n", tty_fd_new);
   // printf("frame_len: %d\n", frame_len);
   // printf("frame: %s\n", frame);
   result = write(tty_fd_new, frame, frame_len);
   if (result == -1)
   {
      printf("error");
      fprintf(stderr, "write() failed: %s\n", strerror(errno));
      return -1;
   }

   return frame_len;
}

int CAN_Interface::frame_recv(unsigned char *frame)
{
   int result, frame_len, checksum;
   unsigned char byte;

   if (print_traffic)
      fprintf(stderr, "<<< ");

   frame_len = 0;
   int count = 0;
   // cout << "before program running" << endl;

   while (program_running)
   {
      count++;
      result = read(tty_fd_new, &byte, 1);
      if (result == -1)
      {
         if (errno != EAGAIN && errno != EWOULDBLOCK)
         {
            fprintf(stderr, "read() failed: %s\n", strerror(errno));
            return -1;
         }
      }
      else if (result > 0)
      {
         if (print_traffic)
            fprintf(stderr, "%02x ", byte);

         if (frame_len >= 20)
         {
            fprintf(stderr, "frame_recv() failed: Overflow\n");
            return -1;
         }

         frame[frame_len++] = byte;

         if (frame_is_complete(frame, frame_len))
         {
            break;
         }
         // cout << " counter: " << count << endl;
      }

      usleep(10);
   }
   if (print_traffic)
      fprintf(stderr, "\n");

   /* Compare checksum for command frames only. */
   if ((frame_len == 20) && (frame[0] == 0xaa) && (frame[1] == 0x55))
   {
      checksum = generate_checksum(&frame[2], 17);
      if (checksum != frame[frame_len - 1])
      {
         fprintf(stderr, "frame_recv() failed: Checksum incorrect\n");
         return -1;
      }
   }

   return frame_len;
}

int CAN_Interface::command_settings(CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame)
{
   int cmd_frame_len;
   unsigned char cmd_frame[20];

   cmd_frame_len = 0;
   cmd_frame[cmd_frame_len++] = 0xaa;
   cmd_frame[cmd_frame_len++] = 0x55;
   cmd_frame[cmd_frame_len++] = 0x12;
   cmd_frame[cmd_frame_len++] = speed;
   cmd_frame[cmd_frame_len++] = frame;
   cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
   cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
   cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
   cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
   cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
   cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
   cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
   cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
   cmd_frame[cmd_frame_len++] = mode;
   cmd_frame[cmd_frame_len++] = 0x01;
   cmd_frame[cmd_frame_len++] = 0;
   cmd_frame[cmd_frame_len++] = 0;
   cmd_frame[cmd_frame_len++] = 0;
   cmd_frame[cmd_frame_len++] = 0;
   cmd_frame[cmd_frame_len++] = generate_checksum(&cmd_frame[2], 17);

   if (frame_send(cmd_frame, cmd_frame_len) < 0)
   {
      printf("Error");
      return -1;
   }

   return 0;
}

int CAN_Interface::send_data_frame(CANUSB_FRAME frame, unsigned char id_lsb, unsigned char id_msb, const char *data, int data_length_code)
{
#define MAX_FRAME_SIZE 13
   int data_frame_len = 0;
   unsigned char data_frame[MAX_FRAME_SIZE] = {0x00};

   if (data_length_code < 0 || data_length_code > 8)
   {
      fprintf(stderr, "Data length code (DLC) must be between 0 and 8!\n");
      return -1;
   }

   /* Byte 0: Packet Start */
   data_frame[data_frame_len++] = 0xaa;

   /* Byte 1: CAN Bus Data Frame Information */
   data_frame[data_frame_len] = 0x00;
   data_frame[data_frame_len] |= 0xC0; /* Bit 7 Always 1, Bit 6 Always 1 */
   if (frame == CANUSB_FRAME_STANDARD)
      data_frame[data_frame_len] &= 0xDF;          /* STD frame */
   else                                            /* CANUSB_FRAME_EXTENDED */
      data_frame[data_frame_len] |= 0x20;          /* EXT frame */
   data_frame[data_frame_len] &= 0xEF;             /* 0=Data */
   data_frame[data_frame_len] |= data_length_code; /* DLC=data_len */
   data_frame_len++;

   /* Byte 2 to 3: ID */
   data_frame[data_frame_len++] = id_lsb; /* lsb */
   data_frame[data_frame_len++] = id_msb; /* msb */

   /* Byte 4 to (4+data_len): Data */
   for (int i = 0; i < data_length_code; i++)
      data_frame[data_frame_len++] = data[i];

   /* Last byte: End of frame */
   data_frame[data_frame_len++] = 0x55;

   if (frame_send(data_frame, data_frame_len) < 0)
   {
      fprintf(stderr, "Unable to send frame!\n");
      return -1;
   }

   return 0;
}

int CAN_Interface::hex_value(int c)
{
   if (c >= 0x30 && c <= 0x39) /* '0' - '9' */
      return c - 0x30;
   else if (c >= 0x41 && c <= 0x46) /* 'A' - 'F' */
      return (c - 0x41) + 10;
   else if (c >= 0x61 && c <= 0x66) /* 'a' - 'f' */
      return (c - 0x61) + 10;
   else
      return -1;
}

int CAN_Interface::convert_from_hex(const char *hex_string, unsigned char *bin_string, int bin_string_len)
{
   int n1, n2, high;

   high = -1;
   n1 = n2 = 0;
   while (hex_string[n1] != '\0')
   {
      if (hex_value(hex_string[n1]) >= 0)
      {
         if (high == -1)
         {
            high = hex_string[n1];
         }
         else
         {
            bin_string[n2] = hex_value(high) * 16 + hex_value(hex_string[n1]);
            n2++;
            if (n2 >= bin_string_len)
            {
               printf("hex string truncated to %d bytes\n", n2);
               break;
            }
            high = -1;
         }
      }
      n1++;
   }

   return n2;
}

int CAN_Interface::inject_data_frame(const char *hex_id, const char *hex_data, int data_len)
{

   unsigned char binary_data[8];
   unsigned char binary_id_lsb = 0, binary_id_msb = 0;
   int error = 0;

   if (data_len == 0)
   {
      fprintf(stderr, "Unable to convert data from hex to binary!\n");
      return -1;
   }

   switch (strlen(hex_id))
   {
   case 1:
      binary_id_lsb = hex_value(hex_id[0]);
      break;

   case 2:
      binary_id_lsb = (hex_value(hex_id[0]) * 16) + hex_value(hex_id[1]);
      break;

   case 3:
      binary_id_msb = hex_value(hex_id[0]);
      binary_id_lsb = (hex_value(hex_id[1]) * 16) + hex_value(hex_id[2]);
      break;

   default:
      fprintf(stderr, "Unable to convert ID from hex to binary!\n");
      return -1;
   }

   error = send_data_frame(CANUSB_FRAME_STANDARD, binary_id_lsb, binary_id_msb, hex_data, data_len);

   return error;
}

void CAN_Interface::dump_data_frames(void)
{
   int i, frame_len;
   unsigned char frame[32];
   struct timespec ts;

   while (program_running)
   {
      frame_len = frame_recv(frame);

      if (!program_running)
         break;

      clock_gettime(CLOCK_MONOTONIC, &ts);
      printf("%lu.%06lu ", ts.tv_sec, ts.tv_nsec / 1000);

      if (frame_len == -1)
      {
         printf("Frame recieve error!\n");
      }
      else
      {

         if ((frame_len >= 6) &&
             (frame[0] == 0xaa) &&
             ((frame[1] >> 4) == 0xc))
         {
            printf("Frame ID: %02x%02x, Data: ", frame[3], frame[2]);
            for (i = frame_len - 2; i > 3; i--)
            {
               printf("%02x ", frame[i]);
            }
            printf("\n");
         }
         else
         {
            printf("Unknown: ");
            for (i = 0; i <= frame_len; i++)
            {
               printf("%02x ", frame[i]);
            }
            printf("\n");
         }
      }

      if (terminate_after && (--terminate_after == 0))
         program_running = 0;
   }
}

int CAN_Interface::adapter_init(const char *tty_device, int baudrate)
{
   int _tty_fd, result;
   struct termios2 tio;

   _tty_fd = open(tty_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
   if (_tty_fd == -1)
   {
      fprintf(stderr, "open(%s) failed: %s\n", tty_device, strerror(errno));
      return -1;
   }

   result = ioctl(_tty_fd, TCGETS2, &tio);
   if (result == -1)
   {
      fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
      close(_tty_fd);
      return -1;
   }

   tio.c_cflag &= ~CBAUD;
   tio.c_cflag = BOTHER | CS8 | CSTOPB;
   tio.c_iflag = IGNPAR;
   tio.c_oflag = 0;
   tio.c_lflag = 0;
   tio.c_ispeed = baudrate;
   tio.c_ospeed = baudrate;

   result = ioctl(_tty_fd, TCSETS2, &tio);
   if (result == -1)
   {
      fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
      close(_tty_fd);
      return -1;
   }
   ioctl(_tty_fd, TCFLSH, 2);
   return _tty_fd;
}
void CAN_Interface::sigterm(int signo)
{
   program_running = 0;

   cout << "exit program" << endl;
   close(tty_fd_new);
   exit(0);
}

void CAN_Interface::sendCmdVel(float velocity, float steering)
{
   int steering_min = -16,
       steering_max = 16;

   cMessage.CANMessgeID = canBus->EncodeCANID(canBus->MRCU, ORIN_SET_DRIVING_COMMAND, canBus->Stream); // receiber board id, message number (from CanBus.h), status of the message (from google form)
   cMessage.CANMessageLength = 0x08;                                                                   // from google form

   steering = steering < steering_min ? steering_min : steering > steering_max ? steering_max
                                                                               : steering;
   Vehicle.DesiredCarParameters.DesiredVelocity = velocity;           // from ros
   Vehicle.DesiredCarParameters.DesiredAcceleration = 0;              // from ros
   Vehicle.DesiredCarParameters.DesiredSteeringAngle = int(steering); // from ros in degrees +- 16
   Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity = 10;
   CONVERTERu16tou8(&cMessage.CANMessageData[0], canBus->float_encode(Vehicle.DesiredCarParameters.DesiredVelocity));                // velocity m/s
   CONVERTERu16tou8(&cMessage.CANMessageData[2], canBus->float_encode(Vehicle.DesiredCarParameters.DesiredAcceleration));            // acceleration m/s^2
   CONVERTERu16tou8(&cMessage.CANMessageData[4], canBus->float_encode(Vehicle.DesiredCarParameters.DesiredSteeringAngle));           // steering angle
   CONVERTERu16tou8(&cMessage.CANMessageData[6], canBus->float_encode(Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity)); // steering angular velocity

   canBus->SendCANMessage(cMessage);
   // printf("Vehicle.DesiredCarParameters.DesiredVelocity= %f \n", Vehicle.DesiredCarParameters.DesiredVelocity);
   // printf("Vehicle.DesiredCarParameters.DesiredAcceleration= %f \n", Vehicle.DesiredCarParameters.DesiredAcceleration);
   // printf("Vehicle.DesiredCarParameters.DesiredSteeringAngle= %f \n", Vehicle.DesiredCarParameters.DesiredSteeringAngle);
   // printf("Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity= %f \n", Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity);
}

void CAN_Interface::sendEmergencyBrake(bool emergencyBrake)
{
   cMessage.CANMessgeID = canBus->EncodeCANID(canBus->MRCU, ORIN_SET_EMERGENCY_BRAKE, canBus->Stream); // receiber board id, message number (from CanBus.h), status of the message (from google form)
   cMessage.CANMessageLength = 0x02;                                                                   // from google form

   Vehicle.DesiredCarParameters.DesiredBrake = emergencyBrake ? 10 : 0;                                            // from ros
   CONVERTERu16tou8(&cMessage.CANMessageData[0], canBus->float_encode(Vehicle.DesiredCarParameters.DesiredBrake)); // velocity m/s

   canBus->SendCANMessage(cMessage);
}

void CAN_Interface::sendDoorControl(bool doorControl)
{
   // TODO re-check with najib
   cMessage.CANMessgeID = canBus->EncodeCANID(canBus->MRCU, ORIN_DOOR_LIFTER_COMMAND, canBus->Stream); // receiber board id, message number (from CanBus.h), status of the message (from google form)
   cMessage.CANMessageLength = 0x01;                                                                   // from google form

   Vehicle.Door_Lifter.DoorCommand = doorControl; // from ros

   CONVERTERu16tou8(&cMessage.CANMessageData[0], canBus->float_encode(Vehicle.Door_Lifter.DoorCommand)); // velocity m/s

   canBus->SendCANMessage(cMessage);
}

pair<float, float> CAN_Interface::getRobotOrientation(float &yaw)
{
   tf2::Quaternion myQuaternion;
   float yaw_rad;
   yaw_rad = yaw * (M_PI / 180);

   myQuaternion.setRPY(0, 0, yaw_rad);
   return make_pair(myQuaternion.getZ(), myQuaternion.getW());
}

float CAN_Interface::getRobotSpeed(Int16MultiArray motorsSpeed) // motors speed in rpm
{
   return (motorsSpeed.data[0] + motorsSpeed.data[1] + motorsSpeed.data[2] + motorsSpeed.data[3]) / 4;
}

string CAN_Interface::getSteeringError(WheelSteeringStatus &steeringError)
{
   string error = "";

   if (steeringError.MotorOverTemperature)
      error += "Motor Over Temperature";
   else if (steeringError.MotorOverCurrent)
      error += "Motor Over Current";
   else if (steeringError.PositionSensorDemaged)
      error += "Position Sensor Demaged";
   else if (steeringError.PositionSensorResponse)
      error += "Position Sensor Response";
   else if (steeringError.MotorError)
      error += "Motor Error";
   else if (steeringError.MotorConnectionLoss)
      error += "Motor Connection Loss";
   else
      error += "ok";
   return error;
}

string CAN_Interface::getBrakingError(WheelBrakingStatus &brakingError)
{
   string error = "";

   if (brakingError.MotorOverTemperature)
      error += "Motor Over Temperature";
   else if (brakingError.MotorOverCurrent)
      error += "Motor Over Current";
   else if (brakingError.MotorError)
      error += "Motor Error";
   else if (brakingError.MotorConnectionLoss)
      error += "Motor Connection Loss";
   else
      error += "ok";
   return error;
}

// String CAN_Interface::getDrivingMode(float &_robotSpeedFb, Int16MultiArray &_brakePercentageFb, float &_robotSpeedCommand)
// {
//    String drivingMode;
//    float brake_percentage_avg = (_brakePercentageFb.data[0] + _brakePercentageFb.data[1] + _brakePercentageFb.data[2] + _brakePercentageFb.data[3]) / 4;

//    if (_robotSpeedFb == 0 && brake_percentage_avg >= 0)
//       drivingMode.data = "P";
//    else if ((_robotSpeedFb > 0 && brake_percentage_avg == 0) || _robotSpeedCommand > 0)
//       drivingMode.data = "D";
//    else if ((_robotSpeedFb < 0 && brake_percentage_avg == 0) || _robotSpeedCommand < 0)
//       drivingMode.data = "R";

//    return drivingMode;
// }

pair<string, string> CAN_Interface::steeringBrakingStatus()
{

   vector<string> steeringStatus, brakingStatus;
   string _steeringStatus, _brakingStatus;

   steeringStatus.push_back(getSteeringError(Vehicle.WheelFrontRight.Steering.WheelSteeringState));
   steeringStatus.push_back(getSteeringError(Vehicle.WheelFrontLeft.Steering.WheelSteeringState));
   steeringStatus.push_back(getSteeringError(Vehicle.WheelRearRight.Steering.WheelSteeringState));
   steeringStatus.push_back(getSteeringError(Vehicle.WheelRearLeft.Steering.WheelSteeringState));

   brakingStatus.push_back(getBrakingError(Vehicle.WheelFrontRight.Braking.BrakingStatus));
   brakingStatus.push_back(getBrakingError(Vehicle.WheelFrontLeft.Braking.BrakingStatus));
   brakingStatus.push_back(getBrakingError(Vehicle.WheelRearRight.Braking.BrakingStatus));
   brakingStatus.push_back(getBrakingError(Vehicle.WheelRearLeft.Braking.BrakingStatus));

   for (int i = 0; i < steeringStatus.size(); i++)
   {  
      if(i == steeringStatus.size() - 1)
      {
         _steeringStatus += steeringStatus[i];
         _brakingStatus += brakingStatus[i];
      }
      else
      {
         _steeringStatus += steeringStatus[i] + ":" ;
         _brakingStatus += brakingStatus[i] + ":";
      }
   }
   return make_pair(_steeringStatus, _brakingStatus);
}

void CAN_Interface::getFeedback(CANFeedback &feedback)
{
   lengthData = frame_recv(RecivedData);
   if (lengthData != 0 && lengthData != -1)
   {
      if (RecivedData[0] == 0xaa && RecivedData[lengthData - 1] == 0x55)
      {
         CAN_ID = RecivedData[3] << 8 | RecivedData[2];
         canBus->DecodeCANID(CAN_ID, &BoardNumber, &MessageNumber, &Status);
         if (BoardNumber == BOARDNUMBER)
         {
            CAN_Message_Length = (uint8_t)(RecivedData[1] & 0x0F);
            canBus->ParserCANMessage(CAN_ID, &RecivedData[4]);

            feedback.motors_speed.data.clear();
            feedback.motors_speed.data.push_back(Vehicle.WheelFrontRight.Driving.DrivingVelocity);
            feedback.motors_speed.data.push_back(Vehicle.WheelFrontLeft.Driving.DrivingVelocity);
            feedback.motors_speed.data.push_back(Vehicle.WheelRearRight.Driving.DrivingVelocity);
            feedback.motors_speed.data.push_back(Vehicle.WheelRearLeft.Driving.DrivingVelocity);

            feedback.steering_angle.data.clear();
            feedback.steering_angle.data.push_back(Vehicle.WheelFrontRight.Steering.SteeringAngle);
            feedback.steering_angle.data.push_back(Vehicle.WheelFrontLeft.Steering.SteeringAngle);
            feedback.steering_angle.data.push_back(Vehicle.WheelRearRight.Steering.SteeringAngle);
            feedback.steering_angle.data.push_back(Vehicle.WheelRearLeft.Steering.SteeringAngle);

            feedback.brake_percentage.data.clear();
            feedback.brake_percentage.data.push_back(Vehicle.WheelFrontRight.Braking.BrakeValue);
            feedback.brake_percentage.data.push_back(Vehicle.WheelFrontLeft.Braking.BrakeValue);
            feedback.brake_percentage.data.push_back(Vehicle.WheelRearRight.Braking.BrakeValue);
            feedback.brake_percentage.data.push_back(Vehicle.WheelRearLeft.Braking.BrakeValue);

            feedback.ultrasonic.data.clear();
            feedback.ultrasonic.data.push_back(Vehicle.UltraSonicSensors.FrontRight * 2); // *2 to convert it to cm
            feedback.ultrasonic.data.push_back(Vehicle.UltraSonicSensors.FrontLeft * 2);
            feedback.ultrasonic.data.push_back(Vehicle.UltraSonicSensors.RearRight * 2);
            feedback.ultrasonic.data.push_back(Vehicle.UltraSonicSensors.RearLeft * 2);
            feedback.ultrasonic.data.push_back(Vehicle.UltraSonicSensors.Right * 2);
            feedback.ultrasonic.data.push_back(Vehicle.UltraSonicSensors.Left * 2);

            feedback.battery_state.voltage = Vehicle.PDU.BatteryStateOfCharge.BatteryVoltage;
            feedback.battery_state.current = Vehicle.PDU.BatteryStateOfCharge.BatteryCurrent;
            feedback.battery_state.percentage = Vehicle.PDU.BatteryStateOfCharge.BatteryChargePercentage;
            feedback.battery_state.capacity = Vehicle.PDU.BatteryStateOfCharge.Range;

            feedback.rpy.data.clear();
            feedback.rpy.data.push_back(Vehicle.IMUSensor.Angle.Angle_Roll);
            feedback.rpy.data.push_back(Vehicle.IMUSensor.Angle.Angle_Pitch);
            feedback.rpy.data.push_back(Vehicle.IMUSensor.Angle.Angle_Yaw);

            _robotOrientation = getRobotOrientation(Vehicle.IMUSensor.Angle.Angle_Yaw);
            feedback.imu.orientation.x = feedback.imu.orientation.y = 0.0;
            feedback.imu.orientation.z = _robotOrientation.first;
            feedback.imu.orientation.w = _robotOrientation.second;

            feedback.imu.angular_velocity.x = Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Roll * (M_PI / 180);
            feedback.imu.angular_velocity.y = Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Pitch * (M_PI / 180);
            feedback.imu.angular_velocity.z = Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Yaw * (M_PI / 180);

            feedback.imu.linear_acceleration.x = Vehicle.IMUSensor.Acceleration.Acceleration_X;
            feedback.imu.linear_acceleration.y = Vehicle.IMUSensor.Acceleration.Acceleration_Y;
            feedback.imu.linear_acceleration.z = Vehicle.IMUSensor.Acceleration.Acceleration_Z;

            feedback.steering_health_check.data = Vehicle.SteeringSystemState.SystemReady;
            feedback.braking_health_check.data = Vehicle.BrakingSystemState.SystemReady;

            feedback.robot_speed.data = getRobotSpeed(feedback.motors_speed);

            feedback.door_state.data = doorStateStr[Vehicle.Door_Lifter.DoorStatus];

            _steering_braking_status = steeringBrakingStatus();
            feedback.steering_status.data = _steering_braking_status.first;
            feedback.braking_status.data = _steering_braking_status.second;

            feedback.steering_health_check.data = Vehicle.SteeringSystemState.SystemReady;
            feedback.braking_health_check.data = Vehicle.BrakingSystemState.SystemReady;

            // feedback.driving_mode = getDrivingMode(feedback.robot_speed.data, feedback.brake_percentage, Vehicle.DesiredCarParameters.DesiredVelocity);

            // TODO: add feedback for lights
            cout << "WheelFrontRight.DrivingVelocity: " << Vehicle.WheelFrontRight.Driving.DrivingVelocity << endl;
            printf(" WheelRearRight.Steering.SteeringAngle %f\n", Vehicle.WheelRearRight.Steering.SteeringAngle);
            printf("WheelFrontRight.BrakeValue: %d\n", Vehicle.WheelFrontRight.Braking.BrakeValue);
            cout << "Angle.Angle_Roll: " << Vehicle.IMUSensor.Angle.Angle_Roll << endl;
            cout << "Angle.Angle_Pitch: " << Vehicle.IMUSensor.Angle.Angle_Pitch << endl;
            cout << "Angle.Angle_Yaw: " << Vehicle.IMUSensor.Angle.Angle_Yaw << endl;
            cout << "steering_health_check: " << Vehicle.SteeringSystemState.SystemReady << endl;
            cout << "BrakingSystemState: " << Vehicle.BrakingSystemState.SystemReady << endl;
            cout << "--------------------" << endl;
         }
      }
   }
}

CAN_Interface::~CAN_Interface()
{
}