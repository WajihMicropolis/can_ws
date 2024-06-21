#include "Dummy_CAN_Interface.hpp"
#include <CANBus.hpp>
static int program_running;
int tty_fd_new;
CANBus::CANMessage cMessage;
int print_traffic = 0;

CAN_Interface::CAN_Interface(char *port) : USB_PORT(port)
{

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
int steer_error = 0;
string CAN_Interface::getSteeringError(WheelSteeringStatus &steeringError)
{
   string error = "";

   if (steer_error == 0)
      error += "Over Temperature";
   else if (steer_error == 1)
      error += "Over Current";
   else if (steer_error == 2)
      error += "Position Sensor Damaged";
   else if (steer_error == 3)
      error += "Position Sensor Response Error";
   else if (steer_error == 4)
      error += "Error";
   else if (steer_error == 5)
      error += "Connecton Lost";
   else
      error += "OK";
   ++steer_error > 10 ?  steer_error = 0 : steer_error;
   return error;
}
int brake_error = 0;
string CAN_Interface::getBrakingError(WheelBrakingStatus &brakingError)
{
   string error = "";

   if (brake_error == 0)
      error += "Over Temperature";
   else if (brake_error == 1)
      error += "Over Current";
   else if (brake_error == 2)
      error += "Error";
   else if (brake_error == 3)
      error += "Connecton Lost";
   else
      error += "OK";

   ++brake_error > 10 ? brake_error = 0 : brake_error;
   return error;
}

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
      if (i == steeringStatus.size() - 1)
      {
         _steeringStatus += steeringStatus[i];
         _brakingStatus += brakingStatus[i];
      }
      else
      {
         _steeringStatus += steeringStatus[i] + ":";
         _brakingStatus += brakingStatus[i] + ":";
      }
   }
   return make_pair(_steeringStatus, _brakingStatus);
}

void CAN_Interface::getFeedback(CANFeedback &feedback)
{
   feedback.motors_speed.data.clear();
   feedback.motors_speed.data.push_back(rand()%100);
   feedback.motors_speed.data.push_back(rand()%100);
   feedback.motors_speed.data.push_back(rand()%100);
   feedback.motors_speed.data.push_back(rand()%100);
   
   feedback.steering_angle.data.clear();
   feedback.steering_angle.data.push_back(rand()%16);
   feedback.steering_angle.data.push_back(rand()%16);
   feedback.steering_angle.data.push_back(rand()%16);
   feedback.steering_angle.data.push_back(rand()%16);

   feedback.brake_percentage.data.clear();
   feedback.brake_percentage.data.push_back(rand()%10);
   feedback.brake_percentage.data.push_back(rand()%10);
   feedback.brake_percentage.data.push_back(rand()%10);
   feedback.brake_percentage.data.push_back(rand()%10);

   feedback.ultrasonic.data.clear();
   feedback.ultrasonic.data.push_back(rand()%200); // *2 to convert it to cm
   feedback.ultrasonic.data.push_back(rand()%200);
   feedback.ultrasonic.data.push_back(rand()%200);
   feedback.ultrasonic.data.push_back(rand()%200);
   feedback.ultrasonic.data.push_back(rand()%200);
   feedback.ultrasonic.data.push_back(rand()%200);

   feedback.front_right_ultrasonic.min_range = feedback.front_left_ultrasonic.min_range = feedback.back_right_ultrasonic.min_range = feedback.back_left_ultrasonic.min_range = 0.4;
   feedback.front_right_ultrasonic.max_range = feedback.front_left_ultrasonic.max_range = feedback.back_right_ultrasonic.max_range = feedback.back_left_ultrasonic.max_range = 4.0;
   feedback.front_right_ultrasonic.range = rand()%200;
   feedback.front_left_ultrasonic.range = rand()%200;
   feedback.back_right_ultrasonic.range = rand()%200;
   feedback.back_left_ultrasonic.range = rand()%200;

   feedback.battery_state.voltage = rand()%52;
   feedback.battery_state.current = -1 * rand()% 5;
   feedback.battery_state.percentage = feedback.battery_state.voltage * 100.0 / 52;
   feedback.battery_state.capacity = rand()%100;
   

   feedback.rpy.data.clear();
   feedback.rpy.data.push_back(rand()%180);
   feedback.rpy.data.push_back(rand()%180);
   feedback.rpy.data.push_back(rand()%180);

   _robotOrientation = getRobotOrientation(feedback.rpy.data[2]);
   feedback.imu.orientation.x = feedback.imu.orientation.y = 0.0;
   feedback.imu.orientation.z = _robotOrientation.first;
   feedback.imu.orientation.w = _robotOrientation.second;

   feedback.imu.angular_velocity.x = rand()%10;
   feedback.imu.angular_velocity.y = rand()%10;
   feedback.imu.angular_velocity.z = rand()%10;

   feedback.imu.linear_acceleration.x = rand()%10;
   feedback.imu.linear_acceleration.y = rand()%10;
   feedback.imu.linear_acceleration.z = -9.8;

   feedback.steering_health_check.data = rand()%2 == 0 ? true : false;
   feedback.braking_health_check.data = rand()%2 == 0 ? true : false;

   feedback.robot_speed.data = getRobotSpeed(feedback.motors_speed);

   // cout<<"Vehicle.Door_Lifter.DoorStatus" << Vehicle.Door_Lifter.DoorStatus << endl;
   // printf("Vehicle.Door_Lifter.DoorStatus: %d\n", Vehicle.Door_Lifter.DoorStatus);
   // printf("Vehicle.Door_Lifter.LifterStatus: %d\n", Vehicle.Door_Lifter.LifterStatus);
   // printf("Vehicle.Door_Lifter.DroneBaseStatus: %d\n", Vehicle.Door_Lifter.DroneBaseStatus);

   feedback.door_state.data = doorStateStr[0];
   feedback.lifter_state.data = doorStateStr[0];
   feedback.drone_base_state.data = doorStateStr[0];

   _steering_braking_status = steeringBrakingStatus();
   feedback.steering_status.data = _steering_braking_status.first;
   feedback.braking_status.data = _steering_braking_status.second;

   // feedback.driving_mode = getDrivingMode(feedback.robot_speed.data, feedback.brake_percentage, Vehicle.DesiredCarParameters.DesiredVelocity);

   // TODO: add feedback for lights
   // cout << "WheelFrontRight.DrivingVelocity: " << Vehicle.WheelFrontRight.Driving.DrivingVelocity << endl;
   // printf(" WheelRearRight.Steering.SteeringAngle %f\n", Vehicle.WheelRearRight.Steering.SteeringAngle);
   // printf("WheelFrontRight.BrakeValue: %d\n", Vehicle.WheelFrontRight.Braking.BrakeValue);
   // cout << "Angle.Angle_Roll: " << Vehicle.IMUSensor.Angle.Angle_Roll << endl;
   // cout << "Angle.Angle_Pitch: " << Vehicle.IMUSensor.Angle.Angle_Pitch << endl;
   // cout << "Angle.Angle_Yaw: " << Vehicle.IMUSensor.Angle.Angle_Yaw << endl;
   // cout << "steering_health_check: " << Vehicle.SteeringSystemState.SystemReady << endl;
   // cout << "BrakingSystemState: " << Vehicle.BrakingSystemState.SystemReady << endl;
   // cout << "--------------------" << endl;
}

CAN_Interface::~CAN_Interface()
{
}