// #include "CANBus.h"
// #include "CAN.h"
#include "CANBus.hpp"

Vehicle_Status Vehicle;
uint8_t MessageNumberSent = 0x00;

CAN_Interface *_CANInterface;
CANBus::CANBus()
{
}

CANBus::~CANBus()
{
}

