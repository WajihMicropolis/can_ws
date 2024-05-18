/*
 * CANBus.h
 *
 *  Created on: Feb 26, 2024
 *      Author: Najib
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stdint.h"
#include "PrjTypedef.h"

/*APIs....................................................*/
void CANInit(void);
int inject_data_frame(const char *hex_id, const char *hex_data , int data_len);
int frame_recv(unsigned char *frame);
void CANMessageParser(unsigned char *Data , int len);
void sigterm(int signo);

#endif /* INC_CANBUS_H_ */
