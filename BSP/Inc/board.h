#ifndef COURSE_BOARD_H
#define COURSE_BOARD_H
#include "course.h"
void Board_Init(void);
uint32_t Board_Micros(void);
int Board_Read(unsigned port,uint8_t *b); /* 0=IMU, 1=PC */
int Board_Write(unsigned port,const uint8_t *p,size_t n); /* Atomic enqueue; 0 if busy. */
void Board_Snapshot(FgCapture fg[2]);
void Board_Apply(const float duty[2],const int8_t direction[2],const RobotConfig *c);
void Board_Stop(void);
void Board_Led(int on);
int Board_Key(void);
uint32_t Board_Errors(void);
void Board_Fatal(void);
#endif
