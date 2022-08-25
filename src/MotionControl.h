#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include <Arduino.h>
#include "RoboTimer.h"
#include "TMC_Registers.h"

struct DrawInstruction {
    int64_t index;
    uint8_t type;
    int8_t dirX;
    int8_t dirY;
    int64_t startX;
    int64_t startY;
    int64_t endX;
    int64_t endY;
    int64_t deltaX;
    int64_t deltaY;
    int64_t deltaXX;
    int64_t deltaYY;
    int64_t deltaXY;
    int64_t error;
    int64_t steps;
};

extern volatile DrawInstruction iBuffer[64];  // use power of 2 size so I can use & in stead of modulo // ex tailIndex = (tailIndex + 1) & 63;
extern volatile uint8_t iBufferWriteIndex;
extern volatile uint8_t iBufferReadIndex;

extern volatile uint32_t plotter_pos_x;
extern volatile uint32_t plotter_pos_y;

extern volatile uint32_t M1_pos;
extern volatile uint32_t M2_pos;
extern volatile uint32_t M3_pos;
extern volatile uint32_t M4_pos;
extern volatile uint32_t M5_pos;

extern TMC262::STATUS  status_M1;
extern TMC262::STATUS  status_M2;
extern TMC262::STATUS  status_M3;

extern TMC262::DRVCONF  driverConfig;
extern TMC262::CHOPCONF ChopperConfig;
extern TMC262::SGCSCONF StallGuardConfig;
extern TMC262::SMARTEN  CoolStepConfig;
extern TMC262::DRVCTRL  DriverControl;

extern const int M1_csPin;
extern const int M2_csPin;
extern const int M3_csPin;

extern volatile bool Limit_Y1_start;
extern volatile bool Limit_Y1_end  ;
extern volatile bool Limit_Y2_start;
extern volatile bool Limit_Y2_end  ;
extern volatile bool Limit_X_start ;
extern volatile bool Limit_X_end   ;
extern volatile bool Limit_Z_start ;
extern volatile bool Limit_Z_end   ;

// extern volatile uint8_t Limit_Y1_start_press_count;
// extern volatile uint8_t Limit_Y1_end_press_count;
// extern volatile uint8_t Limit_Y2_start_press_count;
// extern volatile uint8_t Limit_Y2_end_press_count;



FASTRUN void LimitSwitchesBounce();
FASTRUN void StepLoop();
FASTRUN void StepHomeLoop();

void StartEngines();
void configureSwitches();
void configureStepperDrivers();
TMC262::STATUS setTMC262Register(uint8_t bytes[3], int CSPIN);
void updateStepperStatus();

FASTRUN void MoveNext();

#endif