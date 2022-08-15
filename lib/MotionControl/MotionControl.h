#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include <Arduino.h>
#include "RoboTimer.h"
#include "TMC_Registers.h"

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

extern volatile bool Limit_X1_start;
extern volatile bool Limit_X1_end  ;
extern volatile bool Limit_X2_start;
extern volatile bool Limit_X2_end  ;
extern volatile bool Limit_Y_start ;
extern volatile bool Limit_Y_end   ;
extern volatile bool Limit_Z_start ;
extern volatile bool Limit_Z_end   ;

extern volatile uint8_t Limit_X1_start_press_count;
extern volatile uint8_t Limit_X1_end_press_count;
extern volatile uint8_t Limit_X2_start_press_count;
extern volatile uint8_t Limit_X2_end_press_count;


FASTRUN void StepperLoop();
void configureSwitches();
void configureStepperDrivers();
TMC262::STATUS setTMC262Register(uint8_t bytes[3], int CSPIN);
void updateStepperStatus();

#endif