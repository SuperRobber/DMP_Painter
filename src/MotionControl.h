#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include <Arduino.h>
#include "RoboTimer.h"
#include "TMC_Registers.h"

extern volatile int64_t HeightMap[24];

struct DrawInstruction {
    int64_t index;
    uint8_t type;
    int8_t dirX;
    int8_t dirY;
    int64_t startX;
    int64_t startY;;
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

//======== State Machine  ==========//
enum State {
    state_none=0,
    state_start=1,
    state_home=2,
    state_draw=3,
    state_idle=4,
    state_panic=5,
    state_eof=6,
    state_reset=7,
    state_mapheight=10,
    state_clearheight=11
};

enum Action {
    action_none=0,
    action_moving=1,
    action_drawing=2,
    action_homeing=3,
    action_waiting=4,
    action_sleeping=5,
    action_mapping=6,
    action_panicked=7,
    action_draw_newline=8,
    action_draw_move=9,
    action_draw_draw=10  
};

extern volatile enum Action currentAction;
extern volatile enum State  activeState;
extern volatile enum State  requestedState;

//======== CIRCULAR Buffer for drawinstructions ==========//

/// @brief Circular buffer for drawing instructions
/// @attention use power of 2 size so I can use & in stead of modulo.
/// e.g. tailIndex = (tailIndex + 1) & 63;
extern volatile DrawInstruction iBuffer[64];  

extern volatile uint8_t iBufferWriteIndex;
extern volatile uint8_t iBufferReadIndex;
extern volatile int64_t requestedInstruction;
extern volatile int64_t receivedInstruction;

// extern volatile uint32_t plotter_pos_x;
// extern volatile uint32_t plotter_pos_y;

extern volatile int32_t M1_pos;
extern volatile int32_t M2_pos;
extern volatile int32_t M3_pos;
extern volatile int32_t M4_pos;
extern volatile int32_t M5_pos;

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

extern volatile uint8_t drawFunction;
extern volatile int32_t drawIndex;

// extern volatile uint8_t Limit_Y1_start_press_count;
// extern volatile uint8_t Limit_Y1_end_press_count;
// extern volatile uint8_t Limit_Y2_start_press_count;
// extern volatile uint8_t Limit_Y2_end_press_count;

FASTRUN void MachineLoop();
FASTRUN void CalculateHomeSteps();

/// @brief Perform Motor Steps that are calculated in previous iteration
/// and updates position variables.
/// @return 
FASTRUN void StepMotors();

/// @brief Set PRE-STEP Directions and check LIMIT switches, 
/// disables steps if limits are hit.
/// @attention FASTRUN in interrupt 
/// @return
FASTRUN void SetDirectionsAndLimits();

FASTRUN void CalculateDrawSteps();
FASTRUN void DebounceSwitches();
FASTRUN void StepX(int8_t dir);
FASTRUN void StepY(int8_t dir);
FASTRUN void setCurrent(int cur);
FASTRUN void CalculateStraightLine();
FASTRUN void CalculateQuadBezier();

void StartUp();
void configureSwitches();
void configureStepperDrivers();
TMC262::STATUS setTMC262Register(uint8_t bytes[3], int CSPIN);
void updateStepperStatus();

#endif