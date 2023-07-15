/// TODO:

/// 1. CHECK / REVISIT Motor status, for possible breaking errors that need to halt entire machine.

///
/// 2. Add status from M4 and M5

///
/// 5. Separate Configuration // Hardware setup to another file ?
///

#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include <Arduino.h>
#include "RoboTimer.h"
#include "TMC_Registers.h"

#define XMAX 1100000 // total X steps +- 1127620 | margin (13581)
#define YMAX 1568000 // total Y steps +- 1595169 | margin (13581)

/// @brief Union to convert int64 to byteArray.
union byte64
{
    int64_t value;
    byte bytes[8];
};

/// @brief Union to convert uint64 to byteArray.
union ubyte64
{
    uint64_t value;
    byte bytes[8];
};

/// @brief Union to convert int32 to byteArray.
union byte32
{
    int32_t value;
    byte bytes[4];
};

struct DrawInstruction
{
    int64_t index;
    uint8_t type;
    uint8_t acceleration;
    int8_t dirX;
    int8_t dirY;
    int8_t dirZ;
    uint8_t projection;
    int64_t startX;
    int64_t startY;
    int64_t startZ;
    int64_t endX;
    int64_t endY;
    int64_t endZ;
    int64_t deltaX;
    int64_t deltaY;
    int64_t deltaZ;
    int64_t deltaXX;
    int64_t deltaYY;
    int64_t deltaZZ;
    int64_t deltaXY;
    int64_t deltaXZ;
    int64_t deltaYZ;
    int64_t error;
    int64_t errorX;
    int64_t errorY;
    int64_t errorZ;
    int64_t deltaMax;
    double steps;
    double step;
};

struct MoveInstruction
{
    int8_t dirX;
    int8_t dirY;
    int8_t dirZ;
    int64_t endX;
    int64_t endY;
    int64_t endZ;
    int64_t deltaX;
    int64_t deltaY;
    int64_t error;
    double steps;
    double step;
};


// ===================== Machines task switching & State Machines =====================

enum class Mode
{
    None,
    Start,
    Home,
    Draw,
    Stop,
    Idle,
    Panic,
    EOL,
    Reset,
    MapHeight,
    ClearHeight
};

/// @brief State Machine used for drawing lines.
enum class DrawState
{
    None,
    Choose,
    MoveXY,
    MoveZ,
    Draw
};

/// @brief State Machine used for homing.
enum class HomeState
{
    None,
    Limit,
    Zero,
    Done
};

/// @brief State Machine used for building a heightMap.
enum class MapHeightState
{
    None,
    Choose,
    MoveUp,
    StartMoveXY,
    MoveXY,
    MoveDown,
    Done
};

extern volatile Mode activeMode;
extern volatile Mode requestedMode;

// ===================== Circular buffer for draw instructions. =====================

/// @brief Draw Instruction circular buffer
/// @attention use power of 2 size so I can use & in stead of modulo.
/// e.g. tailIndex = (tailIndex + 1) & 63;
extern volatile DrawInstruction iBuffer[64];

/// @brief Write index for draw instruction buffer
extern volatile uint8_t iBufferWriteIndex;

/// @brief Read index for draw instruction buffer
extern volatile uint8_t iBufferReadIndex;

/// @brief Index for last requested Instruction
extern volatile int64_t requestedInstruction;

/// @brief Index for last received Instruction
extern volatile int64_t receivedInstruction;

// ===================== Height Mapping algorithm. =====================

#define HeightMapWidth 5
#define HeightMapHeight 8
#define HeightMapSize 40

extern volatile int64_t HeightMap[];

/// ===================== Limit switch states =====================

/// Switch hardware pins

#define pinPanic 23
#define pinY1Start 36
#define pinY1End 35
#define pinY2Start 38
#define pinY2End 37
#define pinXStart 40
#define pinXEnd 39
#define pinZStart 41
#define pinZEnd 14

/// Switch array Index names

#define swPanic 0
#define swY1Start 1
#define swY1End 2
#define swY2Start 3
#define swY2End 4
#define swXStart 5
#define swXEnd 6
#define swZStart 7
#define swZEnd 8

const int numSwitches = 9;

struct LimitSwitch
{
    uint8_t hwPin;
    uint8_t onBounce;
    uint8_t offBounce;
    bool pressed;
};

extern volatile LimitSwitch switches[];

/// =====================  Status  =====================


enum class StatusFunction {
    Idle = 0,
    Waiting = 1,
    Moving = 2,
    Drawing = 3,
    Homing = 4,
    Mapping = 5,
};

/// @brief Used to display status.
/// Indicating current action of the machine.
extern volatile StatusFunction statusFunction;

/// @brief Used to display status.
/// Indicating current function of the machine.
extern volatile uint8_t drawFunction;

/// @brief Used to display status.
/// Current line being drawn.
extern volatile int32_t drawIndex;

/// @brief union for Motor 1 status register.
extern TMC262::STATUS status_M1;

/// @brief union for Motor 2 status register.
extern TMC262::STATUS status_M2;

/// @brief union for Motor 3 status register.
extern TMC262::STATUS status_M3;

/// ===================== Motion Control =====================

/// motor position (global for status display)

// struct Stepper
// {
//     int32_t pos;
//     int8_t direction;
//     bool step;
// };

// extern volatile Stepper motors[];

extern volatile int32_t M1_pos;
extern volatile int32_t M2_pos;
extern volatile int32_t M3_pos;
extern volatile int32_t M4_pos;
extern volatile int32_t M5_pos;

/// ===================== FUNCTIONS that are part of the interrupt loop (FASTRUN) =====================

/// @brief The main motion control loop. Called via interrupt timer. To move as
/// continuous as possible, prepared motor stepping is performed at beginning of each
/// loop. After that actions for the do next iteration are calculated, which can take
/// variable amount of time. Following this pattern, movement is controlled at a fixed
/// interval as close to the interrupt call as possible.
FASTRUN void MachineLoop();

/// @brief The homing algorithm
// FASTRUN void CalculateHomeSteps();

/// @brief The drawing algorithm.
FASTRUN void Draw();

/// @brief The homing algorithm.
FASTRUN void Home();

/// @brief The height mapping algorithm.
FASTRUN void MapHeight();

/// @brief Perform motor steps that are calculated in previous iteration,
/// reset step triggers and updates position variables.
/// @return
FASTRUN void StepMotors();

/// @brief Set PRE-STEP Directions and check LIMIT switches,
/// disables steps if limits are hit.
/// @attention FASTRUN in interrupt
/// @return
FASTRUN void SetDirectionsAndLimits();

/// @brief Debouncing of Limit switches.
/// Fast press and slow release debounce.
/// Press debounce is shifted in 8 StepLoops
/// Release debounce is performed every 300000 cycles
FASTRUN void DebounceSwitches();

/// @brief Prepare a step in X direction
/// @param dir positive or negative X direction
FASTRUN void StepX(int8_t dir);

/// @brief Prepare a step in Y direction
/// @param dir positive or negative Y direction
FASTRUN void StepY(int8_t dir);

/// @brief Prepare a step in Z direction
/// @param dir positive or negative Z direction
FASTRUN void StepZ(int8_t dir);

/// @brief Manually set current scale on Stepper Motors
/// @param cur 0 - 31 current scale
FASTRUN void setCurrent(int cur);

/// @brief Draw / Step in a straight Line
FASTRUN void CalculateStraightLine();
FASTRUN void CalculateStraightLine3D();

/// @brief Draw / Step along a Quadratic Bezier
FASTRUN void CalculateQuadBezier();
FASTRUN void CalculateQuadBezier3DXY();
FASTRUN void CalculateQuadBezier3DXZ();
FASTRUN void CalculateQuadBezier3DYZ();

/// ===================== FUNCTIONS not part of the interrupt loop =====================

/// @brief Startup procedure, sets up interrupt timer and
/// enables stepper motors.
void StartUp();

/// @brief Configure hardware pins as input for switches
void configureSwitches();

/// @brief Configure hardware pins as output for stepper motors and configure
/// all the stepper driver registers.
void configureStepperDrivers();

/// @brief Reads status registers from Stepper drivers
void updateStepperStatus();

/// @brief Function to set TMC262 Stepper Driver register
/// @param bytes Register data (24bits)
/// @param CSPin SPI pin the driver is attached to
/// @return A status union with various driver flags and driver information
TMC262::STATUS setTMC262Register(uint8_t bytes[3], int CSPin);

/// @brief Function to set TMC263 Stepper Driver register
/// @param address Register address (8bits)
/// @param data Register data (32bits)
/// @param CSPin SPI pin the driver is attached to
/// @return A status union with various driver flags and driver information
TMC2130::SPI_STATUS setTMC2130Register(uint8_t address, uint32_t data, int CSPin);

#endif