#include "MotionControl.h"
#include <EEPROM.h>
#include <SPI.h>

// ===================== Machines task switching / Machine modes =====================

volatile Mode activeMode = Mode::None;
volatile Mode requestedMode = Mode::None;

// ===================== Circular buffer for draw instructions. =====================

volatile DrawInstruction iBuffer[64];
volatile uint8_t iBufferWriteIndex = 0;
volatile uint8_t iBufferReadIndex = 0;
volatile int64_t requestedInstruction = -1;
volatile int64_t receivedInstruction = -1;

// ===================== Draw algorithm. =====================

volatile DrawState drawState = DrawState::None;

// volatile bool moving = true;
// volatile bool lineStarted = false;

// volatile int64_t drawDeltaX = 0;
// volatile int64_t drawDeltaY = 0;
// volatile int64_t drawDeltaZ = 0;
// volatile int64_t drawDeltaMax = 0;
// volatile int64_t drawError = 0;
// volatile int64_t drawErrX = 0;
// volatile int64_t drawErrY = 0;
// volatile int64_t drawErrZ = 0;
// volatile double drawStep = 0;


// ===================== Home algorithm. =====================

volatile HomeState homeState = HomeState::None;

// ===================== Height Mapping algorithm. =====================

volatile MapHeightState mapHeightState = MapHeightState::None;
volatile int64_t HeightMap[HeightMapSize];

const int heightMapMaxCount = 7; // number of times to probe one position;
volatile int64_t HeightMapProbeResults[heightMapMaxCount];
volatile int heightMapIndex = 0;
volatile int heightMapCount = 0;

/// ===================== Power Saving =====================

volatile uint64_t sleepTimer = 0;
const int workCurrent = 16;
const int sleepCurrent = 2;
volatile bool sleeping = false;

/// ===================== Limit switches =====================

/// Debounce cycle counter
volatile uint32_t debounceCounter = 0;

/// Collection of switches.
volatile LimitSwitch switches[numSwitches];

/// =====================  Stepper motor hardware, config and status  =====================

SPISettings tmc262_spi_config(5000000, MSBFIRST, SPI_MODE3);
SPISettings tmc2130_spi_config(4000000, MSBFIRST, SPI_MODE3);

const int spi_cs_delay = 50;

const int M1_M2_M3_ennPin = 4; // Inverted input (LOW means enable)
const int M4_M5_enPin = 10;    // Inverted input (LOW means enable)

const int M1_stepPin = 31;
const int M1_dirPin = 32;
const int M1_csPin = 5;
const int M1_sgPin = 8;

const int M2_stepPin = 29;
const int M2_dirPin = 30;
const int M2_csPin = 6;
const int M2_sgPin = 9;

const int M3_stepPin = 25;
const int M3_dirPin = 28;
const int M3_csPin = 7;
const int M3_sgPin = 24;

const int M4_stepPin = 3;
const int M4_dirPin = 2;
const int M4_csPin = 0;

const int M5_stepPin = 20;
const int M5_dirPin = 21;
const int M5_csPin = 19;

/// Config registers

TMC262::DRVCONF driverConfig = {0};
TMC262::CHOPCONF chopperConfig = {0};
TMC262::SGCSCONF stallGuardConfig = {0};
TMC262::SMARTEN coolStepConfig = {0};
TMC262::DRVCTRL driverControl = {0};

/// Status
volatile StatusFunction statusFunction = StatusFunction::Idle;

volatile uint8_t drawFunction = 0;
volatile int32_t drawIndex = 0;

TMC262::STATUS status_M1 = {0};
TMC262::STATUS status_M2 = {0};
TMC262::STATUS status_M3 = {0};

/// ===================== Motion Control =====================

// extern volatile Stepper motors[4];

volatile MoveInstruction move;
volatile DrawInstruction draw;

volatile int64_t posX = 0;
volatile int64_t posY = 0;
volatile int64_t posZ = 0;

const int64_t posPenUpZ = 0;

/// Hardware interrupt (PIR) TIMER triggering MachineLoop Interrupt.
RoboTimer IRQTimer;

/// Interrupt iteration times (speeds).
const float moveSpeed = 7.5f;
const float drawSpeed = 17.5f;
const float homeSpeed = 100.0f;
const float normalSpeed = 100.0f;

// acceleration
const double accelerationFactor = 200.0;
const double accelerationRange = 50.0;

/// @brief Interrupt iteration time (speed).
/// Interrupt type is always set to machineSpeed,
/// where machine speed is set depending on different
/// operations.
volatile float machineSpeed = normalSpeed;

/// prepared Step triggers

volatile bool stepM1 = false;
volatile bool stepM2 = false;
volatile bool stepM3 = false;
volatile bool stepM4 = false;
volatile bool stepM5 = false;

volatile int32_t M1_pos = 0;
volatile int32_t M2_pos = 0;
volatile int32_t M3_pos = 0;
volatile int32_t M4_pos = 0;
volatile int32_t M5_pos = 0;

volatile int8_t M1_direction = 0;
volatile int8_t M2_direction = 0;
volatile int8_t M3_direction = 0;
volatile int8_t M4_direction = 0;
volatile int8_t M5_direction = 0;

/// Track performance / measure interrupt-loop time.
volatile uint32_t max_step_cycles = 0;

/// ==========================================

void StartUp()
{
    /// Read the HeightMap from EEPROM
    for (unsigned int i = 0; i < HeightMapSize; i++)
    {
        byte64 b64 = {};
        for (int e = 0; e < 8; e++)
        {
            b64.bytes[e] = EEPROM.read(i * 8 + e);
        }
        HeightMap[i] = b64.value;
        Serial.println(HeightMap[i]);
    }
    activeMode = Mode::None;
    requestedMode = Mode::None;

    setCurrent(sleepCurrent);

    /// Enable stepper drivers
    digitalWriteFast(M1_M2_M3_ennPin, 0);
    digitalWriteFast(M4_M5_enPin, 0);

    setCurrent(workCurrent);

    delay(500);

    requestedMode = Mode::None;
    // lineStarted = false;

    /// Configure interrupt timer
    /// 150Mhz PIT timer clock
    CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;
    IRQTimer.priority(16);
    IRQTimer.begin(MachineLoop, machineSpeed);
}

FASTRUN void MachineLoop()
{
    /// track interrupt time performance
    uint32_t starttime = ARM_DWT_CYCCNT;

    switch (activeMode)
    {
    case Mode::Stop:
    {
        requestedMode = Mode::None;
        activeMode = Mode::None;
    }
    case Mode::EOL:
    {
        requestedMode = Mode::None;
        activeMode = Mode::None;
    }
    case Mode::Draw:
    {
        Draw();
        break;
    }
    case Mode::Home:
    {
        Home();
        break;
    }
    case Mode::MapHeight:
    {
        MapHeight();
        break;
    }
    case Mode::ClearHeight:
    {
        /// clear HeightMap and EEPROM
        for (int i = 0; i < HeightMapSize; i++)
        {
            byte64 b64 = {};
            b64.value = INT64_MIN;
            HeightMap[i] = INT64_MIN;
            for (int e = 0; e < 8; e++)
            {
                EEPROM.write(i * 8 + e, b64.bytes[e]);
            }
        }
        heightMapCount = 0;
        Serial.println("HeightMap cleared.");
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }
    case Mode::None:
    {
        stepM1 = false;
        stepM2 = false;
        stepM3 = false;
        stepM4 = false;
        stepM5 = false;

        if (!sleeping)
        {
            statusFunction = StatusFunction::Waiting;
            sleepTimer++;
            if (sleepTimer > 500000)
            {
                sleeping = true;
                setCurrent(sleepCurrent);
                statusFunction = StatusFunction::Idle;
            }
        }

        if (requestedMode == Mode::Draw)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;

            drawState = DrawState::Choose;
            statusFunction = StatusFunction::Waiting;
            activeMode = Mode::Draw;
        }

        if (requestedMode == Mode::Reset)
        {
            iBufferReadIndex = 0;
            iBufferWriteIndex = 0;
            requestedInstruction = -1;
            receivedInstruction = -1;
            drawIndex = 0;
            requestedMode = Mode::None;
        }

        if (requestedMode == Mode::Home)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;

            homeState = HomeState::Limit;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::Home;
        }

        if (requestedMode == Mode::MapHeight)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;

            mapHeightState = MapHeightState::Choose;
            activeMode = Mode::MapHeight;
        }

        if (requestedMode == Mode::ClearHeight)
        {
            activeMode = Mode::ClearHeight;
        }

        machineSpeed = normalSpeed;
        break;
    }
    default:
    {
        break;
    }
    }

    /// Debounce Switches and set button press or release triggers.
    DebounceSwitches();

    /// Set STEP SPEED (Interrupt time) for next Iteration.
    /// In case of a diagonal steps the loop should take SQRT(2) times
    /// longer to maintain constant speed.

    uint32_t cycles;
    if (stepM1 && stepM2 && stepM3)
    {
        // diagonal step
        cycles = (uint32_t)(150.0f * machineSpeed * M_SQRT2 - 0.5f);
    }
    else
    {
        cycles = (uint32_t)(150.0f * machineSpeed - 0.5f);
    }

    // cycles = (uint32_t)(150.0f * machineSpeed - 0.5f);

    IRQTimer.unsafe_update(cycles);

    /// Update the debounce time tracker.
    debounceCounter += cycles;

    /// Update variables for time tracking interrupt performance.
    uint32_t endtime = ARM_DWT_CYCCNT;
    if (endtime - starttime > max_step_cycles)
    {
        max_step_cycles = endtime - starttime;
    }
}

FASTRUN void Draw()
{
    switch (drawState)
    {
    case (DrawState::None):
    {
        break;
    }
    /// ================================================
    case (DrawState::Choose):
    {
        machineSpeed = normalSpeed;
        if (requestedMode == Mode::Stop)
        {
            // Check / LIFT PEN up
            if (posZ != posPenUpZ)
            {
                // Prepare to lift pen.
                move.endZ = posPenUpZ;
                move.dirZ = (posZ < move.endZ ? 1 : -1);
                move.steps = (double) abs(move.endZ - posZ);
                move.step = 0;
                drawState = DrawState::MoveZ;
                statusFunction = StatusFunction::Moving;
                break;
            }
            activeMode = Mode::Stop;
            drawState = DrawState::None;
            break;
        }

        /// Are there active or new instructions that need to be drawn ?
        if (iBufferReadIndex == iBufferWriteIndex)
        {
            /// Buffer is empty, Wait for EOL, or
            /// additional instructions.

            // Check / LIFT PEN up
            if (posZ != posPenUpZ)
            {
                // Prepare to lift pen.
                move.endZ = posPenUpZ;
                move.dirZ = (posZ < move.endZ ? 1 : -1);
                move.steps = (double) abs(move.endZ - posZ);
                move.step = 0;
                drawState = DrawState::MoveZ;
                statusFunction = StatusFunction::Moving;
                break;
            }

            if (requestedMode == Mode::EOL)
            {
                activeMode = Mode::EOL;
                drawState = DrawState::None;
            }
            break;
        }
        else
        {
            /// Start a new instruction or continue preparing to draw
            drawIndex = iBuffer[iBufferReadIndex].index;


            /// Move first?
            if (posX == iBuffer[iBufferReadIndex].startX && posY == iBuffer[iBufferReadIndex].startY && posZ == iBuffer[iBufferReadIndex].startZ)
            {
                // Already at start position, continue drawing.

                // copy values that will be changine during draw
                draw.error = iBuffer[iBufferReadIndex].error;
                draw.deltaX = iBuffer[iBufferReadIndex].deltaX;
                draw.deltaY = iBuffer[iBufferReadIndex].deltaY;
                draw.deltaZ = iBuffer[iBufferReadIndex].deltaZ;
                draw.step = 0.0;

                if (iBuffer[iBufferReadIndex].type==1)
                {
                    //could move these to the exporter
                    draw.deltaMax = max(draw.deltaZ, max(draw.deltaX, draw.deltaY));
                    draw.errorX = draw.deltaMax / 2;
                    draw.errorY = draw.deltaMax / 2;
                    draw.errorZ = draw.deltaMax / 2;
                }

                if (iBuffer[iBufferReadIndex].type==2)
                {
                    draw.errorX = iBuffer[iBufferReadIndex].errorX;
                    draw.errorY = iBuffer[iBufferReadIndex].errorY;
                    draw.errorZ = iBuffer[iBufferReadIndex].errorZ;
                    draw.deltaMax = iBuffer[iBufferReadIndex].deltaMax;
                    draw.endStage = false;
                }

                drawState = DrawState::Draw;
                statusFunction = StatusFunction::Drawing;

                /// Continue straight on to drawing by calling Draw() again,
                /// to smoothly connect multiple segments,
                /// but make sure to break and beware of endless loops!!
                Draw();
                break;
            }
            else
            {
                if (posX == iBuffer[iBufferReadIndex].startX && posY == iBuffer[iBufferReadIndex].startY)
                {
                    // Arrived at correct XY location, but posZ is not at startPosition.
                    // Prepare to move pen to Z start Position.
                    move.endZ = iBuffer[iBufferReadIndex].startZ;
                    move.dirZ = (posZ < move.endZ ? 1 : -1);
                    move.steps = (double) abs(move.endZ - posZ);
                    move.step = 0;
                    drawState = DrawState::MoveZ;
                    statusFunction = StatusFunction::Moving;
                    break;
                }
                else
                {
                    // Not at correct location yet. Is pen up ?
                    if (posZ == posPenUpZ)
                    {
                        // Pen is up.
                        // Set up a new movement to go to XY Location
                        move.endX = iBuffer[iBufferReadIndex].startX;
                        move.endY = iBuffer[iBufferReadIndex].startY;
                        move.deltaX = abs(move.endX - posX);
                        move.deltaY = -abs(move.endY - posY);
                        move.dirX = (posX < move.endX ? 1 : -1);
                        move.dirY = (posY < move.endY ? 1 : -1);
                        move.error = move.deltaX + move.deltaY;
                        if (move.deltaX > -move.deltaY)
                        {
                            move.steps = (double)move.deltaX;
                        }
                        else
                        {
                            move.steps = -(double)move.deltaY;
                        }
                        move.step = 0;
                        drawState = DrawState::MoveXY;
                        statusFunction = StatusFunction::Moving;
                        break;
                    }
                    else
                    {
                        // Prepare to lift pen.
                        move.endZ = posPenUpZ;
                        move.dirZ = (posZ < move.endZ ? 1 : -1);
                        move.steps = (double) abs(move.endZ - posZ);
                        move.step = 0;
                        drawState = DrawState::MoveZ;
                        statusFunction = StatusFunction::Moving;
                        break;
                    }
                }
            }
        }
        break;
    }

    /// ===============================================
    case (DrawState::MoveZ):
    {
        StepMotors();

        if (posZ == move.endZ)
        {
            // Arrived at Z destination

            // Serial.print(move.steps);
            // Serial.print(" / ");
            // Serial.println(move.step);

            drawState = DrawState::Choose;
            statusFunction = StatusFunction::Waiting;

            /// No urgency to continue straight on need to move // lift pen etc
            break;
        }

        if (posZ != move.endZ)
        {
            StepZ(move.dirZ);
        }

        double t = -fabs(move.steps * 0.5 - move.step) + move.steps * 0.5;
        machineSpeed = max(moveSpeed + accelerationRange - (t / (t + accelerationFactor)) * accelerationRange, moveSpeed);
        move.step++;

        SetDirectionsAndLimits();
        break;
    }

    /// ===============================================
    case (DrawState::MoveXY):
    {
        StepMotors();

        if (posX == move.endX && posY == move.endY)
        {
            // Arrived at XY destination

            // Serial.print(move.steps);
            // Serial.print(" / ");
            // Serial.println(move.step);

            drawState = DrawState::Choose;
            statusFunction = StatusFunction::Waiting;

            /// No urgency to continue straight on need to move // lift pen etc
            break;
        }

        if (posX != move.endX && posY != move.endY)
        {
            if (2 * move.error <= move.deltaX)
            {
                move.error += move.deltaX;
                // stepY??
                StepY(move.dirY);
            }
            if (2 * move.error >= move.deltaY)
            {
                move.error += move.deltaY;
                // stepX??
                StepX(move.dirX);
            }
        }
        else
        {
            /// Either X or Y has reached its final position, if anything
            /// remains, it must be a straight line.

            if (posX != move.endX)
            {
                StepX(move.dirX);
            }

            if (posY != move.endY)
            {
                StepY(move.dirY);
            }
        }
        double t = -fabs(move.steps * 0.5 - move.step) + move.steps * 0.5;
        machineSpeed = max(moveSpeed + accelerationRange - (t / (t + accelerationFactor)) * accelerationRange, moveSpeed);
        move.step++;

        SetDirectionsAndLimits();
        break;
    }
    /// ================================================
    case (DrawState::Draw):
    {
        StepMotors();

        if (posX == iBuffer[iBufferReadIndex].endX && posY == iBuffer[iBufferReadIndex].endY && posZ == iBuffer[iBufferReadIndex].endZ)
        {
            /// Done drawing current line!

            // Serial.print(iBuffer[iBufferReadIndex].steps);
            // Serial.print(" / ");
            // Serial.println(drawStep);

            iBufferReadIndex = (iBufferReadIndex + 1) & 63;
            drawState = DrawState::Choose;

            /// Continue straight on to drawing by calling Draw() again,
            /// to smoothly connect multiple segments,
            /// but make sure to break and beware of endless loops!!
            Draw();
            break;
        }

        if (iBuffer[iBufferReadIndex].type == 1)
        {
            CalculateStraightLine3D();
        }

        if (iBuffer[iBufferReadIndex].type == 2)
        {
            if (iBuffer[iBufferReadIndex].projection == 1)
            {
                CalculateQuadBezier3DXY();
            }

            if (iBuffer[iBufferReadIndex].projection == 2)
            {
                CalculateQuadBezier3DXZ();             
            }

            if (iBuffer[iBufferReadIndex].projection == 3)
            {               
                CalculateQuadBezier3DYZ();             
            }
        }

        double t = -fabs(iBuffer[iBufferReadIndex].steps*0.5 - draw.step) + iBuffer[iBufferReadIndex].steps*0.5;
        machineSpeed = max(drawSpeed + accelerationRange - (t / (t + accelerationFactor))*accelerationRange, drawSpeed);

        if (iBuffer[iBufferReadIndex].acceleration == 0) // single
        {
        }
        if (iBuffer[iBufferReadIndex].acceleration == 1) // start
        {
        }
        if (iBuffer[iBufferReadIndex].acceleration == 2) // continue
        {
        }
        if (iBuffer[iBufferReadIndex].acceleration == 3) // stop
        {
        }

        SetDirectionsAndLimits();
        if (draw.step < iBuffer[iBufferReadIndex].steps)
            draw.step++;

        break;
    }
    }
}

FASTRUN void CalculateStraightLine3D()
{
    // Draw Straight Line

    draw.errorX -= draw.deltaX;
    if (draw.errorX < 0)
    {
        draw.errorX += draw.deltaMax;
        StepX(iBuffer[iBufferReadIndex].dirX);
    }

    draw.errorY -= draw.deltaY;
    if (draw.errorY < 0)
    {
        draw.errorY += draw.deltaMax;
        StepY(iBuffer[iBufferReadIndex].dirY);
    }

    draw.errorZ -= draw.deltaZ;
    if (draw.errorZ < 0)
    {
        draw.errorZ += draw.deltaMax;
        StepZ(iBuffer[iBufferReadIndex].dirZ);
    }
}

FASTRUN void CalculateQuadBezier3DXY()
{
    if (posX != iBuffer[iBufferReadIndex].endX && posY != iBuffer[iBufferReadIndex].endY)
    {
        bool do_step_x = 2 * draw.error - draw.deltaY > 0;
        bool do_step_y = 2 * draw.error - draw.deltaX < 0;  

        if (do_step_x)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
            draw.deltaX -= iBuffer[iBufferReadIndex].deltaXY;
            draw.deltaY += iBuffer[iBufferReadIndex].deltaYY;
            draw.error += draw.deltaY;
            draw.errorZ -= draw.deltaXZ;
        }

        if (do_step_y)
        {        
            StepY(iBuffer[iBufferReadIndex].dirY);
            draw.deltaY -= iBuffer[iBufferReadIndex].deltaXY;
            draw.deltaX += iBuffer[iBufferReadIndex].deltaXX;
            draw.error += draw.deltaX;
            draw.errorZ -= draw.deltaYZ;
        }

        if (draw.errorZ < 0)
        {
            StepZ(iBuffer[iBufferReadIndex].dirZ);
            draw.errorZ += draw.deltaZ;
        }
    } else {
        if (posX != iBuffer[iBufferReadIndex].endX || posY != iBuffer[iBufferReadIndex].endY || posZ != iBuffer[iBufferReadIndex].endZ)
        {
            if (!draw.endStage) {
                // Prepare remaining part of curve as a straight line
                draw.deltaX = abs(draw.endX - posX);
                draw.deltaY = abs(draw.endX - posY);
                draw.deltaZ = abs(draw.endX - posZ);
                draw.deltaMax = max(draw.deltaZ, max(draw.deltaX, draw.deltaY));
                draw.errorX = draw.deltaMax / 2;
                draw.errorY = draw.deltaMax / 2;
                draw.errorZ = draw.deltaMax / 2;                
                draw.endStage=true;
            }
            CalculateStraightLine3D();
        }
    }
}

FASTRUN void CalculateQuadBezier3DXZ()
{

}

FASTRUN void CalculateQuadBezier3DYZ()
{

}

/*
FASTRUN void CalculateQuadBezier()
{
    // Draw Quadratic Bezier
    if (posX != iBuffer[iBufferReadIndex].endX && posY != iBuffer[iBufferReadIndex].endY)
    {
        bool do_step_x = 2 * drawError - drawDeltaX >= 0;
        bool do_step_y = 2 * drawError - drawDeltaY <= 0;
        if (do_step_x)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
            drawDeltaY -= iBuffer[iBufferReadIndex].deltaXY;
            drawDeltaX += iBuffer[iBufferReadIndex].deltaXX;
            drawError += drawDeltaX;
        }
        if (do_step_y)
        {
            StepY(iBuffer[iBufferReadIndex].dirY);
            drawDeltaX -= iBuffer[iBufferReadIndex].deltaXY;
            drawDeltaY += iBuffer[iBufferReadIndex].deltaYY;
            drawError += drawDeltaY;
        }
    }
    else
    {
        // At least x or y has reached its final position, if anything remains,
        // it must be a straight line.
        if (posX != iBuffer[iBufferReadIndex].endX)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
        }

        if (posY != iBuffer[iBufferReadIndex].endY)
        {
            StepY(iBuffer[iBufferReadIndex].dirY);
        }
    }
}

FASTRUN void CalculateStraightLine()
{
    // Draw Straight Line
    if (posX != iBuffer[iBufferReadIndex].endX && posY != iBuffer[iBufferReadIndex].endY)
    {
        if (2 * drawError <= drawDeltaX)
        {
            drawError += drawDeltaX;
            // stepY??
            StepY(iBuffer[iBufferReadIndex].dirY);
        }
        if (2 * drawError >= drawDeltaY)
        {
            drawError += drawDeltaY;
            // stepX??
            StepX(iBuffer[iBufferReadIndex].dirX);
        }
    }
    else
    {
        // At least x or y has reached its final position, if anything remains,
        // it must be a straight line.
        if (posX != iBuffer[iBufferReadIndex].endX)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
        }

        if (posY != iBuffer[iBufferReadIndex].endY)
        {
            StepY(iBuffer[iBufferReadIndex].dirY);
        }
    }
}
*/

FASTRUN void Home()
{
    switch (homeState)
    {
    case (HomeState::None):
    {
        break;
    }
    /// ================================================
    case (HomeState::Limit):
    {
        StepMotors();

        /// Is there a request to Stop homeing?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            mapHeightState = MapHeightState::None;
            // Do not execute the rest of this case.
            break;
        }

        if ((switches[swY1Start].pressed || switches[swY2Start].pressed) && switches[swXStart].pressed && switches[swZStart].pressed)
        {
            /// All limit switches hit,
            /// proceed to margin position
            M1_pos = 0;
            M2_pos = 0;
            M3_pos = 0;
            M4_pos = 0;
            posX = 0;
            posY = 0;
            posZ = 0;
            homeState = HomeState::Zero;
            statusFunction = StatusFunction::Homing;
        }
        else
        {
            M1_direction = -1;
            stepM1 = true;
            M2_direction = -1;
            stepM2 = true;
            M3_direction = -1;
            stepM3 = true;
            M4_direction = -1;
            stepM4 = true;
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed;
        break;
    }
    /// ================================================
    case (HomeState::Zero):
    {
        StepMotors();

        /// Is there a request to Stop homeing?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            mapHeightState = MapHeightState::None;
            // Do not execute the rest of this case.
            break;
        }

        if (M1_pos == 13581 && M2_pos == 13581 && M3_pos == 13581 && M4_pos == 13581)
        {
            /// Margin reached at 1 cm on axis,
            /// reset coordinates.
            M1_pos = 0;
            M2_pos = 0;
            M3_pos = 0;
            M4_pos = 0;
            posX = 0;
            posY = 0;
            posZ = 0;
            homeState = HomeState::Done;
            statusFunction = StatusFunction::Waiting;
        }
        else
        {
            if (M1_pos > 13581)
            {
                M1_direction = -1;
                stepM1 = true;
            }
            if (M1_pos < 13581)
            {
                M1_direction = 1;
                stepM1 = true;
            }

            if (M2_pos > 13581)
            {
                M2_direction = -1;
                stepM2 = true;
            }
            if (M2_pos < 13581)
            {
                M2_direction = 1;
                stepM2 = true;
            }

            if (M3_pos > 13581)
            {
                M3_direction = -1;
                stepM3 = true;
            }
            if (M3_pos < 13581)
            {
                M3_direction = 1;
                stepM3 = true;
            }

            if (M4_pos > 13581)
            {
                M4_direction = -1;
                stepM4 = true;
            }

            if (M4_pos < 13581)
            {
                M4_direction = 1;
                stepM4 = true;
            }
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed;
        break;
    }
    /// ================================================
    case (HomeState::Done):
    {
        requestedMode = Mode::None;
        activeMode = Mode::None;
        homeState = HomeState::None;
        break;
    }
    }
}

FASTRUN void MapHeight()
{
    switch (mapHeightState)
    {
    case (MapHeightState::None):
    {
        break;
    }
    /// ================================================
    case (MapHeightState::Choose):
    {
        heightMapIndex = -1;

        /// Get index of the First HeightMap position that is not set.
        for (int i = (HeightMapSize - 1); i >= 0; i--)
        {
            if (HeightMap[i] == INT64_MIN)
            {
                heightMapIndex = i;
            }
        }

        if (heightMapIndex == -1)
        {
            /// HeightMap is set completely.
            if (posZ != 0)
            {
                /// Move up first, then return here.
                mapHeightState = MapHeightState::MoveUp;
                statusFunction = StatusFunction::Moving;
            }
            else
            {
                /// HeightMap complete and safely moved up.
                mapHeightState = MapHeightState::Done;
            }
        }
        else
        {
            /// heightMapIndex needs mapping.
            if (posZ != 0)
            {
                /// Move up first, then return here.
                mapHeightState = MapHeightState::MoveUp;
                statusFunction = StatusFunction::Moving;
            }
            else
            {
                /// Is there a request to Stop mapping?
                if (requestedMode == Mode::Stop)
                {
                    activeMode = Mode::Stop;
                    mapHeightState = MapHeightState::None;
                    // Do not execute the rest of this case.
                    break;
                }

                /// Ready to move to the correct position
                move.endX = (XMAX / (HeightMapWidth - 1)) * (heightMapIndex % HeightMapWidth);
                move.endY = (YMAX / (HeightMapHeight - 1)) * (heightMapIndex / HeightMapWidth);

                if (posX == move.endX && posY == move.endY)
                {
                    /// Mapping position already reached, proceed to map height.
                    mapHeightState = MapHeightState::MoveDown;
                    statusFunction = StatusFunction::Mapping;
                }
                else
                {
                    Serial.print("moving to:");
                    Serial.print(move.endX);
                    Serial.print(" , ");
                    Serial.println(move.endY);
                    mapHeightState = MapHeightState::StartMoveXY;
                    statusFunction = StatusFunction::Moving;
                }
            }
        }
        break;
    }

    /// ================================================
    case (MapHeightState::MoveUp):
    {
        StepMotors();

        if (posZ == 0)
        {
            /// Switch back to state: Choose
            mapHeightState = MapHeightState::Choose;
        }
        else
        {
            if (posZ > 0)
                StepZ(-1);

            if (posZ < 0)
                StepZ(1);
        }
        machineSpeed = normalSpeed;

        SetDirectionsAndLimits();
        break;
    }

    /// ================================================
    case (MapHeightState::StartMoveXY):
    {
        /// Set up a new movement
        move.deltaX = abs(move.endX - posX);
        move.deltaY = -abs(move.endY - posY);
        move.dirX = (posX < move.endX ? 1 : -1);
        move.dirY = (posY < move.endY ? 1 : -1);
        move.error = move.deltaX + move.deltaY;
        if (move.deltaX > -move.deltaY)
        {
            move.steps = (double)move.deltaX;
        }
        else
        {
            move.steps = -(double)move.deltaY;
        }
        move.step = 0;
        mapHeightState = MapHeightState::MoveXY;
        break;
    }

    /// ================================================
    case (MapHeightState::MoveXY):
    {
        StepMotors();

        if (posX == move.endX && posY == move.endY)
        {
            mapHeightState = MapHeightState::Choose;
            machineSpeed = normalSpeed;
        }
        else
        {
            if (posX != move.endX && posY != move.endY)
            {
                if (2 * move.error <= move.deltaX)
                {
                    move.error += move.deltaX;
                    StepY(move.dirY);
                }
                if (2 * move.error >= move.deltaY)
                {
                    move.error += move.deltaY;
                    StepX(move.dirX);
                }
            }
            else
            {
                ///  At least x or y has reached its final position,
                /// if anything remains, it must be a straight line.
                if (posX != move.endX)
                {
                    StepX(move.dirX);
                }

                if (posY != move.endY)
                {
                    StepY(move.dirY);
                }
            }
            double speedRamp = min(-fabs(move.steps*0.5 - move.step) + move.steps*0.5, 12000.0) / 400.0;
            machineSpeed = max(7.0 + 30.0 - speedRamp, 7.0);
            move.step++;
        }
        SetDirectionsAndLimits();
        break;
    }
    /// ================================================
    case (MapHeightState::MoveDown):
    {
        /// Hit the floor
        StepMotors();

        if (switches[swZEnd].pressed)
        {
            /// add value to sum
            HeightMapProbeResults[heightMapCount] = posZ;
            heightMapCount++;

            if (heightMapCount >= heightMapMaxCount)
            {

                /// Get the index for the heighest and lowest value.
                int indexHeighest = 0;
                int indexLowest = 0;
                for (int i = 1; i < heightMapCount; i++)
                {
                    if (HeightMapProbeResults[i] > HeightMapProbeResults[indexHeighest])
                        indexHeighest = i;
                    if (HeightMapProbeResults[i] < HeightMapProbeResults[indexLowest])
                        indexLowest = i;
                }
                int sumCount = 0;
                uint64_t sum = 0;
                for (int i = 0; i < heightMapCount; i++)
                {
                    if (i == indexHeighest || i == indexLowest)
                    {
                        /// skip
                    }
                    else
                    {
                        sum += HeightMapProbeResults[i];
                        sumCount++;
                    }
                }
                uint64_t height = 0;
                if (heightMapIndex == 0)
                {
                    height = (sum / sumCount);
                }
                else
                {
                    height = HeightMap[0] - (sum / sumCount);
                }

                HeightMap[heightMapIndex] = height;

                /// store in EEPROM
                byte64 b64 = {};
                b64.value = height;
                for (int e = 0; e < 8; e++)
                {
                    EEPROM.write(heightMapIndex * 8 + e, b64.bytes[e]);
                }

                // reset count for next index.
                Serial.print("Height set for index:");
                Serial.print(heightMapIndex);
                Serial.print(" to:");
                Serial.println(HeightMap[heightMapIndex]);
                heightMapCount = 0;
            }

            /// Check if we need to map another position in the map.
            /// Switch to state: Choose
            mapHeightState = MapHeightState::Choose;
            machineSpeed = normalSpeed;
            break;
        }
        StepZ(1);
        machineSpeed = normalSpeed;

        SetDirectionsAndLimits();
        break;
    }

    /// ================================================
    case (MapHeightState::Done):
    {
        Serial.println("Done with heightmap!");
        requestedMode = Mode::None;
        activeMode = Mode::None;
        mapHeightState = MapHeightState::None;
        break;
    }
    }
}

FASTRUN void StepMotors()
{
    if (stepM1)
    {
        digitalToggleFast(M1_stepPin);
        M1_pos += M1_direction;
    }
    if (stepM2)
    {
        digitalToggleFast(M2_stepPin);
        M2_pos += M2_direction;
    }
    if (stepM3)
    {
        digitalToggleFast(M3_stepPin);
        M3_pos += M3_direction;
    }
    if (stepM4)
    {
        digitalToggleFast(M4_stepPin);
        M4_pos += M4_direction;
    }

    /// Reset step triggers.
    stepM1 = false;
    stepM2 = false;
    stepM3 = false;
    stepM4 = false;
    stepM5 = false;

    /// Update local positions.
    posX = M3_pos;
    posY = M1_pos;
    posZ = M4_pos;
}

FASTRUN void SetDirectionsAndLimits()
{
    /// Switches for Y1 and Y2 are linked.
    /// If a switch or switch wire fails,
    /// left and right motors will both moving in that direction.

    if (M1_direction == 1)
    {
        digitalWriteFast(M1_dirPin, LOW); // Motor reverse mount
        if (switches[swY1End].pressed || switches[swY2End].pressed)
            stepM1 = false;
    }
    if (M1_direction == -1)
    {
        digitalWriteFast(M1_dirPin, HIGH); // Motor reverse mount
        if (switches[swY1Start].pressed || switches[swY2Start].pressed)
            stepM1 = false;
    }

    if (M2_direction == 1)
    {
        digitalWriteFast(M2_dirPin, HIGH);
        if (switches[swY1End].pressed || switches[swY2End].pressed)
            stepM2 = false;
    }
    if (M2_direction == -1)
    {
        digitalWriteFast(M2_dirPin, LOW);
        if (switches[swY1Start].pressed || switches[swY2Start].pressed)
            stepM2 = false;
    }

    if (M3_direction == 1)
    {
        digitalWriteFast(M3_dirPin, HIGH);
        if (switches[swXEnd].pressed)
            stepM3 = false;
    }
    if (M3_direction == -1)
    {
        digitalWriteFast(M3_dirPin, LOW);
        if (switches[swXStart].pressed)
            stepM3 = false;
    }

    if (M4_direction == 1)
    {
        digitalWriteFast(M4_dirPin, LOW); // Motor reverse mount
        if (switches[swZEnd].pressed)
            stepM4 = false;
    }
    if (M4_direction == -1)
    {
        digitalWriteFast(M4_dirPin, HIGH); // Motor reverse mount
        if (switches[swZStart].pressed)
            stepM4 = false;
    }

    // Important if Z-limit has hit the deck, do not allow any movement in XY direction
    if (switches[swZEnd].pressed)
    {
        stepM1 = false;
        stepM2 = false;
        stepM3 = false;
    }
}

FASTRUN void StepX(int8_t dir)
{
    if (dir > 0)
    {
        M3_direction = 1;
    }
    else
    {
        M3_direction = -1;
    }
    stepM3 = true;
}

FASTRUN void StepY(int8_t dir)
{
    if (dir > 0)
    {
        M1_direction = 1;
        M2_direction = 1;
    }
    else
    {
        M1_direction = -1;
        M2_direction = -1;
    }
    stepM1 = true;
    stepM2 = true;
}

FASTRUN void StepZ(int8_t dir)
{
    if (dir > 0)
    {
        M4_direction = 1;
    }
    else
    {
        M4_direction = -1;
    }
    stepM4 = true;
}

FASTRUN void setCurrent(int cur)
{
    stallGuardConfig.current_scale = min(31, max(0, cur));
    setTMC262Register(stallGuardConfig.bytes, M1_csPin);
    setTMC262Register(stallGuardConfig.bytes, M2_csPin);
    setTMC262Register(stallGuardConfig.bytes, M3_csPin);
}

FASTRUN void DebounceSwitches()
{
    /// Fast press and slow release debounce.
    /// Press debounce is added in 8 StepLoops
    /// Release debounce is performed every 300000 cycles

    /// ===================== Fast PRESS DEBOUNCE =====================

    for (int s = 0; s < numSwitches; s++)
    {
        switches[s].onBounce += digitalReadFast(switches[s].hwPin);

        if (switches[s].onBounce > 7 && switches[s].pressed == false)
        {
            switches[s].offBounce = __UINT8_MAX__;
            switches[s].pressed = true;
        }
    }

    /// Check for panic.
    if (switches[swPanic].pressed)
    {
        // PANIC Button! Halting ALL MOTORS Right now
        digitalWriteFast(M1_M2_M3_ennPin, 1);
        digitalWriteFast(M4_M5_enPin, 1);
    }

    // ======== SLOW RELEASE DEBOUNCE ========== //

    /// Every 2 ms ?
    if (debounceCounter > 300000)
    {
        for (int s = 0; s < numSwitches; s++)
        {
            switches[s].offBounce <<= 1;
            switches[s].offBounce |= digitalReadFast(switches[s].hwPin);

            if (switches[s].offBounce == 0)
            {
                switches[s].pressed = false;
                switches[s].onBounce = 0;
            }
        }
    }
}

void updateStepperStatus()
{
    status_M1 = setTMC262Register(driverControl.bytes, M1_csPin);
    status_M2 = setTMC262Register(driverControl.bytes, M2_csPin);
    status_M3 = setTMC262Register(driverControl.bytes, M3_csPin);

    // if ((bool)status_M1.Stalled)
    //     Serial.println("M1 Stallguard status: Stalled");
    // if ((bool)status_M2.Stalled)
    //     Serial.println("M2 Stallguard status: Stalled");
    // if ((bool)status_M3.Stalled)
    //     Serial.println("M3 Stallguard status: Stalled");

    // if ((bool) status_M1.OpenLoad_A) Serial.println("M1 OpenLoad detected: COIL A");
    // if ((bool) status_M2.OpenLoad_A) Serial.println("M2 OpenLoad detected: COIL A");
    // if ((bool) status_M3.OpenLoad_A) Serial.println("M3 OpenLoad detected: COIL A");

    // if ((bool) status_M1.OpenLoad_B) Serial.println("M1 OpenLoad detected: COIL B");
    // if ((bool) status_M2.OpenLoad_B) Serial.println("M2 OpenLoad detected: COIL B");
    // if ((bool) status_M3.OpenLoad_B) Serial.println("M3 OpenLoad detected: COIL B");

    if ((bool)status_M1.OverTemp_Warning)
        Serial.println("M1 Over Temperature Warning!");
    if ((bool)status_M2.OverTemp_Warning)
        Serial.println("M2 Over Temperature Warning!");
    if ((bool)status_M3.OverTemp_Warning)
        Serial.println("M3 Over Temperature Warning!");

    if ((bool)status_M1.OverTemp_Shutdown)
        Serial.println("M1 Over Temperature Shutdown!");
    if ((bool)status_M2.OverTemp_Shutdown)
        Serial.println("M2 Over Temperature Shutdown!");
    if ((bool)status_M3.OverTemp_Shutdown)
        Serial.println("M3 Over Temperature Shutdown!");

    // if ((bool)status_M1.StandStill)
    //     Serial.println("M1 StandStill detected.");
    // if ((bool)status_M2.StandStill)
    //     Serial.println("M2 StandStill detected.");
    // if ((bool)status_M3.StandStill)
    //     Serial.println("M3 StandStill detected.");

    if ((bool)status_M1.Short_A)
        Serial.println("M1 Short on COIL A detected.");
    if ((bool)status_M2.Short_A)
        Serial.println("M2 Short on COIL A detected.");
    if ((bool)status_M3.Short_A)
        Serial.println("M3 Short on COIL A detected.");

    if ((bool)status_M1.Short_B)
        Serial.println("M1 Short on COIL B detected.");
    if ((bool)status_M2.Short_B)
        Serial.println("M2 Short on COIL B detected.");
    if ((bool)status_M3.Short_B)
        Serial.println("M3 Short on COIL B detected.");
}

/// ======== Configuration for Switches en Motor Drivers ==========

void configureSwitches()
{
    switches[swPanic].hwPin = pinPanic;
    switches[swY1Start].hwPin = pinY1Start;
    switches[swY1End].hwPin = pinY1End;
    switches[swY2Start].hwPin = pinY2Start;
    switches[swY2End].hwPin = pinY2End;
    switches[swXStart].hwPin = pinXStart;
    switches[swXEnd].hwPin = pinXEnd;
    switches[swZStart].hwPin = pinZStart;
    switches[swZEnd].hwPin = pinZEnd;

    for (int s = 0; s < numSwitches; s++)
    {
        pinMode(switches[s].hwPin, INPUT_PULLUP);
        switches[s].onBounce = 0;
        switches[s].offBounce = __UINT8_MAX__;
        switches[s].pressed = false;
    }
}

void configureStepperDrivers()
{
    Serial.println("Configuring drivers..");
    // disable drives
    pinMode(M1_M2_M3_ennPin, OUTPUT);
    pinMode(M4_M5_enPin, OUTPUT);

    digitalWriteFast(M1_M2_M3_ennPin, 1); // Inverted input: LOW means enable
    digitalWriteFast(M4_M5_enPin, 1);

    pinMode(M1_dirPin, OUTPUT);
    pinMode(M1_stepPin, OUTPUT);
    pinMode(M1_csPin, OUTPUT);
    pinMode(M1_sgPin, INPUT);

    pinMode(M2_dirPin, OUTPUT);
    pinMode(M2_stepPin, OUTPUT);
    pinMode(M2_csPin, OUTPUT);
    pinMode(M2_sgPin, INPUT);

    pinMode(M3_dirPin, OUTPUT);
    pinMode(M3_stepPin, OUTPUT);
    pinMode(M3_csPin, OUTPUT);
    pinMode(M3_sgPin, INPUT);

    pinMode(M4_dirPin, OUTPUT);
    pinMode(M4_stepPin, OUTPUT);
    pinMode(M4_csPin, OUTPUT);

    pinMode(M5_dirPin, OUTPUT);
    pinMode(M5_stepPin, OUTPUT);
    pinMode(M5_csPin, OUTPUT);

    digitalWriteFast(M1_csPin, 1);
    digitalWriteFast(M2_csPin, 1);
    digitalWriteFast(M3_csPin, 1);
    digitalWriteFast(M4_csPin, 1);
    digitalWriteFast(M5_csPin, 1);

    SPI1.begin();

    // M1, M2, M3 : TMC262-BOB
    driverConfig.address = 7;
    driverConfig.test_mode = 0;
    driverConfig.slope_high_side = 3;
    driverConfig.slope_low_side = 3;
    driverConfig.short_GND_protection = 0;
    driverConfig.short_detection_delay = 0;
    driverConfig.Vsense = 0;
    driverConfig.readInfo = 0;

    setTMC262Register(driverConfig.bytes, M1_csPin);
    setTMC262Register(driverConfig.bytes, M2_csPin);
    setTMC262Register(driverConfig.bytes, M3_csPin);

    // chopperConfig.address = 4;
    // chopperConfig.blanking_time = 2;
    // chopperConfig.chopper_mode = 0;
    // chopperConfig.random_t_off = 0;
    // chopperConfig.hysteresis_decrement_interval = 0;
    // chopperConfig.hysteresis_end_value = 3;
    // chopperConfig.hysteresis_start_value = 3;
    // chopperConfig.Toff = 4;

    chopperConfig.address = 4;
    chopperConfig.blanking_time = 1;
    chopperConfig.chopper_mode = 0;
    chopperConfig.random_t_off = 0;
    chopperConfig.hysteresis_decrement_interval = 0;
    chopperConfig.hysteresis_end_value = 3;
    chopperConfig.hysteresis_start_value = 3;
    chopperConfig.Toff = 4;

    setTMC262Register(chopperConfig.bytes, M1_csPin);
    setTMC262Register(chopperConfig.bytes, M2_csPin);
    setTMC262Register(chopperConfig.bytes, M3_csPin);

    stallGuardConfig.address = 6;
    stallGuardConfig.filter = 0;
    stallGuardConfig.stall_threshold = 4; // 2s complement 0..63 = 0..63 / 64..127 = -63..-0
    stallGuardConfig.current_scale = 0;

    setTMC262Register(stallGuardConfig.bytes, M1_csPin);
    setTMC262Register(stallGuardConfig.bytes, M2_csPin);
    setTMC262Register(stallGuardConfig.bytes, M3_csPin);

    coolStepConfig.address = 5;
    coolStepConfig.min_current = 0;
    coolStepConfig.current_decrement_speed = 0;
    coolStepConfig.upper_coolstep_treshold = 0;
    coolStepConfig.current_increment_size = 0;
    coolStepConfig.lower_coolstep_treshold = 0; // disable coolstep;

    setTMC262Register(coolStepConfig.bytes, M1_csPin);
    setTMC262Register(coolStepConfig.bytes, M2_csPin);
    setTMC262Register(coolStepConfig.bytes, M3_csPin);

    driverControl.address = 0;
    driverControl.interpolation = 0;
    driverControl.double_edge_step = 1;
    driverControl.microstep_resolition = 0;

    setTMC262Register(driverControl.bytes, M1_csPin);
    setTMC262Register(driverControl.bytes, M2_csPin);
    setTMC262Register(driverControl.bytes, M3_csPin);

    // M4,M5 TMC2130 Fystec

    // Write GCONF
    TMC2130::GCONF globalConfig = {0};
    globalConfig.diag1_stall = 1;
    globalConfig.StealthChop = 0;
    setTMC2130Register(TMC2130::registers::reg_GCONF, globalConfig.data, M4_csPin);
    // setTMC2130Register(TMC2130::registers::reg_GCONF, globalConfig.data, M5_csPin);

    // Write CHOPCONF
    TMC2130::CHOPCONF chopperConfig = {0};
    chopperConfig.mode = 0;
    chopperConfig.mres = 0;   // 256 microsteps
    chopperConfig.dedge = 1;  // double edge step
    chopperConfig.vsense = 0; // 0 = fullpower
    chopperConfig.blanktime = 2;
    chopperConfig.hend = 1;
    chopperConfig.hstart = 4;
    chopperConfig.toff = 3;
    setTMC2130Register(TMC2130::registers::reg_CHOPCONF, chopperConfig.data, M4_csPin);
    // setTMC2130Register(TMC2130::registers::reg_CHOPCONF, chopperConfig.data, M5_csPin);

    // Write IHOLD_IRUN
    TMC2130::IHOLD_IRUN CurrentControl = {0};
    CurrentControl.IHOLD = 4;
    CurrentControl.IRUN = 12;
    CurrentControl.IHOLDDELAY = 15;
    setTMC2130Register(TMC2130::registers::reg_IHOLD_IRUN, CurrentControl.data, M4_csPin);
    // setTMC2130Register(TMC2130::registers::reg_IHOLD_IRUN, CurrentControl.data, M5_csPin);

    // Write TPOWERDOWN
    TMC2130::TPOWERDOWN PowerdownDelay = {0};
    PowerdownDelay.TPOWERDOWN = 10;
    setTMC2130Register(TMC2130::registers::reg_TPOWERDOWN, PowerdownDelay.data, M4_csPin);
    // setTMC2130Register(TMC2130::registers::reg_TPOWERDOWN, PowerdownDelay.data, M5_csPin);

    /*
    //READ GCONF
    SPI1.beginTransaction(tmc2130_spi_config);
    digitalWriteFast(M4_csPin,0);
    delayNanoseconds(spi_cs_delay);
    d1 = SPI1.transfer(TMC2130::registers::reg_GCONF);
    d2 = SPI1.transfer32(0);
    digitalWriteFast(M4_csPin,1);
    SPI1.endTransaction();

    //READ GCONF
    SPI1.beginTransaction(tmc2130_spi_config);
    digitalWriteFast(M4_csPin,0);
    delayNanoseconds(spi_cs_delay);
    d1 = SPI1.transfer(TMC2130::registers::reg_GCONF);
    d2 = SPI1.transfer32(0);
    digitalWriteFast(M4_csPin,1);
    SPI1.endTransaction();

    Serial.println(d1);
    Serial.println(d2);

    Serial.println("GCONF");
    globalConfig.data = d2;
    Serial.println((bool) globalConfig.I_scale_analog);
    Serial.println((bool) globalConfig.Internal_Rsense);
    Serial.println((bool) globalConfig.StealthChop);
    Serial.println((bool) globalConfig.shaft);
    Serial.println((bool) globalConfig.diag0_error);
    Serial.println((bool) globalConfig.diag0_otpw);
    Serial.println((bool) globalConfig.diag0_stall);
    Serial.println((bool) globalConfig.diag1_stall);
    Serial.println((bool) globalConfig.diag1_index);
    Serial.println((bool) globalConfig.diag1_onstate);
    Serial.println((bool) globalConfig.diag1_steps_skipped);
    Serial.println((bool) globalConfig.diag0_int_pushpull);
    Serial.println((bool) globalConfig.diag1_int_pushpull);
    Serial.println((bool) globalConfig.small_hysteresis);
    Serial.println((bool) globalConfig.stop_enable);
    Serial.println((bool) globalConfig.direct_mode);
    Serial.println((bool) globalConfig.test_mode);
    */
    // concept send 8bit register, send 32bit data
    // uint32_t data=0;
    // SPI1.transfer(TMC2130::registers::GCONF);
    // SPI1.transfer32(data);

    // digitalWriteFast(M1_dirPin,1);
    // digitalWriteFast(M2_dirPin,1);
    // digitalWriteFast(M3_dirPin,0);
    // digitalWriteFast(M4_dirPin,1);
    // digitalWriteFast(M5_dirPin,0);
}

TMC262::STATUS setTMC262Register(uint8_t bytes[3], int CSPin)
{
    TMC262::STATUS st = {0};
    SPI1.beginTransaction(tmc262_spi_config);
    digitalWriteFast(CSPin, 0);
    delayNanoseconds(spi_cs_delay);
    st.bytes[2] = SPI1.transfer(bytes[2]);
    st.bytes[1] = SPI1.transfer(bytes[1]);
    st.bytes[0] = SPI1.transfer(bytes[0]);
    digitalWriteFast(CSPin, 1);
    SPI1.endTransaction();
    return st;
}

TMC2130::SPI_STATUS setTMC2130Register(uint8_t address, uint32_t data, int CSPin)
{
    TMC2130::SPI_STATUS st = {0};
    SPI1.beginTransaction(tmc2130_spi_config);
    digitalWriteFast(CSPin, 0);
    delayNanoseconds(spi_cs_delay);
    st.value = SPI1.transfer(address | 0x80);
    SPI1.transfer32(data);
    digitalWriteFast(CSPin, 1);
    SPI1.endTransaction();
    return st;
}