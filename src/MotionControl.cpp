#include "MotionControl.h"
#include <EEPROM.h>
#include <SPI.h>

// volatile enum Action currentAction = action_none;

volatile State activeState = state_none;
volatile State requestedState = state_none;

// ======== CIRCULAR Buffer for draw instructions ==========//

volatile DrawInstruction iBuffer[64];
volatile uint8_t iBufferWriteIndex = 0;
volatile uint8_t iBufferReadIndex = 0;
volatile int64_t requestedInstruction = -1;
volatile int64_t receivedInstruction = -1;

// ======== Mapping Algorithm

MappingState mappingState = MappingState::MS_None;

// ======== Drawing variables as part of movement algorithms ==========//

int64_t HeightMap[HeightMapSize];
int HeightMapSetIndex = 0;

bool moving = true;
bool lineStarted = false;

int64_t drawDeltaX = 0;
int64_t drawDeltaY = 0;
int64_t drawError = 0;

int64_t moveEndX = 0;
int64_t moveEndY = 0;
uint8_t moveDirX;
uint8_t moveDirY;
int64_t moveDeltaX = 0;
int64_t moveDeltaY = 0;
int64_t moveError = 0;
double moveSteps = 0;
double moveStep = 0;

int64_t posX = 0;
int64_t posY = 0;
int64_t posZ = 0;

uint64_t sleepTimer = 0;

int workCurrent = 20;
int sleepCurrent = 0;

bool sleeping = false;

/// @brief Limit switch debounce cycle counter
uint32_t debounceCounter = 0;

/// ======== Limit switch hardware pins ========== ///

const int panic_pin = 23;
const int Limit_Y1_start_pin = 36;
const int Limit_Y1_end_pin = 35;
const int Limit_Y2_start_pin = 38;
const int Limit_Y2_end_pin = 37;
const int Limit_X_start_pin = 40;
const int Limit_X_end_pin = 39;
const int Limit_Z_start_pin = 41;
const int Limit_Z_end_pin = 14;

/// ======== Limit Switch Debounce Press (ON) counters ========== ///

uint8_t panic_on_count = 0;
uint8_t Limit_Y1_start_on_count = 0;
uint8_t Limit_Y1_end_on_count = 0;
uint8_t Limit_Y2_start_on_count = 0;
uint8_t Limit_Y2_end_on_count = 0;
uint8_t Limit_X_start_on_count = 0;
uint8_t Limit_X_end_on_count = 0;
uint8_t Limit_Z_start_on_count = 0;
uint8_t Limit_Z_end_on_count = 0;

/// ======== Limit Switch Debounce Release (OFF) counters ========== ///

uint8_t panic_off_count = __UINT8_MAX__;
uint8_t Limit_Y1_start_off_count = __UINT8_MAX__;
uint8_t Limit_Y1_end_off_count = __UINT8_MAX__;
uint8_t Limit_Y2_start_off_count = __UINT8_MAX__;
uint8_t Limit_Y2_end_off_count = __UINT8_MAX__;
uint8_t Limit_X_start_off_count = __UINT8_MAX__;
uint8_t Limit_X_end_off_count = __UINT8_MAX__;
uint8_t Limit_Z_start_off_count = __UINT8_MAX__;
uint8_t Limit_Z_end_off_count = __UINT8_MAX__;

/// ======== Limit Switch values ========== ///
volatile bool panic_switch = false;
volatile bool Limit_Y1_start = false;
volatile bool Limit_Y1_end = false;
volatile bool Limit_Y2_start = false;
volatile bool Limit_Y2_end = false;
volatile bool Limit_X_start = false;
volatile bool Limit_X_end = false;
volatile bool Limit_Z_start = false;
volatile bool Limit_Z_end = false;

/// Various bools to check limits, homing, mapheight, etc

volatile bool isHome = false;
volatile bool isZero = false;
volatile bool isMax = false;

volatile bool isZTop = false;
volatile bool isZBottom = false;

/// ======== prepared Streps ========== ///

bool stepM1 = false;
bool stepM2 = false;
bool stepM3 = false;
bool stepM4 = false;
bool stepM5 = false;

/// ========  Stepper Motor hardware pins ========== ///

SPISettings tmc262_spi_config(5000000, MSBFIRST, SPI_MODE3);
SPISettings tmc2130_spi_config(4000000, MSBFIRST, SPI_MODE3);
int spi_cs_delay = 50;

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

/// ======== Motion Control ========== ///

// Hardware interrupt (PIR) TIMER triggering MachineLoop Interrupt.
RoboTimer IRQTimer;

/// Interrupt iteration times (speeds).
float moveSpeed = 15.0f;
float drawSpeed = 15.0f;
float homeSpeed = 100.0f;
float normalSpeed = 100.0f;

/// @brief Interrupt iteration time (speed).
/// Interrupt type is always set to machineSpeed,
/// where machine speed is set depending on different
/// operations.
float machineSpeed = normalSpeed;

/// Track interrupt performance.
/// Measur highest MachineLoop interrupt time.
volatile uint32_t max_step_cycles = 0;

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

TMC262::DRVCONF driverConfig = {0};
TMC262::CHOPCONF chopperConfig = {0};
TMC262::SGCSCONF stallGuardConfig = {0};
TMC262::SMARTEN coolStepConfig = {0};
TMC262::DRVCTRL driverControl = {0};

/// ======== Motor status ========== ///

volatile uint8_t drawFunction = 0;
volatile int32_t drawIndex = 0;

TMC262::STATUS status_M1 = {0};
TMC262::STATUS status_M2 = {0};
TMC262::STATUS status_M3 = {0};

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
    }
    activeState = state_none;
    requestedState = state_none;

    setCurrent(sleepCurrent);

    /// Enable stepper drivers
    digitalWriteFast(M1_M2_M3_ennPin, 0);
    digitalWriteFast(M4_M5_enPin, 0);

    setCurrent(workCurrent);

    delay(500);

    requestedState = state_home;
    lineStarted = false;

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

    switch (activeState)
    {
    case State::state_draw:
    {
        /// Perform Motor Steps.
        /// As calculated in previous iteration and update positions.
        StepMotors();

        /// Reset step trigger and start next iteration.
        stepM1 = stepM2 = stepM3 = stepM4 = stepM5 = false;

        /// Check for end of line and Pause at end of line.
        if (lineStarted)
        {
            if (posX == iBuffer[iBufferReadIndex].endX && posY == iBuffer[iBufferReadIndex].endY)
            {
                /// Done drawing current line!
                iBufferReadIndex = (iBufferReadIndex + 1) & 63;
                drawFunction = 0;
                lineStarted = false;

                /// Is there a request to Pause drawing?
                if (requestedState == state_none)
                {
                    activeState = state_none;
                    // Do not execute the rest of this case.
                    break;
                }
            }
        }

        /// Are there active or new instructions that need to be drawn ?
        if (iBufferReadIndex != iBufferWriteIndex)
        {
            /// Calculate movement for the next step.
            CalculateDrawSteps();
            SetDirectionsAndLimits();

            machineSpeed = drawSpeed;

            if (sleeping)
            {
                /// Wake up
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
        }
        else
        {
            /// Nothing left to do right now.

            if (requestedState == state_eof)
            {
                /// End of File reached. Done drawing all instructions.
                requestedState = state_none;
                activeState = state_none;
            }
            else
            {
                /// Wait for an EOF or new instructions to arrive
                /// in the draw instruction buffer.
                if (!sleeping)
                {
                    sleepTimer++;
                    if (sleepTimer > 50000)
                    {
                        sleeping = true;
                        setCurrent(sleepCurrent);
                        sleepTimer = 0;
                    }
                }
                machineSpeed = 100.0f;
            }
        }
        break;
    }
    case State::state_home:
    {
        if (!isHome)
        {
            /// Perform Motor Steps.
            /// As calculated in previous iteration and update positions.
            StepMotors();

            /// Reset step trigger and start next iteration.
            stepM1 = stepM2 = stepM3 = stepM4 = stepM5 = false;

            CalculateHomeSteps();
            SetDirectionsAndLimits(); // Set PRE-STEP Direction &&& Check LIMITS

            machineSpeed = homeSpeed;

            if (sleeping)
            {
                /// Wake up
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
        }
        else
        {
            if (requestedState == state_home)
                requestedState = state_none;

            activeState = state_none;
        }
        break;
    }
    case State::state_mapheight:
    {

        MapHeight();

        if (sleeping) /// Wake up
        {
            setCurrent(workCurrent);
            sleeping = false;
        }
        sleepTimer = 0;

        break;
    }

    case State::state_clearheight:
    {
        /// clear Heightmap and EEPROM
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
        Serial.println("HeightMap cleared.");
        requestedState = State::state_none;
        activeState = State::state_none;
        break;
    }
    case State::state_none:
    {
        if (!sleeping)
        {
            sleepTimer++;
            if (sleepTimer > 50000)
            {
                sleeping = true;
                setCurrent(sleepCurrent);
            }
        }
        else
        {
            sleepTimer = 0;
        }

        if (requestedState == state_draw)
        {
            stepM1 = false;
            stepM2 = false;
            stepM3 = false;
            stepM4 = false;
            stepM5 = false;
            drawSpeed = 100.0f;
            activeState = state_draw;
        }

        if (requestedState == state_reset)
        {
            iBufferReadIndex = 0;
            iBufferWriteIndex = 0;
            requestedInstruction = -1;
            receivedInstruction = -1;
            drawIndex = 0;
            requestedState = state_none;
        }

        if (requestedState == state_home)
        {
            isHome = false;
            isZero = false;
            isMax = true;
            stepM1 = stepM2 = stepM3 = stepM4 = stepM5 = false;
            homeSpeed = 100.0f;
            activeState = state_home;
        }

        if (requestedState == State::state_mapheight)
        {
            stepM1 = stepM2 = stepM3 = stepM4 = stepM5 = false;
            mappingState = MappingState::MS_None;
            activeState = State::state_mapheight;
        }

        if (requestedState == State::state_clearheight)
        {
            activeState = State::state_clearheight;
        }

        machineSpeed = 100.0f;
        break;
    }
    default:
    {
        break;
    }
    }

    /// Debounce Switches and set button press or release triggers
    DebounceSwitches();

    /// Set STEP SPEED (Intertupt time) for next Iteration.
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

    IRQTimer.unsafe_update(cycles);

    /// Update the debounce timetracker
    debounceCounter += cycles;

    /// Update variables for time tracking interrupt performance.
    uint32_t endtime = ARM_DWT_CYCCNT;
    if (endtime - starttime > max_step_cycles)
    {
        max_step_cycles = endtime - starttime;
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
    // if (stepM5) {
    //     digitalToggleFast(M5_stepPin);
    //     M5_pos += M5_direction;
    // }

    /// Always reset step triggers
    stepM1 = stepM2 = stepM3 = stepM4 = stepM5 = false;
}

FASTRUN void SetDirectionsAndLimits()
{
    if (M1_direction == 1)
    {
        digitalWriteFast(M1_dirPin, LOW); // Motor reverse mount
        if (Limit_Y1_end)
            stepM1 = false;
    }
    if (M1_direction == -1)
    {
        digitalWriteFast(M1_dirPin, HIGH); // Motor reverse mount
        if (Limit_Y1_start)
            stepM1 = false;
    }

    if (M2_direction == 1)
    {
        digitalWriteFast(M2_dirPin, HIGH);
        if (Limit_Y2_end)
            stepM2 = false;
    }
    if (M2_direction == -1)
    {
        digitalWriteFast(M2_dirPin, LOW);
        if (Limit_Y2_start)
            stepM2 = false;
    }

    if (M3_direction == 1)
    {
        digitalWriteFast(M3_dirPin, HIGH);
        if (Limit_X_end)
            stepM3 = false;
    }
    if (M3_direction == -1)
    {
        digitalWriteFast(M3_dirPin, LOW);
        if (Limit_X_start)
            stepM3 = false;
    }

    if (M4_direction == 1)
    {
        digitalWriteFast(M4_dirPin, LOW); // Motor reverse mount
        if (Limit_Z_end)
            stepM4 = false;
    }
    if (M4_direction == -1)
    {
        digitalWriteFast(M4_dirPin, HIGH); // Motor reverse mount
        if (Limit_Z_start)
            stepM4 = false;
    }

    // digitalWriteFast(M5_dirPin, M5_direction);
}

FASTRUN void CalculateHomeSteps()
{
    if (!isZero)
    {
        if (Limit_Y1_start && Limit_Y2_start && Limit_X_start && Limit_Z_start)
        {
            isZero = true;
            M1_pos = 0;
            M2_pos = 0;
            M3_pos = 0;
            M4_pos = 0;
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
    }
    else
    {
        if (!isMax)
        {
            if (Limit_Y1_end && Limit_Y2_end && Limit_X_end)
            {
                Serial.print("M1 max:");
                Serial.println(M1_pos);
                Serial.print("M2 max:");
                Serial.println(M2_pos);
                Serial.print("M3 max:");
                Serial.println(M3_pos);
                isMax = true;
                homeSpeed = 40.0f;
            }
            else
            {
                M1_direction = 1;
                stepM1 = true;
                M2_direction = 1;
                stepM2 = true;
                M3_direction = 1;
                stepM3 = true;
            }
        }
        else
        {
            if (M1_pos == 13581 && M2_pos == 13581 && M3_pos == 13581 && M4_pos == 13581)
            {
                M1_pos = 0;
                M2_pos = 0;
                M3_pos = 0;
                M4_pos = 0;
                isHome = true;
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
        }
    }
}

FASTRUN void MapHeight()
{
    /// Update current positions from motors
    posY = M1_pos;
    posX = M3_pos;
    posZ = M4_pos;

    switch (mappingState)
    {
    /// ================================================
    case (MappingState::MS_None):
    {
        HeightMapSetIndex = -1;

        /// Get index of the First HeightMap postition that is not set.
        for (int i = (HeightMapSize - 1); i >= 0; i--)
        {
            if (HeightMap[i] == INT64_MIN)
            {
                HeightMapSetIndex = i;
            }
        }

        if (HeightMapSetIndex == -1)
        {
            /// HeightMap is set completely.
            if (posZ != 0)
            {
                /// Move up first, then return here
                mappingState = MappingState::MS_Up;
            }
            else
            {
                /// Heightmap complete and safely moved up.
                mappingState = MappingState::MS_Done;
            }
        }
        else
        {
            /// HeightMapIndex needs mapping.
            if (posZ != 0)
            {
                /// Move up first, then return here.
                mappingState = MappingState::MS_Up;
            }
            else
            {
                /// Ready to move to the correct position

                moveEndX = (XMAX / 3) * (HeightMapSetIndex % 4);
                moveEndY = (YMAX / 5) * (HeightMapSetIndex / 4);

                if (posX == moveEndX && posY == moveEndY)
                {
                    /// Mapping position already reached, proceed to map height.
                    mappingState = MappingState::MS_Down;
                }
                else
                {
                    Serial.print("moving to:");
                    Serial.print(moveEndX);
                    Serial.print(" , ");
                    Serial.println(moveEndY);
                    mappingState = MappingState::MS_XYStart;
                }
            }
        }
        break;
    }
    /// ================================================
    case (MappingState::MS_Up):
    {
        StepMotors();
        /// Always reset step triggers and then start next iteration.
        stepM1 = stepM2 = stepM3 = stepM4 = stepM5 = false;

        /// Update current positions from motors
        posY = M1_pos;
        posX = M3_pos;
        posZ = M4_pos;

        if (posZ == 0)
        {
            /// Switch back to state: None
            mappingState = MappingState::MS_None;

            machineSpeed = normalSpeed;
        }
        else
        {
            if (posZ > 0)
                StepZ(-1);

            if (posZ < 0)
                StepZ(1);

            machineSpeed = normalSpeed;
        }

        SetDirectionsAndLimits();
        break;
    }

    /// ================================================
    case (MappingState::MS_XYStart):
    {
        /// start new movement
        moveDeltaX = abs(moveEndX - posX);
        moveDeltaY = -abs(moveEndY - posY);
        moveDirX = (posX < moveEndX ? 1 : -1);
        moveDirY = (posY < moveEndY ? 1 : -1);
        moveError = moveDeltaX + moveDeltaY;
        if (moveDeltaX > -moveDeltaY)
        {
            moveSteps = (double)moveDeltaX * 0.5;
        }
        else
        {
            moveSteps = (double)moveDeltaY * -0.5;
        }
        moveStep = 0;
        mappingState = MappingState::MS_XYMove;
        break;
    }

    /// ================================================
    case (MappingState::MS_XYMove):
    {
        StepMotors();
        /// Always reset step triggers and then start next iteration.
        stepM1 = stepM2 = stepM3 = stepM4 = stepM5 = false;

        /// Update current positions from motors
        posY = M1_pos;
        posX = M3_pos;
        posZ = M4_pos;

        if (posX == moveEndX && posY == moveEndY)
        {
            mappingState = MappingState::MS_None;
            machineSpeed = normalSpeed;
        }
        else
        {
            if (posX != moveEndX && posY != moveEndY)
            {
                if (2 * moveError <= moveDeltaX)
                {
                    moveError += moveDeltaX;
                    StepY(moveDirY);
                }
                if (2 * moveError >= moveDeltaY)
                {
                    moveError += moveDeltaY;
                    StepX(moveDirX);
                }
            }
            else
            {
                // At least x or y has reached its final position, if anything
                // remains, it must be a straight line.
                if (posX != moveEndX)
                {
                    StepX(moveDirX);
                }

                if (posY != moveEndY)
                {
                    StepY(moveDirY);
                }
            }
            double speedRamp = min(-fabs(moveSteps - moveStep) + moveSteps, 12000.0) / 400.0;
            machineSpeed = max(7.0 + 30.0 - speedRamp, 7.0);
            // machineSpeed = normalSpeed;
            moveStep++;
        }
        SetDirectionsAndLimits();
        break;
    }
    /// ================================================
    case (MappingState::MS_Down):
    {
        /// Skip for now
        /// Set height in the heightmap.
        HeightMap[HeightMapSetIndex] = 0;

        Serial.print("Height set for inded:");
        Serial.println(HeightMapSetIndex);

        /// Check if we need to map another position in the map.
        /// Switch to state: None
        mappingState = MappingState::MS_None;
        break;
    }

    /// ================================================
    case (MappingState::MS_Done):
    {
        Serial.println("Done with heightmap!");
        requestedState = State::state_none;
        activeState = State::state_none;
        break;
    }
    }
}

FASTRUN void CalculateDrawSteps()
{
    posY = M1_pos;
    posX = M3_pos;
    if (!lineStarted)
    {
        // check if this is a connected line or do we
        // need to move to a new location ?
        drawIndex = iBuffer[iBufferReadIndex].index;
        moveEndX = iBuffer[iBufferReadIndex].startX;
        moveEndY = iBuffer[iBufferReadIndex].startY;

        if (posX != moveEndX && posY != moveEndY)
        {
            moveDeltaX = abs(moveEndX - posX);
            moveDeltaY = -abs(moveEndY - posY);
            moveDirX = (posX < moveEndX ? 1 : -1);
            moveDirY = (posY < moveEndY ? 1 : -1);
            moveError = moveDeltaX + moveDeltaY;
            if (moveDeltaX > -moveDeltaY)
            {
                moveSteps = (double)moveDeltaX * 0.5;
            }
            else
            {
                moveSteps = (double)moveDeltaY * -0.5;
            }
            drawFunction = 2;
            moving = true;
        }
        else
        {
            // already at start pos no need to move
            if (iBuffer[iBufferReadIndex].type == 1)
            {
                // Draw Straight Line
            }
            if (iBuffer[iBufferReadIndex].type == 2)
            {
                // Quadratic Bezier
                drawError = iBuffer[iBufferReadIndex].error;
                drawDeltaX = iBuffer[iBufferReadIndex].deltaX;
                drawDeltaY = iBuffer[iBufferReadIndex].deltaY;
            }
            drawSpeed = 15.0f;
            drawFunction = 3;
            moving = false;
        }
        moveStep = 0;
        lineStarted = true;
    }

    if (moving)
    {
        // move to new location
        double speedRamp = min(-fabs(moveSteps - moveStep) + moveSteps, 12000.0) / 400.0;
        drawSpeed = max(7.0 + 30.0 - speedRamp, 7.0);
        //  Move in a straight line
        if (posX != moveEndX && posY != moveEndY)
        {
            if (2 * moveError <= moveDeltaX)
            {
                moveError += moveDeltaX;
                // stepY??
                StepY(moveDirY);
            }
            if (2 * moveError >= moveDeltaY)
            {
                moveError += moveDeltaY;
                // stepX??
                StepX(moveDirX);
            }
        }
        else
        {
            // at least x or y has reached its final position, it anything remains, it must be a straight line
            if (posX != moveEndX)
            {
                StepX(moveDirX);
            }
            else
            {
                if (posY != moveEndY)
                {
                    StepY(moveDirY);
                }
                else
                {
                    if (posX == moveEndX && posY == moveEndY)
                    {
                        // we have arrived at the start of the line?;
                        if (iBuffer[iBufferReadIndex].type == 1)
                        {
                            // Straight Line
                        }
                        if (iBuffer[iBufferReadIndex].type == 2)
                        {
                            // Quadratic Bezier
                            drawError = iBuffer[iBufferReadIndex].error;
                            drawDeltaX = iBuffer[iBufferReadIndex].deltaX;
                            drawDeltaY = iBuffer[iBufferReadIndex].deltaY;
                        }
                        drawSpeed = 15.0f;
                        drawFunction = 3;
                        moving = false;

                        // do we pause here ?
                    }
                }
            }
        }
        moveStep++;
    }
    else
    {
        // draw
        if (iBuffer[iBufferReadIndex].type == 1)
        {
            CalculateStraightLine();
        }

        if (iBuffer[iBufferReadIndex].type == 2)
        {
            CalculateQuadBezier();
        }
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
        // at least x or y has reached its final position, it anything remains, it must be a straight line
        if (posX != iBuffer[iBufferReadIndex].endX)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
        }
        else
        {
            if (posY != iBuffer[iBufferReadIndex].endY)
            {
                StepY(iBuffer[iBufferReadIndex].dirY);
            }
        }
    }
}

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
            // posY += iBuffer[iBufferReadIndex].dirY;
            drawDeltaX -= iBuffer[iBufferReadIndex].deltaXY;
            drawDeltaY += iBuffer[iBufferReadIndex].deltaYY;
            drawError += drawDeltaY;
        }
    }
    else
    {
        // at least x or y has reached its final position, it anything remains, it must be a straight line
        if (posX != iBuffer[iBufferReadIndex].endX)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
        }
        else
        {
            if (posY != iBuffer[iBufferReadIndex].endY)
            {
                StepY(iBuffer[iBufferReadIndex].dirY);
            }
        }
    }
}

FASTRUN void DebounceSwitches()
{
    // We do a fast press and slow release debounce
    // press debounce is shifted in 8 StepLoops
    // release debounce is performed every 300000 cycles

    //======== Fast PRESS DEBOUNCE ==========//
    Limit_Y1_start_on_count += digitalReadFast(Limit_Y1_start_pin);
    Limit_Y1_end_on_count += digitalReadFast(Limit_Y1_end_pin);
    Limit_Y2_start_on_count += digitalReadFast(Limit_Y2_start_pin);
    Limit_Y2_end_on_count += digitalReadFast(Limit_Y2_end_pin);
    Limit_X_start_on_count += digitalReadFast(Limit_X_start_pin);
    Limit_X_end_on_count += digitalReadFast(Limit_X_end_pin);
    Limit_Z_start_on_count += digitalReadFast(Limit_Z_start_pin);
    Limit_Z_end_on_count += digitalReadFast(Limit_Z_end_pin);
    panic_on_count += digitalReadFast(panic_pin);

    if (Limit_Y1_start_on_count > 7 && Limit_Y1_start == false)
    {
        Limit_Y1_start_off_count = __UINT8_MAX__;
        Limit_Y1_start = true;
    }

    if (Limit_Y1_end_on_count > 7 && Limit_Y1_end == false)
    {
        Limit_Y1_end_off_count = __UINT8_MAX__;
        Limit_Y1_end = true;
    }

    if (Limit_Y2_start_on_count > 7 && Limit_Y2_start == false)
    {
        Limit_Y2_start_off_count = __UINT8_MAX__;
        Limit_Y2_start = true;
    }

    if (Limit_Y2_end_on_count > 7 && Limit_Y2_end == false)
    {
        Limit_Y2_end_off_count = __UINT8_MAX__;
        Limit_Y2_end = true;
    }

    if (Limit_X_start_on_count > 7 && Limit_X_start == false)
    {
        Limit_X_start_off_count = __UINT8_MAX__;
        Limit_X_start = true;
    }

    if (Limit_X_end_on_count > 7 && Limit_X_end == false)
    {
        Limit_X_end_off_count = __UINT8_MAX__;
        Limit_X_end = true;
    }

    if (Limit_Z_start_on_count > 7 && Limit_Z_start == false)
    {
        Limit_Z_start_off_count = __UINT8_MAX__;
        Limit_Z_start = true;
    }

    if (Limit_Z_end_on_count > 7 && Limit_Z_end == false)
    {
        Limit_Z_end_off_count = __UINT8_MAX__;
        Limit_Z_end = true;
    }

    if (panic_on_count > 7 && panic_switch == false)
    {
        panic_off_count = __UINT8_MAX__;
        panic_switch = true;
        // PANIC Button! STOPPING ALL MOTORS Right now
        digitalWriteFast(M1_M2_M3_ennPin, 1);
        digitalWriteFast(M4_M5_enPin, 1);
    }

    //======== SLOW RELEASE DEBOUNCE ==========//
    if (debounceCounter > 300000)
    { // every 2 ms ?
        debounceCounter = 0;
        Limit_Y1_start_off_count <<= 1;
        Limit_Y1_start_off_count |= digitalReadFast(Limit_Y1_start_pin);
        Limit_Y1_end_off_count <<= 1;
        Limit_Y1_end_off_count |= digitalReadFast(Limit_Y1_end_pin);

        Limit_Y2_start_off_count <<= 1;
        Limit_Y2_start_off_count |= digitalReadFast(Limit_Y2_start_pin);
        Limit_Y2_end_off_count <<= 1;
        Limit_Y2_end_off_count |= digitalReadFast(Limit_Y2_end_pin);

        Limit_X_start_off_count <<= 1;
        Limit_X_start_off_count |= digitalReadFast(Limit_X_start_pin);
        Limit_X_end_off_count <<= 1;
        Limit_X_end_off_count |= digitalReadFast(Limit_X_end_pin);

        Limit_Z_start_off_count <<= 1;
        Limit_Z_start_off_count |= digitalReadFast(Limit_Z_start_pin);
        Limit_Z_end_off_count <<= 1;
        Limit_Z_end_off_count |= digitalReadFast(Limit_Z_end_pin);

        panic_off_count <<= 1;
        panic_off_count |= digitalReadFast(panic_pin);

        if (Limit_Y1_start_off_count == 0)
        {
            Limit_Y1_start = false;
            Limit_Y1_start_on_count = 0;
        }

        if (Limit_Y1_end_off_count == 0)
        {
            Limit_Y1_end = false;
            Limit_Y1_end_on_count = 0;
        }

        if (Limit_Y2_start_off_count == 0)
        {
            Limit_Y2_start = false;
            Limit_Y2_start_on_count = 0;
        }

        if (Limit_Y2_end_off_count == 0)
        {
            Limit_Y2_end = false;
            Limit_Y2_end_on_count = 0;
        }

        if (Limit_X_start_off_count == 0)
        {
            Limit_X_start = false;
            Limit_X_start_on_count = 0;
        }

        if (Limit_X_end_off_count == 0)
        {
            Limit_X_end = false;
            Limit_X_end_on_count = 0;
        }

        if (Limit_Z_start_off_count == 0)
        {
            Limit_Z_start = false;
            Limit_Z_start_on_count = 0;
        }

        if (Limit_Z_end_off_count == 0)
        {
            Limit_Z_end = false;
            Limit_Z_end_on_count = 0;
        }

        if (panic_off_count == 0)
        {
            panic_switch = false;
            panic_on_count = 0;
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

//========                                             ==========//
//======== Configuration for Switches en Motor Drivers ==========//
//========                                             ==========//

void configureSwitches()
{
    pinMode(Limit_Y1_start_pin, INPUT_PULLUP);
    pinMode(Limit_Y1_end_pin, INPUT_PULLUP);
    pinMode(Limit_Y2_start_pin, INPUT_PULLUP);
    pinMode(Limit_Y2_end_pin, INPUT_PULLUP);
    pinMode(Limit_X_start_pin, INPUT_PULLUP);
    pinMode(Limit_X_end_pin, INPUT_PULLUP);
    pinMode(Limit_Z_start_pin, INPUT_PULLUP);
    pinMode(Limit_Z_end_pin, INPUT_PULLUP);

    pinMode(panic_pin, INPUT_PULLUP);
}

TMC262::STATUS setTMC262Register(uint8_t bytes[3], int CSPIN)
{
    TMC262::STATUS st = {0};
    SPI1.beginTransaction(tmc262_spi_config);
    digitalWriteFast(CSPIN, 0);
    delayNanoseconds(spi_cs_delay);
    st.bytes[2] = SPI1.transfer(bytes[2]);
    st.bytes[1] = SPI1.transfer(bytes[1]);
    st.bytes[0] = SPI1.transfer(bytes[0]);
    digitalWriteFast(CSPIN, 1);
    SPI1.endTransaction();
    return st;
}

TMC2130::SPI_STATUS setTMC2130Register(uint8_t address, uint32_t data, int CSPIN)
{
    TMC2130::SPI_STATUS st = {0};
    SPI1.beginTransaction(tmc2130_spi_config);
    digitalWriteFast(CSPIN, 0);
    delayNanoseconds(spi_cs_delay);
    st.value = SPI1.transfer(address | 0x80);
    SPI1.transfer32(data);
    digitalWriteFast(CSPIN, 1);
    SPI1.endTransaction();
    return st;
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
    CurrentControl.IHOLD = 0;
    CurrentControl.IRUN = 16;
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
