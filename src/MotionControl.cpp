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

// ===================== Home algorithm. =====================

volatile HomeState homeState = HomeState::None;

// ===================== Zero algorithm. =====================

volatile ZeroState zeroState = ZeroState::None;

// ===================== Height Mapping algorithm. =====================

#define HeightMapWidth 25
#define HeightMapLength 37
#define HeightMapSize 925

volatile MapHeightState mapHeightState = MapHeightState::None;

int32_t HeightMap[HeightMapSize];
int heightMapIndex = 0;
int HeightMapOffset = 0;
int HeightMapXTileSize = 0;
int HeightMapYTileSize = 0;
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

/// ===================== OD-Mini Height Sensor =====================

int heightMeasurementTime = 0;
volatile int32_t heightMeasurement;

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

volatile MoveAction move;
volatile DrawAction draw;

volatile int32_t posXMotor = 0;
volatile int64_t posXDraw = 0;
volatile int32_t posXStart = 0;
const int32_t posXSensorOffset = 83200; // 2cm paper offset + 4.5cm offset from laser to pen tip
volatile XState xState = XState::None;

volatile int32_t posYMotor = 0;
volatile int64_t posYDraw = 0;
volatile int32_t posYStart = 0;
volatile YState yState = YState::None;

volatile int32_t posZMotor = 0;
volatile int64_t posZDraw = 0;

volatile ZState zState = ZState::None;
volatile int32_t posZUp = 0;      // position for pen up
volatile int32_t posZDrawMin = 0; // position for pen minumum (touching)
volatile int32_t posZDrawMax = 0; // position for pen maximum (full pressure)

/// Hardware interrupt (PIR) TIMER triggering MachineLoop Interrupt.
RoboTimer IRQTimer;

/// Interrupt iteration times (speeds).
const float moveSpeed = 7.0f;
const float drawSpeed = 25.0f;
const float homeSpeed = 100.0f;
const float normalSpeed = 100.0f;

// acceleration
const double accelerationFactor = 300.0;
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

// volatile int32_t ZHeight = 0;

volatile int8_t M1_direction = 0;
volatile int8_t M2_direction = 0;
volatile int8_t M3_direction = 0;
volatile int8_t M4_direction = 0;
volatile int8_t M5_direction = 0;

/// Track performance / measure interrupt-loop time.
volatile uint32_t max_step_cycles = 0;

/// ==========================================
/// Weight function for bilinear interpolation

// float Weight(float a, float b)
// {
//     return a * b;
// }

/// Weight function for optimized (constrained) bicubic

float Weight(float a, float b)
{
    return a * a * b * b * (9.0f - 6.0f * a - 6.0f * b + 4.0f * a * b);
}

int32_t Interpolate(int32_t Px0y0, int32_t Px1y0, int32_t Px0y1, int32_t Px1y1, float tX, float tY)
{
    return Weight(1 - tX, 1 - tY) * Px0y0 + Weight(tX, 1 - tY) * Px1y0 + Weight(1 - tX, tY) * Px0y1 + Weight(tX, tY) * Px1y1;
}

void HaltMotors()
{
    digitalWriteFast(M1_M2_M3_ennPin, 1);
    digitalWriteFast(M4_M5_enPin, 1);
}

int32_t getHeight(int32_t x, int32_t y)
{
    int32_t iX = max(0, min(x / HeightMapXTileSize, HeightMapWidth - 1));
    int32_t iY = max(0, min(y / HeightMapYTileSize, HeightMapLength - 1));

    float tX = max(0.0f, min(1.0f, (float)(x - iX * HeightMapXTileSize) / (float)HeightMapXTileSize));
    float tY = max(0.0f, min(1.0f, (float)(y - iY * HeightMapYTileSize) / (float)HeightMapYTileSize));

    int32_t A = HeightMap[(iY)*HeightMapWidth + (iX)];
    int32_t B = HeightMap[(iY)*HeightMapWidth + (iX + 1)];
    int32_t C = HeightMap[(iY + 1) * HeightMapWidth + (iX)];
    int32_t D = HeightMap[(iY + 1) * HeightMapWidth + (iX + 1)];
    if (A == INT32_MIN)
        A = 0; // heightmap is not set;
    if (B == INT32_MIN)
        B = 0; // heightmap is not set;
    if (C == INT32_MIN)
        C = 0; // heightmap is not set;
    if (D == INT32_MIN)
        D = 0; // heightmap is not set;

    return Interpolate(A, B, C, D, tX, tY);
}
/// ==========================================

void StoreInEEPROM()
{
    byte32 b32 = {};

    b32.value = posXStart;
    for (int e = 0; e < 4; e++)
    {
        EEPROM.write(e + 12, b32.bytes[e]);
    }

    b32.value = posYStart;
    for (int e = 0; e < 4; e++)
    {
        EEPROM.write(e + 16, b32.bytes[e]);
    }

    b32.value = posZUp;
    for (int e = 0; e < 4; e++)
    {
        EEPROM.write(e + 0, b32.bytes[e]);
    }

    b32.value = posZDrawMin;
    for (int e = 0; e < 4; e++)
    {
        EEPROM.write(e + 4, b32.bytes[e]);
    }

    b32.value = posZDrawMax;
    for (int e = 0; e < 4; e++)
    {
        EEPROM.write(e + 8, b32.bytes[e]);
    }

    Serial.println("Offset positions saved to EEPROM.");

}

void RecallFromEEPROM()
{

    byte32 b32 = {};

    /// Read PenPositions from EEPROM
    for (int e = 0; e < 4; e++)
    {
        b32.bytes[e] = EEPROM.read(e + 0);
    }
    posZUp = b32.value;

    for (int e = 0; e < 4; e++)
    {
        b32.bytes[e] = EEPROM.read(e + 4);
    }
    posZDrawMin = b32.value;

    for (int e = 0; e < 4; e++)
    {
        b32.bytes[e] = EEPROM.read(e + 8);
    }
    posZDrawMax = b32.value;

    for (int e = 0; e < 4; e++)
    {
        b32.bytes[e] = EEPROM.read(e + 12);
    }
    posXStart = b32.value;

    for (int e = 0; e < 4; e++)
    {
        b32.bytes[e] = EEPROM.read(e + 16);
    }
    posYStart = b32.value;

    Serial.println("Offset positions loaded from EEPROM.");
}

void StartUp()
{
    /// Read the HeightMap from EEPROM
    byte32 b32 = {};
    for (unsigned int i = 0; i < HeightMapSize; i++)
    {
        for (int e = 0; e < 4; e++)
        {
            b32.bytes[e] = EEPROM.read(i * 4 + e + 64);
        }
        HeightMap[i] = b32.value;
    }

    // Read the HeightMap offset from EEPROM
    for (int e = 0; e < 4; e++)
    {
        b32.bytes[e] = EEPROM.read(e + 60);
    }
    HeightMapOffset = b32.value;

    RecallFromEEPROM();

    HeightMapXTileSize = XMAX / (HeightMapWidth - 1);
    HeightMapYTileSize = YMAX / (HeightMapLength - 1);

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
    case Mode::Zero:
    {
        Zero();
        break;
    }
    case Mode::ClearHeight:
    {
        /// clear HeightMap and EEPROM
        for (int i = 0; i < HeightMapSize; i++)
        {
            byte32 b32 = {};
            b32.value = INT32_MIN;
            HeightMap[i] = INT32_MIN;
            for (int e = 0; e < 4; e++)
            {
                EEPROM.write(i * 4 + e + 64, b32.bytes[e]);
            }
        }
        // heightMeasurementTime = 0;
        Serial.println("HeightMap cleared.");
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::ZUp:
    {
        ZUp();
        break;
    }

    case Mode::ZDown:
    {
        ZDown();
        break;
    }

    case Mode::XUp:
    {
        XUp();
        break;
    }

    case Mode::XDown:
    {
        XDown();
        break;
    }

    case Mode::YUp:
    {
        YUp();
        break;
    }

    case Mode::YDown:
    {
        YDown();
        break;
    }

    case Mode::SetXStart:
    {
        posXStart = posXMotor;
        Serial.print("Set X Start position to: ");
        Serial.println(posXStart);
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::SetYStart:
    {
        posYStart = posYMotor;
        Serial.print("Set Y Start position to: ");
        Serial.println(posYStart);
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::SetPenUp:
    {
        posZUp = posZMotor;
        Serial.print("Set Pen Up position to: ");
        Serial.println(posZUp);
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::PenUpPlus:
    {
        posZUp = posZUp + 20;
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::PenUpMinus:
    {
        posZUp = posZUp - 20;
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::SetPenMin:
    {
        posZDrawMin = posZMotor;
        Serial.print("Set Pen Min position to: ");
        Serial.println(posZDrawMin);
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::PenMinPlus:
    {
        posZDrawMin = posZDrawMin + 20;
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::PenMinMinus:
    {
        posZDrawMin = posZDrawMin - 20;
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::SetPenMax:
    {
        posZDrawMax = posZMotor;
        Serial.print("Set Pen Max position to: ");
        Serial.println(posZDrawMax);
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::PenMaxPlus:
    {
        posZDrawMax = posZDrawMax + 20;
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::PenMaxMinus:
    {
        posZDrawMax = posZDrawMax - 20;
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::Store:
    {
        StoreInEEPROM();
        requestedMode = Mode::None;
        activeMode = Mode::None;
        break;
    }

    case Mode::Recall:
    {
        RecallFromEEPROM();
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

        if (requestedMode == Mode::Zero)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;

            zeroState = ZeroState::Choose;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::Zero;
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

        if (requestedMode == Mode::ClearHeight)
        {
            activeMode = Mode::ClearHeight;
        }

        if (requestedMode == Mode::XUp)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
            xState = XState::Choose;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::XUp;
        }

        if (requestedMode == Mode::XDown)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
            xState = XState::Choose;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::XDown;
        }

        if (requestedMode == Mode::YUp)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
            yState = YState::Choose;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::YUp;
        }

        if (requestedMode == Mode::YDown)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
            yState = YState::Choose;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::YDown;
        }

        if (requestedMode == Mode::ZUp)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
            zState = ZState::Choose;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::ZUp;
        }

        if (requestedMode == Mode::ZDown)
        {
            if (sleeping) /// Wake up
            {
                setCurrent(workCurrent);
                sleeping = false;
            }
            sleepTimer = 0;
            zState = ZState::Choose;
            statusFunction = StatusFunction::Moving;
            activeMode = Mode::ZDown;
        }

        if (requestedMode == Mode::SetXStart)
            activeMode = Mode::SetXStart;

        if (requestedMode == Mode::SetYStart)
            activeMode = Mode::SetYStart;

        if (requestedMode == Mode::SetPenUp)
            activeMode = Mode::SetPenUp;

        if (requestedMode == Mode::PenUpPlus)
            activeMode = Mode::PenUpPlus;

        if (requestedMode == Mode::PenUpMinus)
            activeMode = Mode::PenUpMinus;

        if (requestedMode == Mode::SetPenMin)
            activeMode = Mode::SetPenMin;

        if (requestedMode == Mode::PenMinPlus)
            activeMode = Mode::PenMinPlus;

        if (requestedMode == Mode::PenMinMinus)
            activeMode = Mode::PenMinMinus;

        if (requestedMode == Mode::SetPenMax)
            activeMode = Mode::SetPenMax;

        if (requestedMode == Mode::PenMaxPlus)
            activeMode = Mode::PenMaxPlus;

        if (requestedMode == Mode::PenMaxMinus)
            activeMode = Mode::PenMaxMinus;

        if (requestedMode == Mode::Store)
            activeMode = Mode::Store;

        if (requestedMode == Mode::Recall)
            activeMode = Mode::Recall;

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

FASTRUN void XUp()
{
    switch (xState)
    {
    case (XState::None):
    {
        break;
    }
    case (XState::Choose):
    {
        move.endX = M3_pos - 20;
        requestedMode = Mode::None;
        xState = XState::Move;
        break;
    }
    case (XState::Move):
    {
        StepMotors();

        /// Is there a request to Stop moving?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            xState = XState::None;
            // Do not execute the rest of this case.
            break;
        }

        if (M3_pos == move.endX)
        {
            activeMode = Mode::None;
            xState = XState::None;
            statusFunction = StatusFunction::Waiting;
        }
        else
        {
            if (M3_pos > move.endX)
            {
                M3_direction = -1;
                stepM3 = true;
            }

            if (M3_pos < move.endX)
            {
                M3_direction = 1;
                stepM3 = true;
            }
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed * 4;
        break;
    }
    }
}

FASTRUN void XDown()
{
    switch (xState)
    {
    case (XState::None):
    {
        break;
    }
    case (XState::Choose):
    {
        move.endX = M3_pos + 20;
        requestedMode = Mode::None;
        xState = XState::Move;
        break;
    }
    case (XState::Move):
    {
        StepMotors();

        /// Is there a request to Stop moving?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            xState = XState::None;
            // Do not execute the rest of this case.
            break;
        }

        if (M3_pos == move.endX)
        {
            activeMode = Mode::None;
            xState = XState::None;
            statusFunction = StatusFunction::Waiting;
        }
        else
        {
            if (M3_pos > move.endX)
            {
                M3_direction = -1;
                stepM3 = true;
            }

            if (M3_pos < move.endX)
            {
                M3_direction = 1;
                stepM3 = true;
            }
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed * 4;
        break;
    }
    }
}

FASTRUN void YUp()
{
    switch (yState)
    {
    case (YState::None):
    {
        break;
    }
    case (YState::Choose):
    {
        move.endY = M1_pos - 20;
        requestedMode = Mode::None;
        yState = YState::Move;
        break;
    }
    case (YState::Move):
    {
        StepMotors();

        /// Is there a request to Stop moving?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            yState = YState::None;
            // Do not execute the rest of this case.
            break;
        }

        if (M1_pos == move.endY && M2_pos == move.endY)
        {
            activeMode = Mode::None;
            yState = YState::None;
            statusFunction = StatusFunction::Waiting;
        }
        else
        {
            if (M1_pos > move.endY)
            {
                M1_direction = -1;
                stepM1 = true;
            }

            if (M1_pos < move.endY)
            {
                M1_direction = 1;
                stepM1 = true;
            }

            if (M2_pos > move.endY)
            {
                M2_direction = -1;
                stepM2 = true;
            }
            if (M2_pos < move.endY)
            {
                M2_direction = 1;
                stepM2 = true;
            }
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed * 4;
        break;
    }
    }
}

FASTRUN void YDown()
{
    switch (yState)
    {
    case (YState::None):
    {
        break;
    }
    case (YState::Choose):
    {
        move.endY = M1_pos + 20;
        requestedMode = Mode::None;
        yState = YState::Move;
        break;
    }
    case (YState::Move):
    {
        StepMotors();

        /// Is there a request to Stop moving?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            yState = YState::None;
            // Do not execute the rest of this case.
            break;
        }

        if (M1_pos == move.endY && M2_pos == move.endY)
        {
            activeMode = Mode::None;
            yState = YState::None;
            statusFunction = StatusFunction::Waiting;
        }
        else
        {
            if (M1_pos > move.endY)
            {
                M1_direction = -1;
                stepM1 = true;
            }

            if (M1_pos < move.endY)
            {
                M1_direction = 1;
                stepM1 = true;
            }

            if (M2_pos > move.endY)
            {
                M2_direction = -1;
                stepM2 = true;
            }
            if (M2_pos < move.endY)
            {
                M2_direction = 1;
                stepM2 = true;
            }
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed * 4;
        break;
    }
    }
}

FASTRUN void ZUp()
{
    switch (zState)
    {
    case (ZState::None):
    {
        break;
    }
    case (ZState::Choose):
    {
        move.endZ = M4_pos - 20;
        requestedMode = Mode::None;
        zState = ZState::Move;
        break;
    }
    case (ZState::Move):
    {
        StepMotors();

        /// Is there a request to Stop moving?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            zState = ZState::None;
            // Do not execute the rest of this case.
            break;
        }

        if (M4_pos == move.endZ)
        {
            activeMode = Mode::None;
            zState = ZState::None;
            statusFunction = StatusFunction::Waiting;
        }
        else
        {
            if (M4_pos > move.endZ)
            {
                M4_direction = -1;
                stepM4 = true;
            }

            if (M4_pos < move.endZ)
            {
                M4_direction = 1;
                stepM4 = true;
            }
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed * 4;
        break;
    }
    }
}

FASTRUN void ZDown()
{
    switch (zState)
    {
    case (ZState::None):
    {
        break;
    }
    case (ZState::Choose):
    {
        move.endZ = M4_pos + 20;
        requestedMode = Mode::None;
        zState = ZState::Move;
        break;
    }
    case (ZState::Move):
    {
        StepMotors();

        /// Is there a request to Stop moving?
        if (requestedMode == Mode::Stop)
        {
            activeMode = Mode::Stop;
            zState = ZState::None;
            // Do not execute the rest of this case.
            break;
        }

        if (M4_pos == move.endZ)
        {
            activeMode = Mode::None;
            zState = ZState::None;
            statusFunction = StatusFunction::Waiting;
        }
        else
        {
            if (M4_pos > move.endZ)
            {
                M4_direction = -1;
                stepM4 = true;
            }

            if (M4_pos < move.endZ)
            {
                M4_direction = 1;
                stepM4 = true;
            }
        }

        SetDirectionsAndLimits();
        machineSpeed = homeSpeed * 4;
        break;
    }
    }
}

FASTRUN void Draw()
{
    // Z calculations in Draw Algorithm are based on floating Zdraw.
    // where Zdraw = 0 equals posZDrawMin + posZerr(for current posX,posY)
    // all calculation should here should be relative to Zdraw
    switch (drawState)
    {
    case (DrawState::None):
    {
        break;
    }
    /// ================================================
    case (DrawState::Choose):
    {
        // Set draw positions from current motor position and heightError.

        posXDraw = posXMotor - posXStart;
        posYDraw = posYMotor - posYStart;        
        posZDraw = posZMotor - posZDrawMin - getHeight((posXDraw)+posXSensorOffset, posYDraw);

        machineSpeed = normalSpeed;
        if (requestedMode == Mode::Stop)
        {

            // Check / LIFT PEN totally up
            if (posZDraw != -posZDrawMin) // (eq posZMotor==0);
            {
                // Prepare to lift pen.
                move.endZ = -posZDrawMin;
                move.dirZ = (posZDraw < move.endZ ? 1 : -1);
                move.steps = (double)abs(move.endZ - posZDraw);
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
            if (posZDraw != -posZDrawMin + posZUp)
            {
                // Prepare to lift pen.
                move.endZ = -posZDrawMin + posZUp;
                move.dirZ = (posZDraw < move.endZ ? 1 : -1);
                move.steps = (double)abs(move.endZ - posZDraw);
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
            if (posXDraw == iBuffer[iBufferReadIndex].startX && posYDraw == iBuffer[iBufferReadIndex].startY && posZDraw == iBuffer[iBufferReadIndex].startZ)
            {
                // Already at start position, continue drawing.

                // copy values that will be changine during draw
                draw.deltaX = iBuffer[iBufferReadIndex].deltaX;
                draw.deltaY = iBuffer[iBufferReadIndex].deltaY;
                draw.deltaZ = iBuffer[iBufferReadIndex].deltaZ;
                draw.error = iBuffer[iBufferReadIndex].error;
                draw.errorX = iBuffer[iBufferReadIndex].errorX;
                draw.errorY = iBuffer[iBufferReadIndex].errorY;
                draw.errorZ = iBuffer[iBufferReadIndex].errorZ;
                draw.deltaMax = iBuffer[iBufferReadIndex].deltaMax;
                draw.endStage = false;

                // Set step to 0 only on a new group, not for segment
                if (iBuffer[iBufferReadIndex].groupIndex == 0)
                    draw.step = 0.0;

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
                if (posXDraw == iBuffer[iBufferReadIndex].startX && posYDraw == iBuffer[iBufferReadIndex].startY)
                {
                    // Arrived at correct XY location, but posZ is not at startPosition.
                    // Prepare to move pen to Z start Position.
                    move.endZ = iBuffer[iBufferReadIndex].startZ;
                    move.dirZ = (posZDraw < move.endZ ? 1 : -1);
                    move.steps = (double)abs(move.endZ - posZDraw);
                    move.step = 0;
                    drawState = DrawState::MoveZ;
                    statusFunction = StatusFunction::Moving;
                    break;
                }
                else
                {
                    // Not at correct location yet. Is pen up ?
                    if (posZDraw == -posZDrawMin + posZUp)
                    {
                        // Pen is up.
                        // Set up a new movement to go to XY Location
                        move.endX = iBuffer[iBufferReadIndex].startX;
                        move.endY = iBuffer[iBufferReadIndex].startY;
                        move.deltaX = abs(move.endX - posXDraw);
                        move.deltaY = -abs(move.endY - posYDraw);
                        move.dirX = (posXDraw < move.endX ? 1 : -1);
                        move.dirY = (posYDraw < move.endY ? 1 : -1);
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
                        move.endZ = -posZDrawMin + posZUp;
                        move.dirZ = (posZDraw < move.endZ ? 1 : -1);
                        move.steps = (double)abs(move.endZ - posZDraw);
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

        if (posZDraw == move.endZ)
        {
            // Arrived at Z destination

            drawState = DrawState::Choose;
            statusFunction = StatusFunction::Waiting;

            /// No urgency to continue straight on need to move // lift pen etc
            break;
        }

        if (posZDraw != move.endZ)
        {
            /// Change Zdraw and perform a possible height error adjustment.
            /// Adjust Motor bases on required ZDraw and Zerr
            StepZDraw(move.dirZ);
        }

        double t = -fabs(move.steps * 0.5 - move.step) + move.steps * 0.5;
        machineSpeed = max(moveSpeed * 2 + accelerationRange - (t / (t + accelerationFactor)) * accelerationRange, moveSpeed * 2);
        move.step++;

        SetDirectionsAndLimits();
        break;
    }

    /// ===============================================
    case (DrawState::MoveXY):
    {
        StepMotors();

        if (posXDraw == move.endX && posYDraw == move.endY)
        {
            // Arrived at XY destination

            drawState = DrawState::Choose;
            statusFunction = StatusFunction::Waiting;

            /// No urgency to continue straight on need to move // lift pen etc
            break;
        }

        if (posXDraw != move.endX && posYDraw != move.endY)
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

            if (posXDraw != move.endX)
            {
                StepX(move.dirX);
            }

            if (posYDraw != move.endY)
            {
                StepY(move.dirY);
            }
        }

        /// Perform a possible height error adjustment. Does not change posZDraw, but can step motor to compensate height.
        StepZDraw(0);

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

        if (posXDraw == iBuffer[iBufferReadIndex].endX && posYDraw == iBuffer[iBufferReadIndex].endY && posZDraw == iBuffer[iBufferReadIndex].endZ)
        {
            /// Done drawing current line!

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

        double t = 1;

        if (iBuffer[iBufferReadIndex].acceleration == 0) // single
        {
            t = -fabs(iBuffer[iBufferReadIndex].steps * 0.5 - draw.step) + iBuffer[iBufferReadIndex].steps * 0.5;
        }
        if (iBuffer[iBufferReadIndex].acceleration == 1) // start
        {
            t = draw.step;
            draw.t = t;
        }
        if (iBuffer[iBufferReadIndex].acceleration == 2) // continue
        {
            t = draw.t;
        }
        if (iBuffer[iBufferReadIndex].acceleration == 3) // stop
        {
            t = fabs(iBuffer[iBufferReadIndex].steps - draw.step);
        }

        machineSpeed = max(drawSpeed + accelerationRange - (t / (t + accelerationFactor)) * accelerationRange, drawSpeed);
        if (draw.step < iBuffer[iBufferReadIndex].steps)
            draw.step++;

        SetDirectionsAndLimits();

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
        StepZDraw(iBuffer[iBufferReadIndex].dirZ);
    }
    else
    {
        /// Perform a possible height error adjustment. Does not change posZDraw, but can step motor to compensate height.
        StepZDraw(0);
    }
}

FASTRUN void CalculateQuadBezier3DXY()
{
    if (posXDraw != iBuffer[iBufferReadIndex].endX && posYDraw != iBuffer[iBufferReadIndex].endY)
    {
        bool do_step_x = 2 * draw.error - draw.deltaY > 0;
        bool do_step_y = 2 * draw.error - draw.deltaX < 0;

        if (do_step_x)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
            draw.deltaX -= iBuffer[iBufferReadIndex].deltaXY;
            draw.deltaY += iBuffer[iBufferReadIndex].deltaYY;
            draw.error += draw.deltaY;
            draw.errorZ -= iBuffer[iBufferReadIndex].deltaXZ;
        }

        if (do_step_y)
        {
            StepY(iBuffer[iBufferReadIndex].dirY);
            draw.deltaY -= iBuffer[iBufferReadIndex].deltaXY;
            draw.deltaX += iBuffer[iBufferReadIndex].deltaXX;
            draw.error += draw.deltaX;
            draw.errorZ -= iBuffer[iBufferReadIndex].deltaYZ;
        }

        if (draw.errorZ < 0)
        {
            StepZDraw(iBuffer[iBufferReadIndex].dirZ);
            draw.errorZ += draw.deltaZ;
        }
        else
        {
            /// Perform a possible height error adjustment. Does not change posZDraw, but can step motor to compensate height.
            StepZDraw(0);
        }
    }
    else
    {
        if (posXDraw != iBuffer[iBufferReadIndex].endX || posYDraw != iBuffer[iBufferReadIndex].endY || posZDraw != iBuffer[iBufferReadIndex].endZ)
        {
            if (!draw.endStage)
            {
                // Prepare remaining part of curve as a straight line
                draw.deltaX = abs(iBuffer[iBufferReadIndex].endX - posXDraw);
                draw.deltaY = abs(iBuffer[iBufferReadIndex].endY - posYDraw);
                draw.deltaZ = abs(iBuffer[iBufferReadIndex].endZ - posZDraw);
                draw.deltaMax = max(draw.deltaZ, max(draw.deltaX, draw.deltaY));
                draw.errorX = draw.deltaMax / 2;
                draw.errorY = draw.deltaMax / 2;
                draw.errorZ = draw.deltaMax / 2;
                draw.endStage = true;
            }
            CalculateStraightLine3D();
        }
    }
}

FASTRUN void CalculateQuadBezier3DXZ()
{
    if (posXDraw != iBuffer[iBufferReadIndex].endX && posZDraw != iBuffer[iBufferReadIndex].endZ)
    {
        bool do_step_x = 2 * draw.error - draw.deltaZ > 0;
        bool do_step_z = 2 * draw.error - draw.deltaX < 0;

        if (do_step_x)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
            draw.deltaX -= iBuffer[iBufferReadIndex].deltaXZ;
            draw.deltaZ += iBuffer[iBufferReadIndex].deltaZZ;
            draw.error += draw.deltaZ;
            draw.errorY -= iBuffer[iBufferReadIndex].deltaXY;
        }

        if (do_step_z)
        {
            StepZ(iBuffer[iBufferReadIndex].dirZ);
            draw.deltaZ -= iBuffer[iBufferReadIndex].deltaXZ;
            draw.deltaX += iBuffer[iBufferReadIndex].deltaXX;
            draw.error += draw.deltaX;
            draw.errorY -= iBuffer[iBufferReadIndex].deltaYZ;
        }
        else
        {
            /// Perform a possible height error adjustment. Does not change posZDraw, but can step motor to compensate height.
            StepZDraw(0);
        }

        if (draw.errorY < 0)
        {
            StepY(iBuffer[iBufferReadIndex].dirY);
            draw.errorY += draw.deltaY;
        }
    }
    else
    {
        if (posXDraw != iBuffer[iBufferReadIndex].endX || posYDraw != iBuffer[iBufferReadIndex].endY || posZDraw != iBuffer[iBufferReadIndex].endZ)
        {
            if (!draw.endStage)
            {
                // Prepare remaining part of curve as a straight line
                draw.deltaX = abs(iBuffer[iBufferReadIndex].endX - posXDraw);
                draw.deltaY = abs(iBuffer[iBufferReadIndex].endY - posYDraw);
                draw.deltaZ = abs(iBuffer[iBufferReadIndex].endZ - posZDraw);
                draw.deltaMax = max(draw.deltaZ, max(draw.deltaX, draw.deltaY));
                draw.errorX = draw.deltaMax / 2;
                draw.errorY = draw.deltaMax / 2;
                draw.errorZ = draw.deltaMax / 2;
                draw.endStage = true;
            }
            CalculateStraightLine3D();
        }
    }
}

FASTRUN void CalculateQuadBezier3DYZ()
{
    if (posYDraw != iBuffer[iBufferReadIndex].endY && posZDraw != iBuffer[iBufferReadIndex].endZ)
    {
        bool do_step_z = 2 * draw.error - draw.deltaY > 0;
        bool do_step_y = 2 * draw.error - draw.deltaZ < 0;

        if (do_step_z)
        {
            StepZDraw(iBuffer[iBufferReadIndex].dirZ);
            draw.deltaZ -= iBuffer[iBufferReadIndex].deltaYZ;
            draw.deltaY += iBuffer[iBufferReadIndex].deltaYY;
            draw.error += draw.deltaY;
            draw.errorX -= iBuffer[iBufferReadIndex].deltaXZ;
        }
        else
        {
            /// Perform a possible height error adjustment. Does not change posZDraw, but can step motor to compensate height.
            StepZDraw(0);
        }

        if (do_step_y)
        {
            StepY(iBuffer[iBufferReadIndex].dirY);
            draw.deltaY -= iBuffer[iBufferReadIndex].deltaYZ;
            draw.deltaZ += iBuffer[iBufferReadIndex].deltaZZ;
            draw.error += draw.deltaZ;
            draw.errorX -= iBuffer[iBufferReadIndex].deltaXY;
        }

        if (draw.errorX < 0)
        {
            StepX(iBuffer[iBufferReadIndex].dirX);
            draw.errorX += draw.deltaX;
        }
    }
    else
    {
        if (posXDraw != iBuffer[iBufferReadIndex].endX || posYDraw != iBuffer[iBufferReadIndex].endY || posZDraw != iBuffer[iBufferReadIndex].endZ)
        {
            if (!draw.endStage)
            {
                // Prepare remaining part of curve as a straight line
                draw.deltaX = abs(iBuffer[iBufferReadIndex].endX - posXDraw);
                draw.deltaY = abs(iBuffer[iBufferReadIndex].endY - posYDraw);
                draw.deltaZ = abs(iBuffer[iBufferReadIndex].endZ - posZDraw);
                draw.deltaMax = max(draw.deltaZ, max(draw.deltaX, draw.deltaY));
                draw.errorX = draw.deltaMax / 2;
                draw.errorY = draw.deltaMax / 2;
                draw.errorZ = draw.deltaMax / 2;
                draw.endStage = true;
            }
            CalculateStraightLine3D();
        }
    }
}


FASTRUN void Zero()
{
    switch (zeroState)
    {
    case (ZeroState::None):
    {
        break;
    }

    case (ZeroState::Choose):
    {
        if (posXMotor == 0 && posYMotor == 0 && posZMotor == 0)
        {
            // done
            requestedMode = Mode::None;
            activeMode = Mode::None;
            homeState = HomeState::None;
            break;
        }
        else
        {
            // Not at correct location yet. Is pen up ?
            if (posZMotor == 0)
            {
                // Pen is up.
                // Set up a new movement to go to XY Location
                move.endX = 0;
                move.endY = 0;
                move.deltaX = abs(move.endX - posXMotor);
                move.deltaY = -abs(move.endY - posYMotor);
                move.dirX = (posXMotor < move.endX ? 1 : -1);
                move.dirY = (posYMotor < move.endY ? 1 : -1);
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
                zeroState = ZeroState::MoveXY;
                statusFunction = StatusFunction::Moving;
                break;
            }
            else
            {
                // Prepare to lift pen.
                move.endZ = 0;
                move.dirZ = (posZMotor < move.endZ ? 1 : -1);
                move.steps = (double)abs(move.endZ - posZMotor);
                move.step = 0;
                zeroState = ZeroState::MoveZ;
                statusFunction = StatusFunction::Moving;
                break;
            }
        }
        break;
    }

    /// ===============================================
    case (ZeroState::MoveZ):
    {
        StepMotors();

        if (posZMotor == move.endZ)
        {
            // Arrived at Z destination

            zeroState = ZeroState::Choose;
            statusFunction = StatusFunction::Waiting;

            /// No urgency to continue straight on need to move // lift pen etc
            break;
        }

        if (posZMotor != move.endZ)
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
    case (ZeroState::MoveXY):
    {
        StepMotors();

        if (posXMotor == move.endX && posYMotor == move.endY)
        {
            // Arrived at XY destination

            zeroState = ZeroState::Choose;
            statusFunction = StatusFunction::Waiting;

            /// No urgency to continue straight on need to move // lift pen etc
            break;
        }

        if (posXMotor != move.endX && posYMotor != move.endY)
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

            if (posXMotor != move.endX)
            {
                StepX(move.dirX);
            }

            if (posYMotor != move.endY)
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
    }
}

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

        if ((switches[swY1Start].pressed && switches[swY2Start].pressed) && switches[swXStart].pressed && switches[swZStart].pressed)
        {
            /// All limit switches hit,
            /// proceed to margin position
            heightMeasurementTime = 0;

            M1_pos = 0;
            M2_pos = 0;
            M3_pos = 0;
            M4_pos = 0;
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

        SetHomeDirectionsAndLimits();
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

        if (M1_pos == 7000 && M2_pos == 3000 && M3_pos == 6400 && heightMeasurement == 1500)
        {
            heightMeasurementTime++;
            if (heightMeasurementTime > 1000)
            {
                /// Margin reached at 1 cm on axis,
                /// reset coordinates.
                M1_pos = 0;
                M2_pos = 0; // paper start at +2cm -> 25600
                M3_pos = 0; // paper start at +2cm -> 25600
                M4_pos = 0;

                posXMotor = 0;
                posYMotor = 0;
                posZMotor = 0;

                posXDraw = posXMotor - posXStart;
                posYDraw = posYMotor - posYStart;

                homeState = HomeState::Done;
                statusFunction = StatusFunction::Waiting;
            }
        }
        else
        {
            if (M1_pos > 7000)
            {
                M1_direction = -1;
                stepM1 = true;
            }
            if (M1_pos < 7000)
            {
                M1_direction = 1;
                stepM1 = true;
            }

            if (M2_pos > 3000)
            {
                M2_direction = -1;
                stepM2 = true;
            }
            if (M2_pos < 3000)
            {
                M2_direction = 1;
                stepM2 = true;
            }

            if (M3_pos > 6400)
            {
                M3_direction = -1;
                stepM3 = true;
            }
            if (M3_pos < 6400)
            {
                M3_direction = 1;
                stepM3 = true;
            }

            if (heightMeasurement < 1500)
            {
                M4_direction = -1;
                stepM4 = true;
            }

            if (heightMeasurement > 1500)
            {
                M4_direction = 1;
                stepM4 = true;
            }

            // if (M4_pos > 12800)
            // {
            //     M4_direction = -1;
            //     stepM4 = true;
            // }

            // if (M4_pos < 12800)
            // {
            //     M4_direction = 1;
            //     stepM4 = true;
            // }
        }

        SetHomeDirectionsAndLimits();
        machineSpeed = homeSpeed * 4;
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
            if (HeightMap[i] == INT32_MIN)
            {
                heightMapIndex = i;
            }
        }

        if (heightMapIndex == -1)
        {
            /// HeightMap is set completely.
            if (posZMotor != 0)
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
            if (posZMotor != 0 && heightMapIndex == 0)
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
                move.endY = (YMAX / (HeightMapLength - 1)) * (heightMapIndex / HeightMapWidth);

                if (posXMotor == move.endX && posYMotor == move.endY)
                {
                    heightMeasurementTime = 0;
                    /// Mapping position already reached, proceed to map height.
                    mapHeightState = MapHeightState::MoveZero;
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

        if (posZMotor == 0)
        {
            /// Switch back to state: Choose
            mapHeightState = MapHeightState::Choose;
        }
        else
        {
            if (posZMotor > 0)
                StepZ(-1);

            if (posZMotor < 0)
                StepZ(1);
        }
        machineSpeed = normalSpeed;

        SetDirectionsAndLimits();
        break;
    }

    /// ================================================
    case (MapHeightState::MoveZero):
    {
        StepMotors();

        if (heightMeasurement == 0)
        {
            heightMeasurementTime++;
            if (heightMeasurementTime > 200)
            {
                if (heightMapIndex == 0)
                {
                    HeightMap[heightMapIndex] = 0;
                    HeightMapOffset = posZMotor;

                    // store offset in EEPROM
                    byte32 b32 = {};
                    b32.value = HeightMapOffset;
                    for (int e = 0; e < 4; e++)
                    {
                        EEPROM.write(e + 60, b32.bytes[e]);
                    }
                }
                else
                {
                    HeightMap[heightMapIndex] = posZMotor - HeightMapOffset;
                }

                /// store Mapped height in EEPROM
                byte32 b32 = {};
                b32.value = HeightMap[heightMapIndex];
                for (int e = 0; e < 4; e++)
                {
                    EEPROM.write(heightMapIndex * 4 + e + 64, b32.bytes[e]);
                }

                // reset count for next index.
                Serial.print("Height set for index:");
                Serial.print(heightMapIndex);
                Serial.print(" to:");
                Serial.println(HeightMap[heightMapIndex]);

                /// Check if we need to map another position in the map.
                /// Switch to state: Choose
                mapHeightState = MapHeightState::Choose;
                machineSpeed = normalSpeed;
                break;
            }
        }
        else
        {
            if (heightMeasurement > 0)
                StepZ(1);

            if (heightMeasurement < 0)
                StepZ(-1);
        }
        machineSpeed = normalSpeed * 10;

        SetDirectionsAndLimits();
        break;
    }

    /// ================================================
    case (MapHeightState::StartMoveXY):
    {
        /// Set up a new movement
        move.deltaX = abs(move.endX - posXMotor);
        move.deltaY = -abs(move.endY - posYMotor);
        move.dirX = (posXMotor < move.endX ? 1 : -1);
        move.dirY = (posYMotor < move.endY ? 1 : -1);
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

        if (posXMotor == move.endX && posYMotor == move.endY)
        {
            mapHeightState = MapHeightState::Choose;
            machineSpeed = normalSpeed;
        }
        else
        {
            if (posXMotor != move.endX && posYMotor != move.endY)
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
                if (posXMotor != move.endX)
                {
                    StepX(move.dirX);
                }

                if (posYMotor != move.endY)
                {
                    StepY(move.dirY);
                }
            }
            // double speedRamp = min(-fabs(move.steps * 0.5 - move.step) + move.steps * 0.5, 12000.0) / 400.0;
            // machineSpeed = max(7.0 + 30.0 - speedRamp, 7.0);

            double t = -fabs(move.steps * 0.5 - move.step) + move.steps * 0.5;
            machineSpeed = max(moveSpeed + accelerationRange - (t / (t + accelerationFactor)) * accelerationRange, moveSpeed);
            move.step++;
        }
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
    posXMotor = M3_pos;
    posYMotor = M1_pos;
    
    posXDraw = posXMotor-posXStart;
    posYDraw = posYMotor-posYStart;

    posZMotor = M4_pos;
}


FASTRUN void SetHomeDirectionsAndLimits()
{
    /// Switches for Y1 and Y2 are linked.
    /// If a switch or switch wire fails,
    /// left and right motors will both moving in that direction.

    if (M1_direction == 1)
    {
        digitalWriteFast(M1_dirPin, LOW); // Motor reverse mount
        if (switches[swY1End].pressed)
            stepM1 = false;
    }
    if (M1_direction == -1)
    {
        digitalWriteFast(M1_dirPin, HIGH); // Motor reverse mount
        if (switches[swY1Start].pressed)
            stepM1 = false;
    }

    if (M2_direction == 1)
    {
        digitalWriteFast(M2_dirPin, HIGH);
        if (switches[swY2End].pressed)
            stepM2 = false;
    }
    if (M2_direction == -1)
    {
        digitalWriteFast(M2_dirPin, LOW);
        if (switches[swY2Start].pressed)
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
        digitalWriteFast(M4_dirPin, HIGH);
        if (switches[swZEnd].pressed)
            stepM4 = false;
    }
    if (M4_direction == -1)
    {
        digitalWriteFast(M4_dirPin, LOW);
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
        digitalWriteFast(M4_dirPin, HIGH);
        if (switches[swZEnd].pressed)
            stepM4 = false;
    }
    if (M4_direction == -1)
    {
        digitalWriteFast(M4_dirPin, LOW);
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

FASTRUN void StepZDraw(int8_t dir)
{
    // Calculate requested position,
    // If it changes from actual position take step

    posZDraw += dir;

    // Get Zerr for this position, TODO: cache latest getHeight
    int32_t requestedPosition = min(posZDrawMax, getHeight((posXDraw)+posXSensorOffset, posYDraw) + posZDrawMin + posZDraw);

    if (requestedPosition > posZMotor)
    {
        M4_direction = 1;
        stepM4 = true;
    }
    if (requestedPosition < posZMotor)
    {
        M4_direction = -1;
        stepM4 = true;
    }
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
        // digitalWriteFast(M1_M2_M3_ennPin, 1);
        // digitalWriteFast(M4_M5_enPin, 1);
        HaltMotors();
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

    // if ((bool) status_M1.OpenLoad_A) {
    //     HaltMotors();
    //     Serial.println("M1 OpenLoad detected: COIL A");
    // }
    // if ((bool) status_M2.OpenLoad_A) {
    //     HaltMotors();
    //     Serial.println("M2 OpenLoad detected: COIL A");
    // }
    // if ((bool) status_M3.OpenLoad_A) {
    //     HaltMotors();
    //     Serial.println("M3 OpenLoad detected: COIL A");
    // }
    // if ((bool) status_M1.OpenLoad_B) {
    //     HaltMotors();
    //     Serial.println("M1 OpenLoad detected: COIL B");
    // }
    // if ((bool) status_M2.OpenLoad_B) {
    //     HaltMotors();
    //     Serial.println("M2 OpenLoad detected: COIL B");
    // }
    // if ((bool) status_M3.OpenLoad_B) {
    //     HaltMotors();
    //     Serial.println("M3 OpenLoad detected: COIL B");
    // }

    if ((bool)status_M1.OverTemp_Warning)
        Serial.println("M1 Over Temperature Warning!");
    if ((bool)status_M2.OverTemp_Warning)
        Serial.println("M2 Over Temperature Warning!");
    if ((bool)status_M3.OverTemp_Warning)
        Serial.println("M3 Over Temperature Warning!");

    if ((bool)status_M1.OverTemp_Shutdown)
    {
        HaltMotors();
        Serial.println("M1 Over Temperature Shutdown!");
    }
    if ((bool)status_M2.OverTemp_Shutdown)
    {
        HaltMotors();
        Serial.println("M2 Over Temperature Shutdown!");
    }
    if ((bool)status_M3.OverTemp_Shutdown)
    {
        HaltMotors();
        Serial.println("M3 Over Temperature Shutdown!");
    }

    // if ((bool)status_M1.StandStill)
    //     Serial.println("M1 StandStill detected.");
    // if ((bool)status_M2.StandStill)
    //     Serial.println("M2 StandStill detected.");
    // if ((bool)status_M3.StandStill)
    //     Serial.println("M3 StandStill detected.");

    if ((bool)status_M1.Short_A)
    {
        HaltMotors();
        Serial.println("M1 Short on COIL A detected.");
    }
    if ((bool)status_M2.Short_A)
    {
        HaltMotors();
        Serial.println("M2 Short on COIL A detected.");
    }
    if ((bool)status_M3.Short_A)
    {
        HaltMotors();
        Serial.println("M3 Short on COIL A detected.");
    }
    if ((bool)status_M1.Short_B)
    {
        HaltMotors();
        Serial.println("M1 Short on COIL B detected.");
    }
    if ((bool)status_M2.Short_B)
    {
        HaltMotors();
        Serial.println("M2 Short on COIL B detected.");
    }
    if ((bool)status_M3.Short_B)
    {
        HaltMotors();
        Serial.println("M3 Short on COIL B detected.");
    }
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
    driverControl.microstep_resolution = 0;

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