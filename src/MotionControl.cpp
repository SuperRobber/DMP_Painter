#include "MotionControl.h"
#include <SPI.h>

bool panic = false;

//======== Limit Switch Pins ==========//
const int Limit_X1_start_pin = 36;
const int Limit_X1_end_pin = 35;
const int Limit_X2_start_pin = 38;
const int Limit_X2_end_pin = 37;
const int Limit_Y_start_pin = 40;
const int Limit_Y_end_pin = 39;
// const int Limit_Z_start_pin = 14;
// const int Limit_Z_end_pin = 41;

const int Limit_Z_start_pin = 41;
const int Limit_Z_end_pin = 14;

//======== Limit Switch Debounce (ON) count ==========//
uint8_t Limit_X1_start_on_count = 0;
uint8_t Limit_X1_end_on_count = 0;
uint8_t Limit_X2_start_on_count = 0;
uint8_t Limit_X2_end_on_count = 0;
uint8_t Limit_Y_start_on_count = 0;
uint8_t Limit_Y_end_on_count = 0;
uint8_t Limit_Z_start_on_count = 0;
uint8_t Limit_Z_end_on_count = 0;

//======== Limit Switch Debounce (OFF) count ==========//
uint8_t Limit_X1_start_off_count = __UINT8_MAX__;
uint8_t Limit_X1_end_off_count = __UINT8_MAX__;
uint8_t Limit_X2_start_off_count = __UINT8_MAX__;
uint8_t Limit_X2_end_off_count = __UINT8_MAX__;
uint8_t Limit_Y_start_off_count = __UINT8_MAX__;
uint8_t Limit_Y_end_off_count = __UINT8_MAX__;
uint8_t Limit_Z_start_off_count = __UINT8_MAX__;
uint8_t Limit_Z_end_off_count = __UINT8_MAX__;

//======== Limit Switch values ==========//
volatile bool Limit_X1_start = false;
volatile bool Limit_X1_end = false;
volatile bool Limit_X2_start = false;
volatile bool Limit_X2_end = false;
volatile bool Limit_Y_start = false;
volatile bool Limit_Y_end = false;
volatile bool Limit_Z_start = false;
volatile bool Limit_Z_end = false;

volatile bool isHome = false;
volatile bool isZero = false;

bool stepM1 = false;
bool stepM2 = false;
bool stepM3 = false;
bool stepM4 = false;
bool stepM5 = false;

//======== Stepper Motors ==========//
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

//======== Motion Control ==========//
RoboTimer StepLoopTimer; // PIR TIMER triggering StepLoop Interrupt
volatile float StepLoop_speed = 20.0f;
volatile float StepHomeLoop_speed = 50.0f;

volatile uint32_t plotter_pos_x = 0;
volatile uint32_t plotter_pos_y = 0;

volatile uint32_t M1_pos = 0;
volatile uint32_t M2_pos = 0;
volatile uint32_t M3_pos = 0;
volatile uint32_t M4_pos = 0;
volatile uint32_t M5_pos = 0;

volatile uint8_t M1_direction = 0;
volatile uint8_t M2_direction = 0;
volatile uint8_t M3_direction = 1;
volatile uint8_t M4_direction = 1;
volatile uint8_t M5_direction = 1;

uint32_t debounceCounter = 0;

volatile uint32_t max_step_cycles = 0; // keep track of highest StepperLoop time

TMC262::STATUS status_M1 = {0};
TMC262::STATUS status_M2 = {0};
TMC262::STATUS status_M3 = {0};

TMC262::DRVCONF driverConfig = {0};
TMC262::CHOPCONF ChopperConfig = {0};
TMC262::SGCSCONF StallGuardConfig = {0};
TMC262::SMARTEN CoolStepConfig = {0};
TMC262::DRVCTRL DriverControl = {0};

void StartEngines() {
    //======== ENABLE DRIVERS ==========//
    digitalWriteFast(M1_M2_M3_ennPin, 0);
    // digitalWriteFast(M4_M5_enPin,0);

    //======== Configure Interrupt Timer ===//
    CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL; // 150Mhz PIT clock
    StepLoopTimer.priority(16);

    //======== PERFORM HOMING ==========//
    Serial.println("Going Home");
    isHome = false;
    isZero = false;
    // Set all directions to go to 0

    M1_direction = 0;
    M2_direction = 0;
    M3_direction = 0;
    stepM1 = false;
    stepM2 = false;
    stepM3 = false;
    stepM4 = false;
    stepM5 = false;

    StepLoopTimer.begin(StepHomeLoop, StepHomeLoop_speed);

    while (!isHome) {
        // Serial.println("homing");
        delay(500);
    }
    StepLoopTimer.end();
    Serial.println("Arrived Home!");

    delay(1000);
    digitalWriteFast(M1_M2_M3_ennPin, 1);

    //======== START STEPLOOP ==========//

    Serial.println("Starting Loop");
    StepLoop_speed = 20.0f;
    StepLoopTimer.begin(StepLoop, StepLoop_speed); // blinkLED to run every 10 us
}

FASTRUN void StepHomeLoop() {
    // wait for proper debounce init time before starting motors!
    // so we do not get little accumulatings moves when we home
    // multiple times

    if (!isHome) {
        // PHASE 1 : Perform steps calculated previous Loop
        if (stepM1)
            digitalToggleFast(M1_stepPin);
        if (stepM2)
            digitalToggleFast(M2_stepPin);
        if (stepM3)
            digitalToggleFast(M3_stepPin);
        if (stepM4)
            digitalToggleFast(M4_stepPin);
        if (stepM5)
            digitalToggleFast(M5_stepPin);

        if (stepM1) {
            if (M1_direction) {
                M1_pos++;
            } else {
                M1_pos--;
            }
        }

        if (stepM2) {
            if (M2_direction) {
                M2_pos++;
            } else {
                M2_pos--;
            }
        }

        if (stepM3) {
            if (M3_direction) {
                M3_pos++;
            } else {
                M3_pos--;
            }
        }

        // PHASE 2 : Check and Debounce Limit Switches
        LimitSwitchesBounce();

        if (!isZero) {
            if (Limit_X1_start && Limit_X2_start && Limit_Y_start) {
                isZero = true;

                M1_pos = 0;
                M2_pos = 0;
                M3_pos = 0;
                M1_direction = 1;
                M2_direction = 1;
                M3_direction = 1;
            }

            // PHASE 3 : Calculate future steps for next round

            stepM1 = true;
            stepM2 = true;
            stepM3 = true;

        } else {
            stepM1 = false;
            stepM2 = false;
            stepM3 = false;
            if (M1_pos == 13581 && M2_pos == 13581 && M3_pos == 13581) {
                isHome = true;
            } else {
                if (M1_pos < 13581) {
                    stepM1 = true;
                }
                if (M2_pos < 13581) {
                    stepM2 = true;
                }
                if (M3_pos < 13581) {
                    stepM3 = true;
                }
            }
        }

        //======== PRE-STEP SET Direction ==========//
        digitalWriteFast(M1_dirPin, !M1_direction);
        digitalWriteFast(M2_dirPin, M2_direction); // reverse mount
        digitalWriteFast(M3_dirPin, M3_direction);
        digitalWriteFast(M4_dirPin, M4_direction);
        digitalWriteFast(M5_dirPin, M5_direction);

        // prevent motors from running past limit switches
        if (M1_direction) {
            if (Limit_X1_end)
                stepM1 = false;
        } else {
            if (Limit_X1_start)
                stepM1 = false;
        }

        if (M2_direction) {
            if (Limit_X2_end)
                stepM2 = false;
        } else {
            if (Limit_X2_start)
                stepM2 = false;
        }

        if (M3_direction) {
            if (Limit_Y_end)
                stepM3 = false;
        } else {
            if (Limit_Y_start)
                stepM3 = false;
        }

        //======== STEP SPEED ==========//
        // calculate the amount of time the future step will take
        uint32_t cycles = (float)150 * StepHomeLoop_speed - 0.5f;
        // StepLoopTimer.unsafe_update(cycles);
        debounceCounter += cycles;
    }
}

FASTRUN void StepLoop() {
    // to make steps as continuous as possiple we perform stepping at beginning of this Loop
    // and after take calculate what to do next round (can take variable amount of time)
    // PHASE 1 : Perform steps calculated previous Loop
    // PHASE 2 : Check and Debounce Limit Switches
    // PHASE 3 : Calculate future steps for next round

    uint32_t starttime = ARM_DWT_CYCCNT;

    // PHASE 1 : Perform steps calculated previous Loop
    if (stepM1)
        digitalToggleFast(M1_stepPin);
    if (stepM2)
        digitalToggleFast(M2_stepPin);
    if (stepM3)
        digitalToggleFast(M3_stepPin);
    if (stepM4)
        digitalToggleFast(M4_stepPin);
    if (stepM5)
        digitalToggleFast(M5_stepPin);

    if (stepM1) {
        if (M1_direction) {
            M1_pos++;
        } else {
            M1_pos--;
        }
    }

    if (stepM2) {
        if (M2_direction) {
            M2_pos++;
        } else {
            M2_pos--;
        }
    }

    if (stepM3) {
        if (M3_direction) {
            M3_pos++;
        } else {
            M3_pos--;
        }
    }

    if (stepM4) {
        if (M4_direction) {
            M4_pos++;
        } else {
            M4_pos--;
        }
    }

    if (stepM5) {
        if (M5_direction) {
            M5_pos++;
        } else {
            M5_pos--;
        }
    }
    // PHASE 2 : Check and Debounce Limit Switches

    LimitSwitchesBounce();

    // PHASE 3 : Calculate future steps for next round
    //======== STEP CALCULATION ==========//

    stepM1 = false;
    stepM2 = false;
    stepM3 = false;
    stepM4 = false;
    stepM5 = false;

    // we have calculated the correct step and direction

    // stepM1 = true;
    // stepM2 = true;
    // stepM3 = true;

    M1_direction = 1;
    M2_direction = 1;
    M3_direction = 1;

    //======== STEP CALCULATION ==========//

    // prevent motors from running past limit switches
    if (M1_direction) {
        if (Limit_X1_end)
            stepM1 = false;
    } else {
        if (Limit_X1_start)
            stepM1 = false;
    }

    if (M2_direction) {
        if (Limit_X2_end)
            stepM2 = false;
    } else {
        if (Limit_X2_start)
            stepM2 = false;
    }

    // if (M3_direction) {
    //     if (Limit_Y_end)
    //         stepM3 = false;
    // } else {
    //     if (Limit_Y_start)
    //         stepM3 = false;
    // }

    bool diagonalstep = false;

    //======== PRE-STEP SET Direction ==========//

    // CHECK motor Mouting, if DIRECTION value == 1 motor must move forward
    //  we can already set the direction for the upcoming step
    digitalWriteFast(M1_dirPin, !M1_direction);
    digitalWriteFast(M2_dirPin, M2_direction); // reverse mount
    digitalWriteFast(M3_dirPin, M3_direction);
    digitalWriteFast(M4_dirPin, M4_direction);
    digitalWriteFast(M5_dirPin, M5_direction);

    //======== STEP SPEED ==========//
    // for diagonal steps the loop should take SQRT(2) times longer to
    // maintain constant speed

    // calculate the amount of time the future step will take
    uint32_t cycles;
    if (diagonalstep) {
        cycles = (float)150 * StepLoop_speed * M_SQRT2 - 0.5f;
    } else {
        cycles = (float)150 * StepLoop_speed - 0.5f;
    }
    // new timer values are loaded after the upcoming trigger.
    StepLoopTimer.unsafe_update(cycles);
    debounceCounter += cycles;

    // END keep track of time for this IRQ
    uint32_t endtime = ARM_DWT_CYCCNT;
    if (endtime - starttime > max_step_cycles && endtime > starttime) {
        max_step_cycles = endtime - starttime;
    }
}

FASTRUN void LimitSwitchesBounce() {
    // We do a fast press and slow release debounce
    // press debounce is shifted in 8 StepLoops
    // release debounce is performed every 300000 cycles

    //======== Fast PRESS DEBOUNCE ==========//

    Limit_X1_start_on_count += digitalReadFast(Limit_X1_start_pin);
    Limit_X1_end_on_count += digitalReadFast(Limit_X1_end_pin);
    Limit_X2_start_on_count += digitalReadFast(Limit_X2_start_pin);
    Limit_X2_end_on_count += digitalReadFast(Limit_X2_end_pin);
    Limit_Y_start_on_count += digitalReadFast(Limit_Y_start_pin);
    Limit_Y_end_on_count += digitalReadFast(Limit_Y_end_pin);
    Limit_Z_start_on_count += digitalReadFast(Limit_Z_start_pin);
    Limit_Z_end_on_count += digitalReadFast(Limit_Z_end_pin);

    if (Limit_X1_start_on_count > 7 && Limit_X1_start == false) {
        Limit_X1_start_off_count = __UINT8_MAX__;
        Limit_X1_start = true;
    }

    if (Limit_X1_end_on_count > 7 && Limit_X1_end == false) {
        Limit_X1_end_off_count = __UINT8_MAX__;
        Limit_X1_end = true;
    }

    if (Limit_X2_start_on_count > 7 && Limit_X2_start == false) {
        Limit_X2_start_off_count = __UINT8_MAX__;
        Limit_X2_start = true;
    }

    if (Limit_X2_end_on_count > 7 && Limit_X2_end == false) {
        Limit_X2_end_off_count = __UINT8_MAX__;
        Limit_X2_end = true;
    }

    if (Limit_Y_start_on_count > 7 && Limit_Y_start == false) {
        Limit_Y_start_off_count = __UINT8_MAX__;
        Limit_Y_start = true;
    }

    if (Limit_Y_end_on_count > 7 && Limit_Y_end == false) {
        Limit_Y_end_off_count = __UINT8_MAX__;
        Limit_Y_end = true;
    }

    if (Limit_Z_start_on_count > 7 && Limit_Z_start == false) {
        Limit_Z_start_off_count = __UINT8_MAX__;
        Limit_Z_start = true;
    }

    if (Limit_Z_end_on_count > 7 && Limit_Z_end == false) {
        Limit_Z_end_off_count = __UINT8_MAX__;
        Limit_Z_end = true;

        // PANIC Button! STOPPING ALL MOTORS
        digitalWriteFast(M1_M2_M3_ennPin, 1);
        digitalWriteFast(M4_M5_enPin, 1);
    }

    //======== SLOW RELEASE DEBOUNCE ==========//
    if (debounceCounter > 300000) { // every 2 ms ?
        Limit_X1_start_off_count <<= 1;
        Limit_X1_start_off_count |= digitalReadFast(Limit_X1_start_pin);
        Limit_X1_end_off_count <<= 1;
        Limit_X1_end_off_count |= digitalReadFast(Limit_X1_end_pin);

        Limit_X2_start_off_count <<= 1;
        Limit_X2_start_off_count |= digitalReadFast(Limit_X2_start_pin);
        Limit_X2_end_off_count <<= 1;
        Limit_X2_end_off_count |= digitalReadFast(Limit_X2_end_pin);

        Limit_Y_start_off_count <<= 1;
        Limit_Y_start_off_count |= digitalReadFast(Limit_Y_start_pin);
        Limit_Y_end_off_count <<= 1;
        Limit_Y_end_off_count |= digitalReadFast(Limit_Y_end_pin);

        Limit_Z_start_off_count <<= 1;
        Limit_Z_start_off_count |= digitalReadFast(Limit_Z_start_pin);
        Limit_Z_end_off_count <<= 1;
        Limit_Z_end_off_count |= digitalReadFast(Limit_Z_end_pin);

        if (Limit_X1_start_off_count == 0) {
            Limit_X1_start = false;
            Limit_X1_start_on_count = 0;
        }

        if (Limit_X1_end_off_count == 0) {
            Limit_X1_end = false;
            Limit_X1_end_on_count = 0;
        }

        if (Limit_X2_start_off_count == 0) {
            Limit_X2_start = false;
            Limit_X2_start_on_count = 0;
        }

        if (Limit_X2_end_off_count == 0) {
            Limit_X2_end = false;
            Limit_X2_end_on_count = 0;
        }

        if (Limit_Y_start_off_count == 0) {
            Limit_Y_start = false;
            Limit_Y_start_on_count = 0;
        }

        if (Limit_Y_end_off_count == 0) {
            Limit_Y_end = false;
            Limit_Y_end_on_count = 0;
        }

        if (Limit_Z_start_off_count == 0) {
            Limit_Z_start = false;
            Limit_Z_start_on_count = 0;
        }

        if (Limit_Z_end_off_count == 0) {
            Limit_Z_end = false;
            Limit_Z_end_on_count = 0;
        }

        debounceCounter = 0;
    }
}

void updateStepperStatus() {
    status_M1 = setTMC262Register(DriverControl.bytes, M1_csPin);
    status_M2 = setTMC262Register(DriverControl.bytes, M2_csPin);
    status_M3 = setTMC262Register(DriverControl.bytes, M3_csPin);

    if ((bool)status_M1.Stalled)
        Serial.println("M1 Stallguard status: Stalled");
    if ((bool)status_M2.Stalled)
        Serial.println("M2 Stallguard status: Stalled");
    if ((bool)status_M3.Stalled)
        Serial.println("M3 Stallguard status: Stalled");

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

    if ((bool)status_M1.StandStill)
        Serial.println("M1 StandStill detected.");
    if ((bool)status_M2.StandStill)
        Serial.println("M2 StandStill detected.");
    if ((bool)status_M3.StandStill)
        Serial.println("M3 StandStill detected.");

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

void configureSwitches() {
    pinMode(Limit_X1_start_pin, INPUT_PULLUP);
    pinMode(Limit_X1_end_pin, INPUT_PULLUP);
    pinMode(Limit_X2_start_pin, INPUT_PULLUP);
    pinMode(Limit_X2_end_pin, INPUT_PULLUP);
    pinMode(Limit_Y_start_pin, INPUT_PULLUP);
    pinMode(Limit_Y_end_pin, INPUT_PULLUP);
    pinMode(Limit_Z_start_pin, INPUT_PULLUP);
    pinMode(Limit_Z_end_pin, INPUT_PULLUP);
}

TMC262::STATUS setTMC262Register(uint8_t bytes[3], int CSPIN) {
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

TMC2130::SPI_STATUS setTMC2130Register(uint8_t adress, uint32_t data, int CSPIN) {
    TMC2130::SPI_STATUS st = {0};
    SPI1.beginTransaction(tmc2130_spi_config);
    digitalWriteFast(CSPIN, 0);
    delayNanoseconds(spi_cs_delay);
    st.value = SPI1.transfer(adress | 0x80);
    SPI1.transfer32(data);
    digitalWriteFast(CSPIN, 1);
    SPI1.endTransaction();
    return st;
}

void configureStepperDrivers() {
    Serial.println("Configuring drivers..");
    // disable drives
    pinMode(M1_M2_M3_ennPin, OUTPUT);
    pinMode(M4_M5_enPin, OUTPUT);

    digitalWriteFast(M1_M2_M3_ennPin, 1); // Inverted input: LOW means enable
    digitalWriteFast(M4_M5_enPin, 1);     // Regular input:  HIGH means enable

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
    driverConfig.Vsense = 1;
    driverConfig.readInfo = 1;

    setTMC262Register(driverConfig.bytes, M1_csPin);
    setTMC262Register(driverConfig.bytes, M2_csPin);
    setTMC262Register(driverConfig.bytes, M3_csPin);

    ChopperConfig.address = 4;
    ChopperConfig.blanking_time = 2;
    ChopperConfig.chopper_mode = 0;
    ChopperConfig.random_t_off = 0;
    ChopperConfig.hysteresis_decrement_interval = 0;
    ChopperConfig.hysteresis_end_value = 3;
    ChopperConfig.hysteresis_start_value = 3;
    ChopperConfig.Toff = 4;

    setTMC262Register(ChopperConfig.bytes, M1_csPin);
    setTMC262Register(ChopperConfig.bytes, M2_csPin);
    setTMC262Register(ChopperConfig.bytes, M3_csPin);

    StallGuardConfig.address = 6;
    StallGuardConfig.filter = 0;
    StallGuardConfig.stall_threshold = 4; // 2s complement 0..63 = 0..63 / 64..127 = -63..-0
    StallGuardConfig.current_scale = 20;

    setTMC262Register(StallGuardConfig.bytes, M1_csPin);
    setTMC262Register(StallGuardConfig.bytes, M2_csPin);
    setTMC262Register(StallGuardConfig.bytes, M3_csPin);

    CoolStepConfig.address = 5;
    CoolStepConfig.min_current = 0;
    CoolStepConfig.current_decrement_speed = 0;
    CoolStepConfig.upper_coolstep_treshold = 0;
    CoolStepConfig.current_increment_size = 0;
    CoolStepConfig.lower_coolstep_treshold = 0; // disable coolstep;

    setTMC262Register(CoolStepConfig.bytes, M1_csPin);
    setTMC262Register(CoolStepConfig.bytes, M2_csPin);
    setTMC262Register(CoolStepConfig.bytes, M3_csPin);

    DriverControl.address = 0;
    DriverControl.interpolation = 0;
    DriverControl.double_edge_step = 1;
    DriverControl.microstep_resolition = 0;

    setTMC262Register(DriverControl.bytes, M1_csPin);
    setTMC262Register(DriverControl.bytes, M2_csPin);
    setTMC262Register(DriverControl.bytes, M3_csPin);

    // M4,M5 TMC2130 Fystec
    //  uint8_t d1;
    //  uint32_t d2;

    // Write GCONF
    TMC2130::GCONF globalConfig = {0};
    globalConfig.diag1_stall = 1;
    globalConfig.StealthChop = 0;
    setTMC2130Register(TMC2130::registers::reg_GCONF, globalConfig.data, M4_csPin);
    setTMC2130Register(TMC2130::registers::reg_GCONF, globalConfig.data, M5_csPin);

    // Write CHOPCONF
    TMC2130::CHOPCONF ChopperConfig = {0};
    ChopperConfig.mode = 0;
    ChopperConfig.mres = 1;   // 256 microsteps
    ChopperConfig.dedge = 1;  // double edge step
    ChopperConfig.vsense = 1; // fullpower
    ChopperConfig.blanktime = 2;
    ChopperConfig.hend = 1;
    ChopperConfig.hstart = 4;
    ChopperConfig.toff = 3;
    setTMC2130Register(TMC2130::registers::reg_CHOPCONF, ChopperConfig.data, M4_csPin);
    setTMC2130Register(TMC2130::registers::reg_CHOPCONF, ChopperConfig.data, M5_csPin);

    // Write IHOLD_IRUN
    TMC2130::IHOLD_IRUN CurrentControl = {0};
    CurrentControl.IHOLD = 2;
    CurrentControl.IRUN = 4;
    CurrentControl.IHOLDDELAY = 6;
    setTMC2130Register(TMC2130::registers::reg_IHOLD_IRUN, CurrentControl.data, M4_csPin);
    delayMicroseconds(1);
    setTMC2130Register(TMC2130::registers::reg_IHOLD_IRUN, CurrentControl.data, M5_csPin);

    // Write TPOWERDOWN
    TMC2130::TPOWERDOWN PowerdownDelay = {0};
    PowerdownDelay.TPOWERDOWN = 10;
    setTMC2130Register(TMC2130::registers::reg_TPOWERDOWN, PowerdownDelay.data, M4_csPin);
    setTMC2130Register(TMC2130::registers::reg_TPOWERDOWN, PowerdownDelay.data, M5_csPin);

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
