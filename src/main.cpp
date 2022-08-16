// #include "Display6963.h"
#include <Arduino.h>
#include "MotionControl.h"
#include "PWMSounds.h"
// #include "font.h"
#include <SPI.h>

/////////////////////////////

elapsedMicros LCDRefreshTimer;
elapsedMicros LOADRefreshTimer;
elapsedMicros POWERRefreshTimer;
elapsedMillis disconnectTimer;
elapsedMillis statusTimer;

const int AudioPin = 33;
// const int lcd_cmdPin = 23;
// const int lcd_writePin = 22;
const int powersense_cs_Pin = 34;

SPISettings spi_powersense_config(16000000, MSBFIRST, SPI_MODE0);
int32_t powerData = 0;

void setup() {
    Serial.begin(115200);

    pinMode(AudioPin, OUTPUT);
    pinMode(powersense_cs_Pin, OUTPUT);
    digitalWriteFast(powersense_cs_Pin, HIGH);

    // LCD and PowerSense are on SPI0
    SPI.begin();

    // lcd_init(lcd_writePin, lcd_cmdPin);

    // drawText(0, 0, "Startup: ZERO Position", msx_8x8_font, false);
    // drawScreen();

    // playVictorySound(AudioPin);
    playBeep(AudioPin);

    configureSwitches();
    configureStepperDrivers();

    StartEngines();

    playBeep(AudioPin);
    // while (!Serial) {
    // }
}

void loop() {
    if (Serial.dtr()) {
        // getInstructions();

        if (statusTimer > 50) {

            SPI.beginTransaction(spi_powersense_config);
            digitalWriteFast(powersense_cs_Pin, LOW);
            powerData = SPI.transfer16(0);
            digitalWriteFast(powersense_cs_Pin, HIGH);
            SPI.endTransaction();

            updateStepperStatus();

            statusTimer = 0;
            String status = "";
            status += "$status$";
            status += String(M1_pos) ;
            status +="$";
            status += String(M2_pos) ;
            status +="$";
            status += String(M3_pos) ;
            status +="$";
            status += String(M4_pos) ;
            status +="$";
            status += String(M5_pos) ;
            status +="$";
            status += String(status_M1.Stallguard_Load) ;
            status +="$";
            status += String(status_M2.Stallguard_Load) ;
            status +="$";
            status += String(status_M3.Stallguard_Load) ;
            status +="$";
            status += String(0) ;
            status +="$";
            status += String(0) ;
            status +="$";
            status += String((powerData - 2047) * 12) ;
            status +="$";
            status += String(Limit_X1_start) ;
            status +="$";
            status += String(Limit_X1_end) ;
            status +="$";
            status += String(Limit_X2_start) ;
            status +="$";
            status += String(Limit_X2_end) ;
            status +="$";
            status += String(Limit_Y_start) ;
            status +="$";
            status += String(Limit_Y_end) ;
            status +="$";
            status += String(Limit_Z_start) ;
            status +="$";
            status += String(Limit_Z_end) ;
            status +="$";
            Serial.println(status);
        }

        disconnectTimer = 0;
    } else {
        if (disconnectTimer > 60000) {
            // disconnect for 1 minute
            // finish list of instructions
            // go home.
        }
    }

    // refresh display
    // partially refresh screen about 3us/step
    //< 8ms for a total screen redraw in 32x64 = 2048+12 steps


    /*
    drawScreenStep();

    if (lcd_drawstep == 0) {
        // copy from interrupt
        uint32_t m1p = M1_pos;
        uint32_t m2p = M2_pos;
        uint32_t m3p = M3_pos;
        uint32_t m4p = M4_pos;
        uint32_t m5p = M5_pos;
        // powerData=4000;
        char m1_mm[10];
        char m2_mm[10];
        char m3_mm[10];
        char m4_mm[10];
        char m5_mm[10];

        char power[10];

        dtostrf(((float)(m1p * 12) * PI) / (200.0f * 256.0f), 9, 2, m1_mm);
        dtostrf(((float)(m2p * 12) * PI) / (200.0f * 256.0f), 9, 2, m2_mm);
        dtostrf(((float)(m3p * 12) * PI) / (200.0f * 256.0f), 9, 2, m3_mm);
        dtostrf(((float)(m4p * 12) * PI) / (200.0f * 256.0f), 9, 2, m4_mm);
        dtostrf(((float)(m5p * 12) * PI) / (200.0f * 256.0f), 9, 2, m5_mm);

        dtostrf((powerData - 2047) * 12, 5, 0, power);

        // drawText(0,  0, "POSITION", msx_8x8_font,false);
        drawText(0, 0, "M1", msx_8x8_font, false);
        drawText(24, 0, m1_mm, msx_8x8_font, false);
        drawFrame(94, 0, 200, 6);
        drawSquare(96, 2, 96 + (1024 - status_M1.Stallguard_Load) / 10, 4);
        drawFrame(212, 0, 218, 6);
        drawFrame(220, 0, 226, 6);
        // drawFrame(233,0,239,6);
        if (Limit_X1_start)
            drawSquare(214, 2, 216, 4);
        if (Limit_X1_end)
            drawSquare(222, 2, 224, 4);

        drawText(0, 9, "M2", msx_8x8_font, false);
        drawText(24, 9, m2_mm, msx_8x8_font, false);
        drawFrame(94, 9, 200, 15);
        drawSquare(96, 11, 96 + (1024 - status_M2.Stallguard_Load) / 10, 13);
        drawFrame(212, 9, 218, 15);
        drawFrame(220, 9, 226, 15);
        // drawFrame(233,9,239,15);
        if (Limit_X2_start)
            drawSquare(214, 11, 216, 13);
        if (Limit_X2_end)
            drawSquare(222, 11, 224, 13);

        drawText(0, 18, "M3", msx_8x8_font, false);
        drawText(24, 18, m3_mm, msx_8x8_font, false);
        drawFrame(94, 18, 200, 24);
        drawSquare(96, 20, 96 + (1024 - status_M3.Stallguard_Load) / 10, 22);
        drawFrame(212, 18, 218, 24);
        drawFrame(220, 18, 226, 24);
        // drawFrame(233,18,239,24);
        if (Limit_Y_start)
            drawSquare(214, 20, 216, 22);
        if (Limit_Y_end)
            drawSquare(222, 20, 224, 22);

        drawText(0, 27, "M4", msx_8x8_font, false);
        drawText(24, 27, m4_mm, msx_8x8_font, false);
        drawFrame(94, 27, 200, 33);
        drawSquare(96, 29, 96 + (1024 - 1024) / 10, 31);
        drawFrame(212, 27, 218, 33);
        drawFrame(220, 27, 226, 33);
        // drawFrame(233,27,239,33);
        if (Limit_Z_start)
            drawSquare(214, 29, 216, 31);
        if (Limit_Z_end)
            drawSquare(222, 29, 224, 31);

        drawText(0, 36, "M4", msx_8x8_font, false);
        drawText(24, 36, m5_mm, msx_8x8_font, false);
        drawFrame(94, 36, 200, 42);
        drawSquare(96, 38, 96 + (1024 - 1024) / 10, 40);
        // drawFrame(212,36,218,42);
        // drawFrame(233,36,239,42);

        drawText(190, 57, power, msx_8x8_font, false);
        drawText(226, 57, "mA", msx_8x8_font, false);
        drawText(164, 57, "24v:", msx_8x8_font, false);

        // drawText(0, 57, Limit_X1_start_press_count, msx_8x8_font, false);
        // drawText(20, 57, Limit_X1_end_press_count, msx_8x8_font, false);
        // drawText(40, 57, Limit_X2_start_press_count, msx_8x8_font, false);
        // drawText(60, 57, Limit_X2_end_press_count, msx_8x8_font, false);
    }

    if (POWERRefreshTimer > 50000) {
        POWERRefreshTimer = 0;
        SPI.beginTransaction(spi_powersense_config);
        digitalWriteFast(powersense_cs_Pin, LOW);
        powerData = SPI.transfer16(0);
        digitalWriteFast(powersense_cs_Pin, HIGH);
        SPI.endTransaction();
        // Serial.print("Power Usage @ 24v: ");
        // Serial.print((powerData-2047) * 12);
        // Serial.println(" mA");

        updateStepperStatus();
        // Serial.print("M1 Load: ");
        // Serial.println(1024-status_M1.Stallguard_Load);
        // Serial.print("M2 Load: ");
        // Serial.println(1024-status_M2.Stallguard_Load);
        // Serial.print("M3 Load: ");
        // Serial.println(1024-status_M3.Stallguard_Load);
    }

    */
}