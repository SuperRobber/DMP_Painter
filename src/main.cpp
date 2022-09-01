#include <Arduino.h>
#include <SPI.h>

#include "MotionControl.h"
#include "PWMSounds.h"

/////////////////////////////

// elapsedMicros POWERRefreshTimer;
elapsedMillis disconnectTimer;
elapsedMillis statusTimer;
elapsedMillis requestTimer;
elapsedMicros drawTimeout;

const int AudioPin = 33;
// const int lcd_cmdPin = 23;
// const int lcd_writePin = 22;
const int powersense_cs_Pin = 34;

SPISettings spi_powersense_config(16000000, MSBFIRST, SPI_MODE0);
int32_t powerData = 0;

// DATA Layout of one message should be:
//  - 10 x 0xFF byte header
//  - 1 byte containing the size of the data to be received (N)
//  - N number of values up to 8byte (int64) each, with a 0x00 byte in between
//  - a checksum calculated on the original data.

// union for converting int32 to bytearray
union byte32 {
    int32_t value;
    byte bytes[4];
};

// union for converting int64 to bytearray
union byte64 {
    int64_t value;
    byte bytes[8];
};

int64_t requestedInstruction = -1;
int64_t recievedInstruction = -1;
int requestCounter = 0;

//======== Serial Binary Protocol ==========//
int serialMessageSize = 0;
int serialInstructionHeaderCount = 0;
bool serialInstructionStarted = false;
int serialMessageByteCount = 0;
int serialMessageChecksumByteCount = 0;
int32_t serialMessageCalculatedChecksum = 0;
byte32 serialMessageReceivedChecksum = {0};
byte serialMessageData[250];  // reserve 250bytes for serialmessage data

int serialHomeHeaderCount = 0;
int serialPauseHeaderCount = 0;
int serialHeightMapHeaderCount = 0;
int serialResetHeaderCount = 0;
int serialDrawHeaderCount = 0;
// bool serialHomeCommand = false;

void getSerial(int bytesToRead);

void setup() {
    Serial.begin(115200);

    pinMode(AudioPin, OUTPUT);
    pinMode(powersense_cs_Pin, OUTPUT);
    digitalWriteFast(powersense_cs_Pin, HIGH);

    // PowerSense on SPI0
    SPI.begin();

    // playVictorySound(AudioPin);
    playBeep(AudioPin);

    configureSwitches();
    configureStepperDrivers();

    StartEngines();

    playBeep(AudioPin);
}

void loop() {
    if (Serial.dtr()) {
        int bytesToRead = Serial.available();
        if (bytesToRead > 0) {
            getSerial(bytesToRead);
        }

        // do we have room in the buffer ?
        if (((iBufferWriteIndex + 1) & 63) != iBufferReadIndex) {
            // have we received our latest request ?
            if (recievedInstruction == requestedInstruction) {
                // yes, lets request a new instruction to add into the buffer
                requestedInstruction++;
                Serial.print("@");
                Serial.print(requestedInstruction);
                Serial.println("@ requesting instruction ");
                requestTimer = 0;
            } else {
                // no
                if (requestTimer > 10000) {
                    // its been 10seconds, lets re-request
                    Serial.print("@");
                    Serial.print(requestedInstruction);
                    Serial.print("@ requesting instruction, ");
                    Serial.print("retry: ");
                    Serial.println(requestCounter);
                    if (requestCounter > 20) {
                        Serial.println("returning home.");
                    } else {
                        requestCounter++;
                    }
                    requestTimer = 0;
                }
            }
        } else {
            // buffer is full, wait for drawing to advance before requesting more
        }

        if (statusTimer > 50) {  // 20fps update

            SPI.beginTransaction(spi_powersense_config);
            digitalWriteFast(powersense_cs_Pin, LOW);
            powerData = SPI.transfer16(0);
            digitalWriteFast(powersense_cs_Pin, HIGH);
            SPI.endTransaction();

            updateStepperStatus();

            statusTimer = 0;
            String status = "";
            status += "$status$";
            status += String(M1_pos);
            status += "$";
            status += String(M2_pos);
            status += "$";
            status += String(M3_pos);
            status += "$";
            status += String(M4_pos);
            status += "$";
            status += String(M5_pos);
            status += "$";
            status += String(status_M1.Stallguard_Load);
            status += "$";
            status += String(status_M2.Stallguard_Load);
            status += "$";
            status += String(status_M3.Stallguard_Load);
            status += "$";
            status += String(0);
            status += "$";
            status += String(0);
            status += "$";
            status += String((powerData - 2047) * 12);
            status += "$";
            status += String(Limit_Y1_start);
            status += "$";
            status += String(Limit_Y1_end);
            status += "$";
            status += String(Limit_Y2_start);
            status += "$";
            status += String(Limit_Y2_end);
            status += "$";
            status += String(Limit_X_start);
            status += "$";
            status += String(Limit_X_end);
            status += "$";
            status += String(Limit_Z_start);
            status += "$";
            status += String(Limit_Z_end);
            status += "$";
            status += drawFunction;
            status += "$";
            status += drawIndex;
            status += "$";
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
}

void getSerial(int bytesToRead) {
    // read chars from serial port and convert to a byte array
    char charbuffer[bytesToRead];
    byte bytebuffer[bytesToRead];
    Serial.readBytes(charbuffer, bytesToRead);
    memcpy(bytebuffer, charbuffer, bytesToRead);

    // iterate over all the bytes
    for (int i = 0; i < bytesToRead; i++) {
        //// check for home command (start 10 x 0xF0)
        if (bytebuffer[i] == 0xF0) {
            if (serialHomeHeaderCount == 9) {
                Serial.println("Received HOME command");
                serialHomeHeaderCount = 0;
            } else {
                serialHomeHeaderCount++;
            }
        } else {
            serialHomeHeaderCount = 0;
        }

        //// check for Draw command (start 10 x 0xF1)
        if (bytebuffer[i] == 0xF1) {
            if (serialDrawHeaderCount == 9) {
                Serial.println("Received Draw command");
                serialDrawHeaderCount = 0;
            } else {
                serialDrawHeaderCount++;
            }
        } else {
            serialDrawHeaderCount = 0;
        }

        //// check for Reset command (start 10 x 0xF2)
        if (bytebuffer[i] == 0xF2) {
            if (serialResetHeaderCount == 9) {
                Serial.println("Received Reset command");
                serialResetHeaderCount = 0;
            } else {
                serialResetHeaderCount++;
            }
        } else {
            serialResetHeaderCount = 0;
        }

        //// check for HeightMap command (start 10 x 0xF3)
        if (bytebuffer[i] == 0xF3) {
            if (serialHeightMapHeaderCount == 9) {
                Serial.println("Received HeightMap command");
                serialHeightMapHeaderCount = 0;
            } else {
                serialHeightMapHeaderCount++;
            }
        } else {
            serialHeightMapHeaderCount = 0;
        }

        //// check for pause command (start 10 x 0xF4)
        if (bytebuffer[i] == 0xF4) {
            if (serialPauseHeaderCount == 9) {
                Serial.println("Received PAUSE command");
                serialPauseHeaderCount = 0;
            } else {
                serialPauseHeaderCount++;
            }
        } else {
            serialPauseHeaderCount = 0;
        }

        //// check for instructions (start 10 x 0xFF)
        if (!serialInstructionStarted) {
            if (serialInstructionHeaderCount == 10) {
                // we have read exactly 10 startbytes (0xFF)
                serialInstructionStarted = true;
                serialMessageSize = bytebuffer[i];
                serialMessageByteCount = 0;
                serialMessageChecksumByteCount = 0;
                serialMessageReceivedChecksum.value = 0;
            } else {
                if (bytebuffer[i] == 0xFF) {
                    serialInstructionHeaderCount++;
                } else {
                    // we did not receive a correct header, reset check
                    serialInstructionHeaderCount = 0;
                }
            }
        } else {
            if (serialMessageByteCount < serialMessageSize) {
                // add byte to our checksum
                serialMessageCalculatedChecksum += bytebuffer[i];
                serialMessageData[serialMessageByteCount] = bytebuffer[i];
                serialMessageByteCount++;
            } else {
                // we should have received a complete message now, verify the checksum
                // the next 4 bytes are the checksum

                serialMessageReceivedChecksum.bytes[serialMessageChecksumByteCount] = bytebuffer[i];
                serialMessageChecksumByteCount++;

                if (serialMessageChecksumByteCount == 3) {
                    requestTimer = 0;
                    requestCounter = 0;
                    if (serialMessageReceivedChecksum.value == serialMessageCalculatedChecksum) {
                        // get Index
                        byte64 receivedIndex = {};
                        // memcpy(receivedIndex.bytes, bytebuffer + 11, 8); // nope we shoud use our serialMessageDATA!
                        memcpy(receivedIndex.bytes, serialMessageData, 8);

                        Serial.print("Received #");
                        Serial.print(receivedIndex.value);
                        Serial.print(" :");
                        Serial.print(serialMessageSize);
                        Serial.print(" bytes, checksum ");
                        Serial.print(serialMessageCalculatedChecksum);
                        Serial.println(" ok!");

                        if (receivedIndex.value == requestedInstruction) {
                            // All is good we have the requested instruction
                            // Write it to the current index!
                            byte64 b64 = {};
                            // byte64 b32 = {};

                            // grab all bytes and put them into drawinstruction struct
                            // remember we interleaved bytes with zero bytes as binary protocol

                            memcpy(b64.bytes, serialMessageData, 8);
                            iBuffer[iBufferWriteIndex].index = b64.value;
                            iBuffer[iBufferWriteIndex].type = serialMessageData[9];
                            iBuffer[iBufferWriteIndex].dirX = serialMessageData[11];
                            iBuffer[iBufferWriteIndex].dirY = serialMessageData[13];
                            memcpy(b64.bytes, serialMessageData + 15, 8);
                            iBuffer[iBufferWriteIndex].startX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 24, 8);
                            iBuffer[iBufferWriteIndex].startY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 33, 8);
                            iBuffer[iBufferWriteIndex].endX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 42, 8);
                            iBuffer[iBufferWriteIndex].endY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 51, 8);
                            iBuffer[iBufferWriteIndex].deltaX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 60, 8);
                            iBuffer[iBufferWriteIndex].deltaY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 69, 8);
                            iBuffer[iBufferWriteIndex].deltaXX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 78, 8);
                            iBuffer[iBufferWriteIndex].deltaYY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 87, 8);
                            iBuffer[iBufferWriteIndex].deltaXY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 96, 8);
                            iBuffer[iBufferWriteIndex].error = b64.value;
                            memcpy(b64.bytes, serialMessageData + 105, 8);
                            iBuffer[iBufferWriteIndex].steps = b64.value;

                            iBufferWriteIndex = (iBufferWriteIndex + 1) & 63;
                            recievedInstruction = receivedIndex.value;

                            // we already checked this before sending a request
                            if (((iBufferWriteIndex + 1) & 63) != iBufferReadIndex) {
                            }
                        } else {
                            Serial.print("But index ");
                            Serial.print(receivedIndex.value);
                            Serial.println(" does not match!");
                            Serial.print("requesting message ");
                            Serial.print(requestedInstruction);
                            Serial.println(" again..");
                            Serial.print("@");
                            Serial.print(requestedInstruction);
                            Serial.println("@ requesting instruction ");
                            delay(100);
                        }
                    } else {
                        Serial.print("Received data, checksum ");
                        Serial.print(serialMessageCalculatedChecksum);
                        Serial.println(" is bad! message corrupt ?");
                        Serial.println("requesting message again..");
                        Serial.print("@");
                        Serial.print(requestedInstruction);
                        Serial.println("@ requesting instruction ");
                    }

                    // restart
                    serialInstructionStarted = false;
                    serialInstructionHeaderCount = 0;
                    serialMessageByteCount = 0;
                    serialMessageCalculatedChecksum = 0;
                }
            }
        }
    }
}
