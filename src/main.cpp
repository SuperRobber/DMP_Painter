#include <Arduino.h>
#include <SPI.h>

#include "MotionControl.h"

elapsedMillis disconnectTimer;
elapsedMillis statusTimer;
elapsedMillis requestTimer;
elapsedMicros drawTimeout;

/// A module to measure power usage (ampere) is connected via SPI0

const int powerSenseCSPin = 34;
SPISettings spiPowerSenseConfig(16000000, MSBFIRST, SPI_MODE0);
int32_t powerSenseData = 0;

/// ===================== DATA Layout =====================

/// Incoming serial data can consist of either commands or drawing instructions.

/// A custom serial binary protocol is used (bytes instead of chars) and
/// consists of a 10byte command header with additional data if data is a
/// drawing instruction.

/// 10 byte header indicating a command (mode) or drawInstruction data

/// HOME command                (10 x 0xF0)
/// DRAW command                (10 x 0xF1)
/// RESET command               (10 x 0xF2)
/// MAPHEIGHT command           (10 x 0xF3)
/// STOP command                (10 x 0xF4)
/// EOL command                 (10 x 0xF5)
/// CLEARHEIGHT command         (10 x 0xF6)
/// DRAWINSTRUCTION command     (10 x 0xFF)

/// DrawInstruction:
/// - 10 byte header 0xFF
//  - 1 byte containing the size of the data to be received (N)
//  - N number of values up to 8byte (int64) each, with a 0x00 byte in between
//  - a checksum calculated on the original data.

enum command
{
    BYTE_HOME = 0xF0,
    BYTE_DRAW = 0xF1,
    BYTE_RESET = 0xF2,
    BYTE_MAPHEIGHT = 0xF3,
    BYTE_STOP = 0xF4,
    BYTE_EOL = 0xF5,
    BYTE_CLEARHEIGHT = 0xF6,
    BYTE_DRAW_INSTRUCTION = 0xFF
};

/// ===================== Serial protocol =====================

byte serialMessageData[250]; /// 250bytes reserved for serial message data.
int serialMessageSize = 0;
int serialMessageByteCount = 0;
int serialMessageChecksumByteCount = 0;
bool serialInstructionStarted = false;
int32_t serialMessageCalculatedChecksum = 0;
byte32 serialMessageReceivedChecksum = {0};

// counters to parse data headers (see DATA LAYOUT)

int serialInstructionHeaderCount = 0;
int serialHomeHeaderCount = 0;
int serialStopHeaderCount = 0;
int serialHeightMapHeaderCount = 0;
int serialClearHeightMapHeaderCount = 0;
int serialResetHeaderCount = 0;
int serialDrawHeaderCount = 0;
int serialEOLHeaderCount = 0;

void getSerial(int bytesToRead);

/// ===================== setup - Configure and initialise hardware. =====================

void setup()
{
    Serial.begin(115200);

    pinMode(powerSenseCSPin, OUTPUT);
    digitalWriteFast(powerSenseCSPin, HIGH);

    // PowerSense on SPI0
    SPI.begin();

    delay(100);

    configureSwitches();
    configureStepperDrivers();

    StartUp();
}

/// ===================== loop  - Main program loop =====================

void loop()
{

    /// Connected to the Loader program?
    if (Serial.dtr())
    {
        disconnectTimer = 0;

        /// Process pending data from Loader program.
        int bytesToRead = Serial.available();
        if (bytesToRead > 0)
        {
            getSerial(bytesToRead);
        }

        if (activeMode == Mode::Draw && requestedMode != Mode::EOL && requestedMode != Mode::Stop)
        {
            /// Request draw instructions if there is room in the buffer. If the
            /// buffer is full, skip and wait for drawing to advance, before
            /// requesting a new instruction.
            if (((iBufferWriteIndex + 1) & 63) != iBufferReadIndex)
            {
                /// Have we received our latest request?
                if (receivedInstruction == requestedInstruction)
                {
                    /// Yes, request a new instruction to add into the buffer
                    requestedInstruction++;
                    Serial.print("@");
                    Serial.print(requestedInstruction);
                    Serial.println("@ requesting instruction ");
                    requestTimer = 0;
                }
                else
                {
                    /// No, request again every 10 seconds
                    if (requestTimer > 10000)
                    {
                        Serial.print("@");
                        Serial.print(requestedInstruction);
                        Serial.print("@ requesting instruction.");
                        requestTimer = 0;
                    }
                }
            }
        }

        /// Update status 20 frames per second.
        if (statusTimer > 50)
        {
            statusTimer = 0;
            String status = "";

            /// Read powerSense module
            SPI.beginTransaction(spiPowerSenseConfig);
            digitalWriteFast(powerSenseCSPin, LOW);
            powerSenseData = SPI.transfer16(0);
            digitalWriteFast(powerSenseCSPin, HIGH);
            SPI.endTransaction();

            /// Check stepper motor drives for error and update status flags.
            updateStepperStatus();

            /// Gather all status information into a status string. Use "$"" as
            /// a separator character for parsing in the Loader program.
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
            status += String((powerSenseData - 2047) * 12);
            status += "$";
            for (int s = 0; s < numSwitches; s++)
            {
                status += String(switches[s].pressed);
                status += "$";
            }
            status += (int) statusFunction;
            status += "$";
            status += drawIndex;
            status += "$";

            /// Send status to Loader program.
            Serial.println(status);
        }
    }
    else
    {
        if (disconnectTimer > 60000)
        {
            /// We are not / no longer connected. All drawing instructions that
            /// are currently in buffer will still be processed.

            /// TODO: Pen up if we are in drawing state ?  Go home?
        }
    }
}
/// @brief Parse data from serial buffer into commands or drawing instructions
/// @param bytesToRead Number of bytes to process.
void getSerial(int bytesToRead)
{
    // read chars from serial port and convert to a byte array
    char charBuffer[bytesToRead];
    byte byteBuffer[bytesToRead];
    Serial.readBytes(charBuffer, bytesToRead);
    memcpy(byteBuffer, charBuffer, bytesToRead);

    // iterate over all the bytes
    for (int i = 0; i < bytesToRead; i++)
    {
        //// check for HOME command header (10 x 0xF0)
        if (byteBuffer[i] == command::BYTE_HOME)
        {
            serialHomeHeaderCount++;
            if (serialHomeHeaderCount == 10)
            {
                Serial.println("Received Home command");
                requestedMode = Mode::Home;
                serialHomeHeaderCount = 0;
            }
        }
        else
        {
            serialHomeHeaderCount = 0;
        }

        //// check for DRAW command header (10 x 0xF1)
        if (byteBuffer[i] == command::BYTE_DRAW)
        {
            serialDrawHeaderCount++;
            if (serialDrawHeaderCount == 10)
            {
                Serial.println("Received Draw command");
                requestedMode = Mode::Draw;
                serialDrawHeaderCount = 0;
            }
        }
        else
        {
            serialDrawHeaderCount = 0;
        }

        /// Check for RESET command header.
        if (byteBuffer[i] == command::BYTE_RESET)
        {
            serialResetHeaderCount++;
            if (serialResetHeaderCount == 10)
            {
                Serial.println("Received Reset command");
                requestedMode = Mode::Reset;
                serialResetHeaderCount = 0;
            }
        }
        else
        {
            serialResetHeaderCount = 0;
        }

        /// Check for MAPHEIGHT command header.
        if (byteBuffer[i] == command::BYTE_MAPHEIGHT)
        {
            serialHeightMapHeaderCount++;
            if (serialHeightMapHeaderCount == 10)
            {
                Serial.println("Received HeightMap command");
                requestedMode = Mode::MapHeight;
                serialHeightMapHeaderCount = 0;
            }
        }
        else
        {
            serialHeightMapHeaderCount = 0;
        }

        /// Check for STOP command header.
        if (byteBuffer[i] == command::BYTE_STOP)
        {
            serialStopHeaderCount++;
            if (serialStopHeaderCount == 10)
            {
                Serial.println("Received Stop command");
                requestedMode = Mode::Stop;
                serialStopHeaderCount = 0;
            }
        }
        else
        {
            serialStopHeaderCount = 0;
        }

        /// Check for EOF command header.
        if (byteBuffer[i] == command::BYTE_EOL)
        {
            serialEOLHeaderCount++;
            if (serialEOLHeaderCount == 10)
            {
                Serial.println("Received EOF command");
                requestedMode = Mode::EOL;
                serialEOLHeaderCount = 0;
            }
        }
        else
        {
            serialEOLHeaderCount = 0;
        }

        //// Check for CLEARHEIGHT command header.
        if (byteBuffer[i] == command::BYTE_CLEARHEIGHT)
        {
            serialClearHeightMapHeaderCount++;
            if (serialClearHeightMapHeaderCount == 10)
            {
                Serial.println("Received Clear Height command");
                requestedMode = Mode::ClearHeight;
                serialClearHeightMapHeaderCount = 0;
            }
        }
        else
        {
            serialClearHeightMapHeaderCount = 0;
        }

        //// check for DRAW_INSTRUCTION header.
        if (!serialInstructionStarted)
        {
            if (serialInstructionHeaderCount == 10)
            {
                /// 10 DRAW_INSTRUCTION header bytes have been read in a row.
                /// The following bytes belongs to a draw instruction and the
                /// current byte (11) is the MessageSize.
                serialInstructionStarted = true;
                serialMessageSize = byteBuffer[i];
                serialMessageByteCount = 0;
                serialMessageChecksumByteCount = 0;
                serialMessageReceivedChecksum.value = 0;
            }

            if (byteBuffer[i] == command::BYTE_DRAW_INSTRUCTION)
            {
                serialInstructionHeaderCount++;
            }
            else
            {
                serialInstructionHeaderCount = 0;
            }
        }
        else
        {
            if (serialMessageByteCount < serialMessageSize)
            {
                // Add byte to the checksum.
                serialMessageCalculatedChecksum += byteBuffer[i];
                // Add byte to the message.
                serialMessageData[serialMessageByteCount] = byteBuffer[i];
                // Increment message byte count.
                serialMessageByteCount++;
            }
            else
            {
                /// A complete message should have been received.
                /// The next 4 bytes are the checksum.

                serialMessageReceivedChecksum.bytes[serialMessageChecksumByteCount] = byteBuffer[i];
                serialMessageChecksumByteCount++;

                if (serialMessageChecksumByteCount == 3)
                {
                    /// Verify the checksum.
                    if (serialMessageReceivedChecksum.value == serialMessageCalculatedChecksum)
                    {
                        /// Valid draw instruction received.

                        /// Copy the instruction index
                        byte64 receivedIndex = {};
                        memcpy(receivedIndex.bytes, serialMessageData, 8);

                        Serial.print("Received #");
                        Serial.print(receivedIndex.value);
                        Serial.print(" :");
                        Serial.print(serialMessageSize);
                        Serial.print(" bytes, checksum ");
                        Serial.print(serialMessageCalculatedChecksum);
                        Serial.println(" ok!");

                        if (receivedIndex.value == requestedInstruction)
                        {
                            /// Instruction indices match.
                            /// The requested instruction is received correctly.

                            /// Parse values directly into a new drawInstruction in our drawInstructionBuffer.
                            /// @attention Interleave values with a zero byte as part of the binary protocol

                            /// A byte64 union is used to convert 8 bytes to int64.
                            byte64 b64 = {};
                            ubyte64 u64 = {};

                            memcpy(u64.bytes, serialMessageData, 8);
                            iBuffer[iBufferWriteIndex].index = u64.value;
                            iBuffer[iBufferWriteIndex].type = serialMessageData[9];
                            iBuffer[iBufferWriteIndex].acceleration = serialMessageData[11];
                            iBuffer[iBufferWriteIndex].dirX = serialMessageData[13];
                            iBuffer[iBufferWriteIndex].dirY = serialMessageData[15];
                            iBuffer[iBufferWriteIndex].dirZ = serialMessageData[17];
                            iBuffer[iBufferWriteIndex].projection = serialMessageData[19];
                            memcpy(b64.bytes, serialMessageData + 21, 8);
                            iBuffer[iBufferWriteIndex].startX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 30, 8);
                            iBuffer[iBufferWriteIndex].startY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 39, 8);
                            iBuffer[iBufferWriteIndex].startZ = b64.value;
                            memcpy(b64.bytes, serialMessageData + 48, 8);
                            iBuffer[iBufferWriteIndex].endX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 57, 8);
                            iBuffer[iBufferWriteIndex].endY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 66, 8);
                            iBuffer[iBufferWriteIndex].endZ = b64.value;
                            memcpy(b64.bytes, serialMessageData + 75, 8);
                            iBuffer[iBufferWriteIndex].deltaX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 84, 8);
                            iBuffer[iBufferWriteIndex].deltaY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 93, 8);
                            iBuffer[iBufferWriteIndex].deltaZ = b64.value;
                            memcpy(b64.bytes, serialMessageData + 102, 8);
                            iBuffer[iBufferWriteIndex].deltaXX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 111, 8);
                            iBuffer[iBufferWriteIndex].deltaYY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 120, 8);
                            iBuffer[iBufferWriteIndex].deltaZZ = b64.value;
                            memcpy(b64.bytes, serialMessageData + 129, 8);
                            iBuffer[iBufferWriteIndex].deltaXY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 138, 8);
                            iBuffer[iBufferWriteIndex].deltaXZ = b64.value;
                            memcpy(b64.bytes, serialMessageData + 147, 8);
                            iBuffer[iBufferWriteIndex].deltaYZ = b64.value;
                            memcpy(b64.bytes, serialMessageData + 156, 8);
                            iBuffer[iBufferWriteIndex].error = b64.value;
                            memcpy(b64.bytes, serialMessageData + 165, 8);
                            iBuffer[iBufferWriteIndex].errorX = b64.value;
                            memcpy(b64.bytes, serialMessageData + 174, 8);
                            iBuffer[iBufferWriteIndex].errorY = b64.value;
                            memcpy(b64.bytes, serialMessageData + 183, 8);
                            iBuffer[iBufferWriteIndex].errorZ = b64.value;
                            memcpy(u64.bytes, serialMessageData + 192, 8);
                            iBuffer[iBufferWriteIndex].steps = (double)u64.value;

                            // Increment the bufferWriteIndex for an upcoming instruction.
                            iBufferWriteIndex = (iBufferWriteIndex + 1) & 63;

                            // Finally confirm the processed instuction.
                            receivedInstruction = receivedIndex.value;
                        }
                        else
                        {
                            /// A valid drawInstuction was received. But it did
                            /// not have the correct index.
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
                    }
                    else
                    {
                        /// The checksum did not match.
                        Serial.print("Received data, checksum ");
                        Serial.print(serialMessageCalculatedChecksum);
                        Serial.println(" is bad! message corrupt ?");
                        Serial.println("requesting message again..");
                        Serial.print("@");
                        Serial.print(requestedInstruction);
                        Serial.println("@ requesting instruction ");
                    }

                    // Reset variables for an upcoming instruction.
                    serialInstructionStarted = false;
                    serialInstructionHeaderCount = 0;
                    serialMessageByteCount = 0;
                    serialMessageCalculatedChecksum = 0;
                }
            }
        }
    }
}
