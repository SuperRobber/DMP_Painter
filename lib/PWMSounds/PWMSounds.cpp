#include "PWMSounds.h"

void playBeep(uint8_t AudioPin)
{
    // analogWriteResolution(16);
    analogWriteFrequency(AudioPin, 400);
    analogWrite(AudioPin, 80);
    delay(30);

    analogWrite(AudioPin, 0);
    delay(100);

    analogWriteFrequency(AudioPin, 600);
    analogWrite(AudioPin, 80);
    delay(30);

    analogWrite(AudioPin, 0);
    delay(100);
}

void playVictorySound(uint8_t AudioPin)
{
    for (int y = 0; y < 3; y++)
    {
        for (int i = 0; i < 4; i++)
        {
            analogWriteFrequency(AudioPin, 400);
            analogWrite(AudioPin, 10 + i);
            delay(15);
            analogWriteFrequency(AudioPin, 100);
            analogWrite(AudioPin, 10 + i);
            delay(15);
            analogWriteFrequency(AudioPin, 200);
            analogWrite(AudioPin, 10 + i);
            delay(15);
        }
        analogWrite(AudioPin, 0);
        delay(20);
    }

    for (int i = 0; i < 20; i++)
    {
        analogWriteFrequency(AudioPin, 400);
        analogWrite(AudioPin, 10 + i);
        delay(18);
        analogWriteFrequency(AudioPin, 300);
        analogWrite(AudioPin, 10 + i);
        delay(18);
        analogWriteFrequency(AudioPin, 200);
        analogWrite(AudioPin, 10 + i);
        delay(21);
    }

    for (int i = 0; i < 20; i++)
    {
        analogWriteFrequency(AudioPin, 400);
        analogWrite(AudioPin, 20 + i);
        delay(18);
        analogWriteFrequency(AudioPin, 200);
        analogWrite(AudioPin, 30 + i);
        delay(18);
        analogWriteFrequency(AudioPin, 500);
        analogWrite(AudioPin, 10 + i);
        delay(21);
    }
    analogWrite(AudioPin, 0);
    delay(50);
    for (int i = 0; i < 40; i++)
    {
        analogWriteFrequency(AudioPin, 400);
        analogWrite(AudioPin, 10 + i * 2);
        delay(15 + i * 2);
        analogWriteFrequency(AudioPin, 600);
        analogWrite(AudioPin, 10 + i * 2);
        delay(15 + i * 3);
        analogWriteFrequency(AudioPin, 500);
        analogWrite(AudioPin, 10 + i * 2);
        delay(15 + i * 2);
    }
    analogWriteFrequency(AudioPin, 400);
    analogWrite(AudioPin, 80);
    delay(110);

    analogWriteFrequency(AudioPin, 200);
    analogWrite(AudioPin, 120);
    delay(1000);

    analogWrite(AudioPin, 0);
    delay(500);

    analogWriteFrequency(AudioPin, 400);
    analogWrite(AudioPin, 60);
    delay(50);

    analogWriteFrequency(AudioPin, 600);
    analogWrite(AudioPin, 60);
    delay(200);

    analogWrite(AudioPin, 00);
    delay(50);

    analogWriteFrequency(AudioPin, 800);
    analogWrite(AudioPin, 100);
    delay(1000);

    analogWrite(AudioPin, 0);
}