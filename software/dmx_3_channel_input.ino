/*
 * Based on example code by Jostein Løwer
 *
 * Starts a DMX Input on GPIO pin 0 and read channel 1-3 repeatedly
 */

#include <Arduino.h>
#include "DmxInput.h"
DmxInput dmxInput;

#define START_CHANNEL 1
#define NUM_CHANNELS 8

volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

void setup()
{
    // Setup our DMX Input to read on GPIO 0, from channel 1 to 3
    dmxInput.begin(0, START_CHANNEL, NUM_CHANNELS);

    // Setup the onboard LED so that we can blink when we receives packets
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    Serial.println("STARTING DMX INPUT");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    // Wait for next DMX packet
    dmxInput.read(buffer);

    // Print the DMX channels
    Serial.print("Received packet: ");
    for (uint i = 1; i < sizeof(buffer); i++)
    {
        Serial.print(buffer[i]);
        Serial.print(", ");
    }
    Serial.println("");

    // Blink the LED to indicate that a packet was received
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}
