#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h> // https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SH110X.h> // https://github.com/adafruit/Adafruit_SH110x
#include <Adafruit_PWMServoDriver.h> // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <DmxInput.h> // https://github.com/jostlowe/Pico-DMX


#define START_CHANNEL 1
#define NUM_CHANNELS 8
#define MIN_MOVE_INTERVAL_MS 3

#define PWM_ADDR 0x40
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

//#define DEBUG

DmxInput dmxInput;
volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

Adafruit_SH1106G display = Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR, Wire);

// Minimum and maximum pwm values to set as -90/90 degrees.
// We found by testing in the servos we had that the following
// min/max values result in a 180 rotation range.
int servo_min = 128;
int servo_max = 512;

int pos = 0;
int last_update = 0;

void setup()
{
    // Setup our DMX Input to read on GPIO 0, from channel 1 to NUM_CHANNELS+1
    dmxInput.begin(0, START_CHANNEL, NUM_CHANNELS);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);

#ifdef DEBUG
    Serial.begin(9600);
    Serial.println("STARTING DMX INPUT");
#endif

    display.begin(0x3C, true);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(6,0);
    display.println("DMX servo controller");
    display.println();
    display.println();
    display.println();
    display.println();
    display.println("Waiting for input...");
    display.display();

    pwm.begin();
    pwm.setPWMFreq(50);
}

void show_channel_value(int n, int value) {
        display.print("Channel ");
        display.print(n);
        display.print(": ");
        display.println(value);
}

void loop()
{
    // Wait for next DMX packet
    dmxInput.read(buffer);

    display.clearDisplay();
    display.setCursor(0,0);

    for (uint i = 1; i < sizeof(buffer); i++)
    {
        show_channel_value(i, buffer[i]);
        if (i == 1) {
          int current_time = millis();
          if ((current_time - last_update) >= MIN_MOVE_INTERVAL_MS) {
            pos = map(buffer[i], 0, 255, servo_min, servo_max);
            last_update = current_time;
          }
        }
        if (i == 2) {
          servo_min = map(buffer[i], 0, 255, 0, 800);
        }
        if (i == 3) {
          servo_max = map(buffer[i], 0, 255, 300, 1200);
        }
    }

#ifdef DEBUG
    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.print(", min: ");
    Serial.print(servo_min);
    Serial.print(", max: ");
    Serial.print(servo_max);
    Serial.println();
#endif

    display.display();

    pwm.setPWM(0, 0, pos);
    pwm.setPWM(15, 0, pos);
}
