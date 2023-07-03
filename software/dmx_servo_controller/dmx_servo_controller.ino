#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h> // https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h> // https://github.com/adafruit/Adafruit_SSD1306
#include <Adafruit_PWMServoDriver.h> // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <DmxInput.h> // https://github.com/jostlowe/Pico-DMX
#include <RotaryEncoder.h> // https://github.com/mathertel/RotaryEncoder


#define START_CHANNEL 1
#define NUM_CHANNELS 8

#define PWM_ADDR 0x40
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define OLED_RESET     -1

#define DMX_INPUT_PIN 18
#define PIN_ENCODER_PUSH 7
#define PIN_ENCODER_A 9
#define PIN_ENCODER_B 8

// Minimum and maximum pwm values to set as -90/90 degrees.
// We found by testing in the servos we had that the following
// min/max values result in a 180 rotation range.
#define DEFAULT_SERVO_MIN 128
#define DEFAULT_SERVO_MAX 512
#define DEFAULT_SERVO_DMX_VALUE 127

#define DEBUG

DmxInput dmxInput;
volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];

Adafruit_SSD1306 display = Adafruit_SSD1306(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR, Wire);

RotaryEncoder encoder(PIN_ENCODER_A, PIN_ENCODER_B, RotaryEncoder::LatchMode::TWO03);
int last_rotary_pos = 0;

struct Servo {
  int dmx_channel;
  int pwm_channel;
  int dmx_value;
  int min_pos;
  int max_pos;
  int last_update;
};

struct Servo servos[NUM_CHANNELS];

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_ENCODER_PUSH, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), update_encoder_position, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), update_encoder_position, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_PUSH), encoder_pushed, FALLING);

#ifdef DEBUG
    Serial.begin(9600);
    Serial.println("STARTING DMX INPUT");
#endif

    dmxInput.begin(DMX_INPUT_PIN, START_CHANNEL, NUM_CHANNELS);
    dmxInput.read_async(buffer);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true);
    pwm.begin();
    pwm.setPWMFreq(50);

    initialize_servos();
    show_splash_screen();
}

void loop()
{
    handle_dmx_message();
    handle_rotary_encoder();
    update_display();
}

void update_display() {
    display.clearDisplay();
    display.setCursor(0,0);

    for (uint i = 0; i < NUM_CHANNELS; i++)
    {
        show_channel_value(i+1, servos[i].dmx_value);
    }

    display.display();
}

void handle_dmx_message() {
    for (uint i = 1; i < NUM_CHANNELS+1; i++)
    {
        struct Servo *current_servo = &servos[i-1];
        update_servo(current_servo, buffer[i]);
    }
}

void handle_rotary_encoder() {
    int curr_rotary_pos = encoder.getPosition();
    RotaryEncoder::Direction direction = encoder.getDirection();
    if (curr_rotary_pos != last_rotary_pos) {
#ifdef DEBUG
      Serial.print("Encoder value: ");
      Serial.print(curr_rotary_pos);
      Serial.print(" direction: ");
      Serial.println((int)direction);
#endif
    }
    last_rotary_pos = curr_rotary_pos;
}

void show_channel_value(int n, int value) {
        display.print("Channel ");
        display.print(n);
        display.print(": ");
        display.println(value);
}

void show_splash_screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(6,0);
    display.println("DMX servo controller");
    display.println();
    display.println();
    display.println();
    display.println();
    display.println("Waiting for input...");
    display.display();
}


void initialize_servos() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      servos[i].dmx_channel = i+1;
      servos[i].pwm_channel = i;
      servos[i].dmx_value = DEFAULT_SERVO_DMX_VALUE;
      servos[i].min_pos = DEFAULT_SERVO_MIN;
      servos[i].max_pos = DEFAULT_SERVO_MAX;
      servos[i].last_update = 0;
    }
}


void update_servo(struct Servo *servo, int new_dmx_value) {
    servo->dmx_value = new_dmx_value;
    servo->last_update = millis();
    int pos = map(servo->dmx_value, 0, 255, servo->min_pos, servo->max_pos);
    pwm.setPWM(servo->pwm_channel, 0, pos);
#ifdef DEBUG
#ifdef PRINT_DMX_MESSAGES
    Serial.print("dmx_value: ");
    Serial.print(servo->dmx_value);
    Serial.print(", pos: ");
    Serial.print(pos);
    Serial.print(", min: ");
    Serial.print(servo->min_pos);
    Serial.print(", max: ");
    Serial.print(servo->max_pos);
    Serial.println();
#endif
#endif
}

void encoder_pushed() {
#ifdef DEBUG
  Serial.println("Encoder pushed");
#endif
}

void update_encoder_position() {
  encoder.tick();
}
