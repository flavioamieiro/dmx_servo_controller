#include <Arduino.h>//
#include <Wire.h>                                                                     //////////////////////////////////////////////////////////
#include <SPI.h>                     //                                               |                 Adafruit_SSD1306.h                     |
#include <Adafruit_GFX.h> // https://github.com/adafruit/Adafruit-GFX-Library         | VERSION 2.5.1 otherwise it fails PGMspace include lib  |
#include <Adafruit_SSD1306.h> // https://github.com/adafruit/Adafruit_SSD1306  -----  //////////////////////////////////////////////////////////                                                
#include <Adafruit_PWMServoDriver.h> // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <DmxInput.h> // https://github.com/jostlowe/Pico-DMX





#define PWM_ADDR 0x3C
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define OLED_RESET     -1




// Minimum and maximum pwm values to set as -90/90 degrees.
// We found by testing in the servos we had that the following
// min/max values result in a 180 rotation range.
#define DEFAULT_SERVO_MIN 64
#define DEFAULT_SERVO_MAX 512
#define DEFAULT_SERVO_DMX_VALUE 127

/*VERBOSE*/
#define DEBUG
#define PRINT_DMX_MESSAGES


/*DMX INIT*/
DmxInput dmxInput;
#define NUM_CHANNELS 4
#define DMX_INPUT_PIN 18
byte ledState = LOW;

volatile uint8_t START_CHANNEL = 3;
volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(START_CHANNEL, NUM_CHANNELS)];


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
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);

#ifdef DEBUG
    Serial.begin(9600);
    Serial.println("STARTING DMX INPUT");
#endif

    dmxInput.begin(DMX_INPUT_PIN, START_CHANNEL, NUM_CHANNELS);
    initialize_dmx_buffer();
    dmxInput.read_async(buffer);

  
}

void loop()
{
    handle_dmx_message();
    delay(50);
}



void handle_dmx_message() {
    if(millis() > 100+dmxInput.latest_packet_timestamp()) {
        #ifdef DEBUG
        #ifdef PRINT_DMX_MESSAGES
            Serial.println("no data!");
        #endif
        #endif
        digitalWrite(LED_BUILTIN, HIGH);
        return;
    }
    else{
        for (uint i = 1; i < NUM_CHANNELS+1; i++)
        {
            struct Servo *current_servo = &servos[i-1];
            update_servo(current_servo, buffer[i]);
        }
        ledState = (ledState == HIGH) ? LOW: HIGH;
    }
    digitalWrite(LED_BUILTIN, ledState);
}

void initialize_dmx_buffer(){
    for(uint i = 0; i < sizeof(buffer); i++)
    buffer[i] = 0;
}

void start_addr_changer(uint newChan){
    dmxInput.end();
    START_CHANNEL = newChan;
    initialize_dmx_buffer();
    dmxInput.begin(DMX_INPUT_PIN, START_CHANNEL, NUM_CHANNELS);
    
}

void initialize_servos() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      servos[i].dmx_channel = i+1;
      servos[i].pwm_channel = i;
      servos[i].dmx_value =   DEFAULT_SERVO_DMX_VALUE;
      servos[i].min_pos =     DEFAULT_SERVO_MIN;
      servos[i].max_pos =     DEFAULT_SERVO_MAX;
      servos[i].last_update = 0;
    }
}


void update_servo(struct Servo *servo, int new_dmx_value) {
    servo->dmx_value = new_dmx_value;
    servo->last_update = millis();
    int pos = map(servo->dmx_value, 0, 255, servo->min_pos, servo->max_pos);
    //pwm.setPWM(servo->pwm_channel, 0, pos);

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
