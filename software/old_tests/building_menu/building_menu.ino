
#include <Arduino.h>
#include <Wire.h>                                                                     
#include <SPI.h>     

#include <Adafruit_SSD1306.h> //                                  
//////////////////////////////////////////////////////////////
//|                 Adafruit_SSD1306.h                     |//
//| VERSION 2.5.1 otherwise it fails PGMspace include lib  |//
//|     https://github.com/adafruit/Adafruit_SSD1306       |//
//|       https://github.com/alanesq/BasicOLEDMenu         |//
//////////////////////////////////////////////////////////////
 
#include <Adafruit_GFX.h> //                                      https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_PWMServoDriver.h> //                           https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <DmxInput.h> //                                          https://github.com/jostlowe/Pico-DMX

extern "C" {//                                                    https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
  #include <hardware/sync.h>
  #include <hardware/flash.h>
};

// ----------------------------------------------------------------
//                         S E T T I N G S
// ----------------------------------------------------------------
// Encoder
  #define encoder0PinA    9                    // Rotary encoder gpio pin
  #define encoder0PinB    8                    // Rotary encoder gpio pin
  #define encoder0Press   7                    // Rotary encoder button gpio pin

// oLED
  #define OLED_ADDR       0x3C                 // OLED i2c address
  #define SCREEN_WIDTH    128                  // OLED display width, in pixels (usually 128)
  #define SCREEN_HEIGHT   32                   // OLED display height, in pixels (64 for larger oLEDs)
  #define OLED_RESET      -1                   // Reset pin gpio (or -1 if sharing Arduino reset pin)
  #define SSD1306_NO_SPLASH

// PWM driver
  #define PWM_ADDR        0x40                 // PCA9685 i2c address

// SERVO
// Minimum and maximum pwm values to set as -90/90 degrees.
// We found by testing in the servos we had that the following
// min/max values result in a 180 rotation range.
  #define DEFAULT_SERVO_MIN       64
  #define DEFAULT_SERVO_MAX       512
  #define DEFAULT_SERVO_DMX_VALUE 127

// DMX512
  DmxInput dmxInput;
  #define   NUM_CHANNELS    4
  uint16_t  START_CHANNEL   1       //------------------------------------------- we need to work on this one 
  #define   DMX_INPUT_PIN   18
  byte ledState = LOW;

//Pi Pico - Flash memory
// Set the target offest to the last sector of flash
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

//---------------VERBOSE---------------
  #define DEBUG
  #define PRINT_DMX_MESSAGES
  const String SOFTWARE_VERSION = "0.1";

// Misc
  #define     BUTTONPRESSEDSTATE    0        // rotary encoder gpio pin logic level when the button is pressed (usually 0)
  #define     DEBOUNCEDELAY         20       // debounce delay for button inputs
  const int   menuTimeout =         999;     // menu inactivity timeout (seconds)
  const bool  menuLargeText =       0;       // show larger text when possible (if struggling to read the small text)
  const int   maxmenuItems =        10;      // max number of items used in any of the menus (keep as low as possible to save memory)
  const int   itemTrigger =         2;       // rotary encoder - counts per tick (varies between encoders usually 1 or 2)
  const int   topLine =             9;       // y position of lower area of the display (18 with two colour displays)
  const byte  lineSpace1 =          6;       // line spacing for textsize 1 (small text)
  const byte  lineSpace2 =          8;       // line spacing for textsize 2 (large text)
  const int   displayMaxLines =     3;       // max lines that can be displayed in lower section of display in textsize1 (5 on larger oLeds)
  const int   MaxmenuTitleLength =  10;      // max characters per line when using text size 2 (usually 10)
  bool        no_dmx_flag =         false;   // Flag for the screen timeout function
  uint32_t    lcd_blink_timestamp = 0;
  bool        toggle_title =        true;

  volatile uint8_t buffer[DMXINPUT_BUFFER_SIZE(1, NUM_CHANNELS)];  // CHANGE TO START_CHANNEL

//Memory related variables 
  int mem_buf[FLASH_PAGE_SIZE/sizeof(int)];  // One page buffer of ints
  int *p, mem_addr;
  unsigned int page;                         // prevent comparison of unsigned and signed int
  int first_empty_page = -1;

// forward functions declarations
  void doEncoder();
  void mainMenu();
  void menuActions();
  void addr_changer();
  void menuValues();
  void reUpdateButton();
  void serviceMenu();
  int  serviceValue(bool _blocking);
  void createList(String _title, int _noOfElements, String *_list);
  void displayMessage(String _title, String _message);
  void resetMenu();
  void handle_dmx_message();
  void initialize_servos();
  void update_servo();
  void idleDisplay();
  void menuValues();
  void addr_changer();
  void menuActions();
  void idleMenu();
  void mainMenu();
  void defaultMenu();


  // modes that the menu system can be in
  enum menuModes {
      off,                                  // display is off
      menu,                                 // a menu is active
      value,                                // 'enter a value' none blocking is active
      message,                              // displaying a message
      blocking,                             // a blocking procedure is in progress (see enter value)
      idle
  };
  menuModes menuMode = off;                 // default mode at startup is off

  struct oledMenus {
    String menuTitle = "";                    // the title of active mode
    int noOfmenuItems = 0;                    // number if menu items in the active menu
    int selectedMenuItem = 0;                 // when a menu item is selected it is flagged here until actioned and cleared
    int highlightedMenuItem = 0;              // which item is curently highlighted in the menu
    String menuItems[maxmenuItems+1];         // store for the menu item titles
    uint32_t lastMenuActivity = 0;            // time the menu last saw any activity (used for timeout)
    // 'enter a value'
    int mValueEntered = 0;                    // store for number entered by value entry menu
    int mValueLow = 0;                        // lowest allowed value
    int mValueHigh = 0;                       // highest allowed value
    int mValueStep = 0;                       // step size when encoder is turned
  };
  oledMenus oledMenu;

  struct rotaryEncoders {
    volatile int encoder0Pos = 0;             // current value selected with rotary encoder (updated by interrupt routine)
    volatile bool encoderPrevA;               // used to debounced rotary encoder
    volatile bool encoderPrevB;               // used to debounced rotary encoder
    uint32_t reLastButtonChange = 0;          // last time state of button changed (for debouncing)
    bool encoderPrevButton = 0;               // used to debounce button
    int reButtonDebounced = 0;                // debounced current button state (1 when pressed)
    const bool reButtonPressedState = BUTTONPRESSEDSTATE;  // the logic level when the button is pressed
    const uint32_t reDebounceDelay = DEBOUNCEDELAY;        // button debounce delay setting
    bool reButtonPressed = 0;                 // flag set when the button is pressed (it has to be manually reset)
  };
  rotaryEncoders rotaryEncoder;

  struct Servo {
    int dmx_channel;
    int pwm_channel;
    int dmx_value;
    uint16_t min_pos; // -----------------------changed this one to uint16_t to fit in one single flash page mem (256bytes)
    uint16_t max_pos;
    int last_update;
  };
struct Servo servos[NUM_CHANNELS];

// oled SSD1306 display connected to I2C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// PCA9685 I2C pwm module
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_ADDR, Wire);


// ----------------------------------------------------------------
//                              -setup
// ----------------------------------------------------------------
void setup() {

    #ifdef DEBUG
      Serial.begin(115200); //while (!Serial); delay(50);       // start serial comms non-blocking
      Serial.println("\nStarting menu\n");
    #endif

    pinMode(LED_BUILTIN, OUTPUT);     // onboard indicator led
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);

  // configure gpio pins for rotary encoder
    pinMode(encoder0Press,  INPUT_PULLUP);
    pinMode(encoder0PinA,   INPUT);
    pinMode(encoder0PinB,   INPUT);

  // initialise the oled display

    Wire.begin();
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true)) {
      #ifdef DEBUG
        Serial.println(("\nError initialising the oled display"));
      #endif 
    }
    Wire.setClock(100000);

  // Interrupt for reading the rotary encoder position
    rotaryEncoder.encoder0Pos = 0;
    attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE);

  //Starting the DMX interpreter
    dmxInput.begin(DMX_INPUT_PIN, 1, NUM_CHANNELS); // CHANGE TO START_CHANNEL
    dmxInput.read_async(buffer);

    initialize_servos();    //------------------------------------------------NEED TO CHANGE THIS ONE TO FLASH STORED VALUES
  
  // Starting the PWM Driver
    pwm.begin();
    pwm.setPWMFreq(50);
    

  // display greeting message - pressing button will start menu
    displayMessage("DELED", "DMX Servo "+SOFTWARE_VERSION+"\nController");
    display.display();
    delay(2000);
    idleMenu();
}


// ----------------------------------------------------------------
//                              -loop
// ----------------------------------------------------------------
void loop() {
  handle_dmx_message();
  reUpdateButton();      // update rotary encoder button status (if pressed activate default menu)
  menuUpdate();          // update or action the oled menu
} 


// Start the default menu
void defaultMenu() {
  idleMenu();
}

//--------------------------------------------------------------------------------------------------


// a demonstration of how to create a menu
// when an item is selected it is actioned in menuActions()
void mainMenu() {
  resetMenu();                            // clear any previous menu
  menuMode = menu;                        // enable menu mode
  oledMenu.noOfmenuItems = 6;             // set the number of items in this menu
  oledMenu.menuTitle = "MENU";            // menus title (used to identify it)

  oledMenu.menuItems[1] = "DMX Address";
  oledMenu.menuItems[2] = "Servo Limits";
  oledMenu.menuItems[3] = "Version";
  oledMenu.menuItems[4] = "Reset All";
  oledMenu.menuItems[5] = "Display Off";
  oledMenu.menuItems[6] = "Back";
} 

//--------------------------------------------------------------------------------------------------           // BUILD SERVO LIMITS INTERACTION
// Actions for menu selections are put in here

void menuActions() {
  //podia ser uma declaracao tipo "CASE"
  if (oledMenu.menuTitle == "MENU") {    // actions when an item is selected in menu
    // ADDRESS - 'enter a value' (none blocking) 
    if (oledMenu.selectedMenuItem == 1) {
      #ifdef DEBUG
        Serial.println("menu: enter addr value");
      #endif
      resetMenu();
      addr_changer();       // enter a value
    }
    // LIMITS create a menu from a list 
    else if (oledMenu.selectedMenuItem == 2) {
      #ifdef DEBUG
        Serial.println("menu: LIMITS");
      #endif
      String tList[]={"servo 1 - min", "servo 1 - max","servo 2 - min", "servo 2 - max","servo 3 - min", "servo 3 - max","servo 4 - min", "servo 4 - max","Back"};
      createList("Servo Limits", 9, &tList[0]);
    }
    // Info -- VERSION message
    else if (oledMenu.selectedMenuItem == 3) {
      #ifdef DEBUG
        Serial.println("menu: Version message");
      #endif
      displayMessage("Version v"+ SOFTWARE_VERSION, "SERVO DMX CONTROLLER\nDELED + AMIEIRO");    // 21 chars per line, "\n" = next line
    }
    // RESET ALL
    else if (oledMenu.selectedMenuItem == 4) {
      #ifdef DEBUG
        Serial.println("menu: RESET ALL");
      #endif
      //-------------------------------------------------------RESET ALL FUNC
    }
    // turn menu/oLED off            --  SCREEN OFF
    else if (oledMenu.selectedMenuItem == 5) {
      #ifdef DEBUG
        Serial.println("menu: menu off");
      #endif
      resetMenu();    // turn menus off
    }
    // Back to idleMenu
    else if (oledMenu.selectedMenuItem == 6) {
      #ifdef DEBUG
        Serial.println("menu: Back");
      #endif
      idleMenu();    // turn menus off
    }
    oledMenu.selectedMenuItem = 0;                // clear menu item selected flag as it has been actioned
  }
  
  // actions when an item is selected in the demo_list menu
  if (oledMenu.menuTitle == "Servo Limits") {
    if (oledMenu.selectedMenuItem == 9) {
      #ifdef DEBUG
        Serial.println("Servo Limits: Back to main menu");
      #endif
      mainMenu();
    }
    oledMenu.selectedMenuItem = 0;                // clear menu item selected flag as it has been actioned
  }

  if (oledMenu.menuTitle == "SERVO DMX") {
    //PRECISA DE CODIGO AQUI ?
  }
}

//--------------------------------------------------------------------------------------------------

//    when an item is selected it is actioned in menuActions()
void idleMenu() {
  resetMenu();                            // clear any previous menu
  menuMode = idle;                        // enable menu mode
  oledMenu.menuTitle = "SERVO DMX";       // menus title (used to identify it)
} 

//-------------------------------------------------------------------------------------------------- // DEVELOP DMX CHANGE START ADDRESS 

void addr_changer() {
  resetMenu();                           // clear any previous menu
  menuMode = value;                      // enable value entry
  oledMenu.menuTitle = "DMX Address";    // title (used to identify which number was entered)
  oledMenu.mValueLow = 0;                // minimum value allowed
  oledMenu.mValueHigh = 512;             // maximum value allowed
  oledMenu.mValueStep = 1;               // step size
  oledMenu.mValueEntered = 50;           // starting value                  // VARIAVEL GLOBAL DE ENDEREÇO
}

// ----------------------------------------------------------------
//                   -button debounce (rotary encoder)
//             update rotary encoder current button status
// ----------------------------------------------------------------
void reUpdateButton() {
    bool tReading = digitalRead(encoder0Press);        // read current button state
    if (tReading != rotaryEncoder.encoderPrevButton) rotaryEncoder.reLastButtonChange = millis();     // if it has changed reset timer
    if ( (unsigned long)(millis() - rotaryEncoder.reLastButtonChange) > rotaryEncoder.reDebounceDelay ) {  // if button state is stable
      if (rotaryEncoder.encoderPrevButton == rotaryEncoder.reButtonPressedState) {
        if (rotaryEncoder.reButtonDebounced == 0) {    // if the button has been pressed
          rotaryEncoder.reButtonPressed = 1;           // flag set when the button has been pressed
          if (menuMode == off) defaultMenu();          // if the display is off start the default menu
        }
        rotaryEncoder.reButtonDebounced = 1;           // debounced button status  (1 when pressed)
      } else {
        rotaryEncoder.reButtonDebounced = 0;
      }
    }
    rotaryEncoder.encoderPrevButton = tReading;            // update last state read
}

// ----------------------------------------------------------------
//                    -update the active menu
// ----------------------------------------------------------------
void menuUpdate() {

  if(no_dmx_flag == false){
    if (menuMode == off) return;    // if menu system is turned off do nothing more

    // if no recent activity then turn oled off
    if ( (unsigned long)(millis() - oledMenu.lastMenuActivity) > (menuTimeout * 1000)) {
      resetMenu();
      return;
    }
  }
  
    switch (menuMode) {
      // if there is an active menu
      case menu:
        serviceMenu();
        menuActions();
        break;

      // if there is an active none blocking 'enter value'
      case value:
        serviceValue(0);            //NEW ADDRESS DMX CHANNEL -> SERVO STRUCT
        if (rotaryEncoder.reButtonPressed) {                        // if the button has been pressed
          menuValues();                                             // a value has been entered so action it
          break;
        }

      // if a message is being displayed
      case message:
        if (rotaryEncoder.reButtonPressed == 1) mainMenu();    // if button has been pressed return to default menu
        break;

      // if DMX comm is lost while display is off (menuMode == off)
      case off:
        resetMenu();
        break;

      case idle:
        idleDisplay();
        if (rotaryEncoder.reButtonPressed == 1) mainMenu();    // if button has been pressed return to default menu
        break;
    }
}

//--------------------------------------------------------------------------------------------------

// actions for value entered put in here
void menuValues() {
  // action for "DMX Address"
  if (oledMenu.menuTitle == "DMX Address") {
    String tString = String(oledMenu.mValueEntered);
    #ifdef DEBUG
      Serial.println("DMX Address: The value entered was " + tString);
    #endif 
    mainMenu();
    // alternatively use 'resetMenu()' here to turn menus off after value entered - or use 'defaultMenu()' to re-start the default menu
  }
}
// ----------------------------------------------------------------
//                       -service active menu
// ----------------------------------------------------------------
void serviceMenu() {

    // rotary encoder
      if (rotaryEncoder.encoder0Pos >= itemTrigger) {
        rotaryEncoder.encoder0Pos -= itemTrigger;
        oledMenu.highlightedMenuItem++;
        oledMenu.lastMenuActivity = millis();   // log time
      }
      if (rotaryEncoder.encoder0Pos <= -itemTrigger) {
        rotaryEncoder.encoder0Pos += itemTrigger;
        oledMenu.highlightedMenuItem--;
        oledMenu.lastMenuActivity = millis();   // log time
      }
      if (rotaryEncoder.reButtonPressed == 1) {
        oledMenu.selectedMenuItem = oledMenu.highlightedMenuItem;     // flag that the item has been selected
        oledMenu.lastMenuActivity = millis();   // log time
        #ifdef DEBUG
          Serial.println("menu '" + oledMenu.menuTitle + "' item '" + oledMenu.menuItems[oledMenu.highlightedMenuItem] + "' selected");
        #endif
        
      }

    const int _centreLine = displayMaxLines / 2 + 1;    // mid list point
    display.clearDisplay();
    display.setTextColor(WHITE);

    // verify valid highlighted item
      if (oledMenu.highlightedMenuItem > oledMenu.noOfmenuItems) oledMenu.highlightedMenuItem = oledMenu.noOfmenuItems;
      if (oledMenu.highlightedMenuItem < 1) oledMenu.highlightedMenuItem = 1;

    // title
      display.setCursor(0, 0);
      if (menuLargeText) {
        display.setTextSize(2);
        display.println(oledMenu.menuItems[oledMenu.highlightedMenuItem].substring(0, MaxmenuTitleLength));
      } else {
        if (oledMenu.menuTitle.length() > MaxmenuTitleLength) display.setTextSize(1);
        else display.setTextSize(1);
        display.println(oledMenu.menuTitle);
      }
      display.drawLine(0, topLine-1, display.width(), topLine-1, WHITE);       // draw horizontal line under title

    // menu
      display.setTextSize(1);
      display.setCursor(0, topLine);
      for (int i=1; i <= displayMaxLines; i++) {
        int item = oledMenu.highlightedMenuItem - _centreLine + i;
        if (item == oledMenu.highlightedMenuItem) display.setTextColor(BLACK, WHITE);
        else display.setTextColor(WHITE);
        if (item > 0 && item <= oledMenu.noOfmenuItems) display.println(oledMenu.menuItems[item]);
        else display.println(" ");
      }

    //// how to display some updating Info. on the menu screen
    // display.setCursor(80, 25);
    // display.println(millis());
 
    display.display();
}

// ---------------------------------------------------------------------------------       //ADICIONAR ACELERAÇÃO NO CONTROLE DO DMX ADDR 
//                        -service value entry
// if _blocking set to 1 then all other tasks are stopped until a value is entered
// ---------------------------------------------------------------------------------

int serviceValue(bool _blocking) {

  const int _valueSpacingX = 35;      // spacing for the displayed value x position
  const int _valueSpacingY = 1;       // spacing for the displayed value y position

  if (_blocking) {
    menuMode = blocking;
    oledMenu.lastMenuActivity = millis();   // log time of last activity (for timeout)
  }
  uint32_t tTime;

  do {
  // rotary encoder
    if (rotaryEncoder.encoder0Pos >= itemTrigger) {
      rotaryEncoder.encoder0Pos   -= itemTrigger;
      oledMenu.mValueEntered      -= oledMenu.mValueStep;
      oledMenu.lastMenuActivity   =  millis();   // log time
    }
    if (rotaryEncoder.encoder0Pos <= -itemTrigger) {
      rotaryEncoder.encoder0Pos   += itemTrigger;
      oledMenu.mValueEntered      += oledMenu.mValueStep;
      oledMenu.lastMenuActivity   =  millis();   // log time
    }
    if (oledMenu.mValueEntered    < oledMenu.mValueLow) {
      oledMenu.mValueEntered      = oledMenu.mValueLow;
      oledMenu.lastMenuActivity   = millis();   // log time
    }
    if (oledMenu.mValueEntered    > oledMenu.mValueHigh) {
      oledMenu.mValueEntered      = oledMenu.mValueHigh;
      oledMenu.lastMenuActivity   = millis();   // log time
    }

    display.clearDisplay();
    display.setTextColor(WHITE);

    // title
    display.setCursor(0, 0);
    if (oledMenu.menuTitle.length() > MaxmenuTitleLength) display.setTextSize(1);
    else display.setTextSize(1);
    display.println(oledMenu.menuTitle);
    display.drawLine(0, topLine-1, display.width(), topLine-1, WHITE);       // draw horizontal line under title

    // value selected
    display.setCursor(_valueSpacingX, topLine + _valueSpacingY);
    display.setTextSize(3);
    if(oledMenu.mValueEntered < 100) display.print(0);
    if(oledMenu.mValueEntered < 10) display.print(00); //--------------------------------nao precisava de um ELSE aqui nao ? entretanto ta funcoinando assim
    display.println(oledMenu.mValueEntered);

    /*// range
    display.setCursor(0, display.height() - lineSpace1 - 1 );   // bottom of display
    display.setTextSize(1);
    display.println(String(oledMenu.mValueLow) + " to " + String(oledMenu.mValueHigh));
    */

    /*// bar
    int Tlinelength = map(oledMenu.mValueEntered, oledMenu.mValueLow, oledMenu.mValueHigh, 0 , display.width());
    display.drawLine(0, display.height()-1, Tlinelength, display.height()-1, WHITE);
    */

    display.display();

    reUpdateButton();        // check status of button
    tTime = (unsigned long)(millis() - oledMenu.lastMenuActivity);      // time since last activity

  } while (_blocking && rotaryEncoder.reButtonPressed == 0 && tTime < (menuTimeout * 1000));        // if in blocking mode repeat until button is pressed or timeout

  if (_blocking) menuMode = off;

  return oledMenu.mValueEntered;        // used when in blocking mode

}

// ----------------------------------------------------------------
//                       -create list for menu
// ----------------------------------------------------------------
void createList(String _title, int _noOfElements, String *_list) {
  resetMenu();                      // clear any previous menu
  menuMode = menu;                  // enable menu mode
  oledMenu.noOfmenuItems = _noOfElements;    // set the number of items in this menu
  oledMenu.menuTitle = _title;               // menus title (used to identify it)

  for (int i=1; i <= _noOfElements; i++) {
    oledMenu.menuItems[i] = _list[i-1];        // set the menu items
  }
}

// ----------------------------------------------------------------
//                         -message display
//             21 characters per line, use "\n" for next line
// ----------------------------------------------------------------
 void displayMessage(String _title, String _message) {
  resetMenu();
  menuMode = message;

  display.clearDisplay();
  display.setTextColor(WHITE);

  // title
    display.setCursor(0, 0);
    if (menuLargeText) {
      display.setTextSize(2);
      display.println(_title.substring(0, MaxmenuTitleLength));
    } else {
      if (_title.length() > MaxmenuTitleLength) display.setTextSize(1);
      else display.setTextSize(1);
      display.println(_title);
    }

  // message
    display.setCursor(0, topLine);
    display.setTextSize(1);
    display.println(_message);

  display.display();

 }


// ----------------------------------------------------------------
//                         -idle display
// ----------------------------------------------------------------             // ADD blinking screen when no dmx signal (no_dmx_flag == true)
void idleDisplay(){
 
  menuMode = idle;

  char channelBuffer[4];
  sprintf(channelBuffer, "%03d", START_CHANNEL);
  String originalTitle = oledMenu.menuTitle + " " + String(channelBuffer);
  String t_title;

  display.clearDisplay();
  display.setTextColor(WHITE);

  // title
    display.setCursor(0, 0);
    if (oledMenu.menuTitle.length() > MaxmenuTitleLength) display.setTextSize(1);
    else display.setTextSize(1);

    if(no_dmx_flag){
      if(millis() - lcd_blink_timestamp >= 350){ 
        lcd_blink_timestamp = millis();
        toggle_title = !toggle_title;  // Toggle the flag
      }
      if(toggle_title){
          t_title = originalTitle;
        } 
        else {
          t_title = "";
        }
    }
    else t_title = originalTitle;

    display.println(t_title);
    display.drawLine(0, topLine-1, display.width(), topLine-1, WHITE);       // draw horizontal line under title


  //Servos in a grid - bar graph controlled by dmx_value
    display.setTextSize(1);
    display.setCursor(0, topLine+5);
    display.println("S1");
    int t_rectLength_1 = map(servos[0].dmx_value, 0, 255, 2, 46);
    display.fillRect(14, topLine+5, t_rectLength_1, 7, WHITE);

    display.setCursor(0, topLine+14);
    display.println("S2");
    int t_rectLength_2 = map(servos[1].dmx_value, 0, 255, 2, 46);
    display.fillRect(14, topLine+14, t_rectLength_2, 7, WHITE);

    display.setCursor(64, topLine+5);
    display.println("S3");
    int t_rectLength_3 = map(servos[2].dmx_value, 0, 255, 2, 46);
    display.fillRect(78, topLine+5, t_rectLength_3, 7, WHITE);

    display.setCursor(64, topLine+14);
    display.println("S4");
    int t_rectLength_4 = map(servos[3].dmx_value, 0, 255, 2, 46);
    display.fillRect(78, topLine+14, t_rectLength_4, 7, WHITE);

    display.display();
}


// ----------------------------------------------------------------
//                        -reset menu system
// ----------------------------------------------------------------
void resetMenu() {
  // reset all menu variables / flags
    menuMode = off;
    oledMenu.selectedMenuItem = 0;
    rotaryEncoder.encoder0Pos = 0;
    oledMenu.noOfmenuItems = 0;
    oledMenu.menuTitle = "";
    oledMenu.highlightedMenuItem = 0;
    oledMenu.mValueEntered = 0;
    rotaryEncoder.reButtonPressed = 0;

  oledMenu.lastMenuActivity = millis();   // log time

  // clear oled display
    display.clearDisplay();
    display.display();
}


// ----------------------------------------------------------------
//                     -Handle DMX received inputs
// ----------------------------------------------------------------
void handle_dmx_message() {
    if(millis() > 200+dmxInput.latest_packet_timestamp()) {
        #ifdef DEBUG
        #ifdef PRINT_DMX_MESSAGES
            Serial.println("no data!");
        #endif
        #endif
        digitalWrite(LED_BUILTIN, HIGH);
        no_dmx_flag = true;
        return;
    }
    else{
        for (uint i = 1; i < NUM_CHANNELS+1; i++){
            struct Servo *current_servo = &servos[i-1];
            update_servo(current_servo, buffer[i]);
        }
        if(millis() > 15+dmxInput.latest_packet_timestamp()){   // this line delays the led blinking to a human readable frequency
            ledState = (ledState == HIGH) ? LOW: HIGH; //toggle
        }
        no_dmx_flag = false;
    }
    digitalWrite(LED_BUILTIN, ledState);
}


// ----------------------------------------------------------------
//                     -Handle Servo data
// ----------------------------------------------------------------
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


// -------------------------------------------------------------------------
//                     -interrupt for rotary encoder
// rotary encoder interrupt routine to update position counter when turned
// interrupt Info: https://www.gammon.com.au/forum/bbshowpost.php?id=11488
// -------------------------------------------------------------------------
void doEncoder() {

  bool pinA = digitalRead(encoder0PinA);
  bool pinB = digitalRead(encoder0PinB);

  if ( (rotaryEncoder.encoderPrevA == pinA && rotaryEncoder.encoderPrevB == pinB) ) return;  // no change since last time (i.e. reject bounce)

  // same direction (alternating between 0,1 and 1,0 in one direction or 1,1 and 0,0 in the other direction)
         if (rotaryEncoder.encoderPrevA == 1 && rotaryEncoder.encoderPrevB == 0 && pinA == 0 && pinB == 1) rotaryEncoder.encoder0Pos -= 1;
    else if (rotaryEncoder.encoderPrevA == 0 && rotaryEncoder.encoderPrevB == 1 && pinA == 1 && pinB == 0) rotaryEncoder.encoder0Pos -= 1;
    else if (rotaryEncoder.encoderPrevA == 0 && rotaryEncoder.encoderPrevB == 0 && pinA == 1 && pinB == 1) rotaryEncoder.encoder0Pos += 1;
    else if (rotaryEncoder.encoderPrevA == 1 && rotaryEncoder.encoderPrevB == 1 && pinA == 0 && pinB == 0) rotaryEncoder.encoder0Pos += 1;

  // change of direction
    else if (rotaryEncoder.encoderPrevA == 1 && rotaryEncoder.encoderPrevB == 0 && pinA == 0 && pinB == 0) rotaryEncoder.encoder0Pos += 1;
    else if (rotaryEncoder.encoderPrevA == 0 && rotaryEncoder.encoderPrevB == 1 && pinA == 1 && pinB == 1) rotaryEncoder.encoder0Pos += 1;
    else if (rotaryEncoder.encoderPrevA == 0 && rotaryEncoder.encoderPrevB == 0 && pinA == 1 && pinB == 0) rotaryEncoder.encoder0Pos -= 1;
    else if (rotaryEncoder.encoderPrevA == 1 && rotaryEncoder.encoderPrevB == 1 && pinA == 0 && pinB == 1) rotaryEncoder.encoder0Pos -= 1;
  /*
    #ifdef DEBUG
      Serial.println("Error: invalid rotary encoder pin state - prev=" + String(rotaryEncoder.encoderPrevA) + ","+ String(rotaryEncoder.encoderPrevB) + " new=" + String(pinA) + "," + String(pinB));
    #endif
  */

  // update previous readings
    rotaryEncoder.encoderPrevA = pinA;
    rotaryEncoder.encoderPrevB = pinB;
}


// ---------------------------------------------- end ----------------------------------------------