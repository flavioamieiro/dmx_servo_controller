extern "C" {
  #include <hardware/sync.h>
  #include <hardware/flash.h>
};

#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

uint16_t flash_buf_temp[9];  // This is where we'll store the values in memory
uint16_t flash_buf[FLASH_PAGE_SIZE/sizeof(uint16_t)];  // One page buffer of uint16_t

#define DEBUG  // Define the DEBUG flag

int find_free_memory_page() {
    int first_empty_page = -1;
    int mem_addr;
    unsigned int page;
    for(page = 0; page < FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE; page++){
        mem_addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
        uint16_t *p = (uint16_t *)mem_addr;
        if( *p == 0xFFFF && first_empty_page < 0){
            first_empty_page = page;
            break;
        }
    }
    #ifdef DEBUG
    Serial.println("[DEBUG] First empty page found at: " + String(first_empty_page));
    #endif
    return first_empty_page;
}

void writeValuesToFlash() {
    int first_empty_page = find_free_memory_page();
    if (first_empty_page < 0){
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
        first_empty_page = 0;
        restore_interrupts (ints);
        #ifdef DEBUG
        Serial.println("[DEBUG] Erased flash sector.");
        #endif
    }
    // Copy values to flash buffer
    for(int i = 0; i < 9; i++) {
        flash_buf[i] = flash_buf_temp[i];
    }
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)flash_buf, FLASH_PAGE_SIZE);
    restore_interrupts (ints);
    #ifdef DEBUG
    Serial.println("[DEBUG] Values stored to flash.");
    #endif
}

void readValuesFromFlash() {
    int last_valid_page = -1;
    int mem_addr;
    for(unsigned int page = (FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE) - 1; page >= 0; page--){
        mem_addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
        uint16_t *p = (uint16_t *)mem_addr;
        if( *p != 0xFFFF){
            last_valid_page = page;
            break;
        }
    }
    if(last_valid_page >= 0) {
        uint16_t *p = (uint16_t *)(XIP_BASE + FLASH_TARGET_OFFSET + (last_valid_page * FLASH_PAGE_SIZE));
        for(int i = 0; i < 9; i++) {
            flash_buf_temp[i] = p[i];
        }
        #ifdef DEBUG
        Serial.println("[DEBUG] Values read from flash at startup.");
        #endif
    }
}

void displayValuesFromMemory() {
    Serial.println("[DEBUG] Values from memory:");
    for(int i = 0; i < 9; i++) {
        Serial.println(flash_buf_temp[i]);
    }
    Serial.println("[DEBUG]");
}

void setup() {
    Serial.begin(9600);
    while(!Serial);

    // Read values from flash at startup
    readValuesFromFlash();

    #ifdef DEBUG
    Serial.println("[DEBUG] Finished reading from flash.");
    #endif

    // Display the read values on the serial monitor
    Serial.println("Values read from flash memory:");
    for(int i = 0; i < 9; i++) {
        Serial.println(flash_buf_temp[i]);
    }

    // Prompt the user to enter 9 numbers
    Serial.println("Please enter 9 numbers, one at a time:");
}

void loop() {
    static int inputCount = 0;

    if(Serial.available()) {
        String inputStr = Serial.readStringUntil('\n');
        inputStr.trim();  // Remove any leading or trailing whitespace

        if(inputStr == "R" || inputStr == "r" || inputStr == "read") {
            displayValuesFromMemory();
            Serial.println("Please continue entering numbers or type 'R', 'r', or 'read' to read from memory:");
            return;
        }

        uint16_t inputValue = inputStr.toInt();

        #ifdef DEBUG
        Serial.println("[DEBUG] Received input: " + inputStr);
        #endif

        if(inputValue >= 0 && inputValue <= 65535) {  // Check if the input is a valid uint16_t
            flash_buf_temp[inputCount] = inputValue;
            inputCount++;

            if(inputCount == 9) {
                // All 9 numbers have been entered
                writeValuesToFlash();

                Serial.println("Values saved to flash memory!");
                inputCount = 0;  // Reset the count for the next set of inputs
                Serial.println("Please enter 9 numbers, one at a time, or type 'read' to read from memory:");
            }
        } else {
            Serial.println("Invalid input. Please enter a number between 0 and 65535 or type 'read' to read from memory.");
        }
    }
}





// SERVO
// Minimum and maximum pwm values to set as -90/90 degrees.
// We found by testing in the servos we had that the following
// min/max values result in a 180 rotation range.
  #define DEFAULT_SERVO_MIN       64
  #define DEFAULT_SERVO_MAX       512
  #define DEFAULT_SERVO_DMX_VALUE 127
  #define SERVO_ABSOLUTE_MIN      0
  #define SERVO_ABSOLUTE_MAX      700