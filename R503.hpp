#ifndef R503_HPP
#define R503_HPP

#define DEBUG_R503

#include <stdint.h>
#include "SoftwareSerial.h"

/*
 * Confirmation Codes
 */
// ESP8266 side confirmation codes
#define SUCCESS 0
#define ERROR_ADDRESS_MISMATCH (-2)
#define ERROR_CHECKSUM_MISMATCH (-3)
#define ERROR_TIMEOUT (-4)
#define ERROR_PID_MISMATCH (-5)
#define ERROR_NOT_ENOUGH_MEMORY (-6)
// R503 side confirmation codes
#define COMMAND_EXECUTION_COMPLETE 0x00
#define ERROR_RECEIVING_PACKAGE 0x01
#define WRONG_PASSWORD 0x13


// package ID
#define PID_COMMAND 0x01
#define PID_DATA 0x02
#define PID_ACKNOWLEDGE 0x07
#define PID_END 0x08

//aura control code
#define AURA_BREATH 0x01
#define AURA_FLASH 0x02
#define AURA_ON 0x03
#define AURA_OFF 0x04
#define AURA_GRADUAL_ON 0x05
#define AURA_GRADUAL_OFF 0x06

//aura color index
#define AURA_RED 0x01
#define AURA_BLUE 0x02
#define AURA_PURPLE 0x03


#define RECEIVE_TIMEOUT 1000

struct Package {
    uint8_t id;
    uint16_t length; // length (exclusive checksum)
    uint8_t *data;
    uint16_t checksum;
    Package(uint8_t id, uint16_t length, uint8_t *data);
    Package(uint16_t length, uint8_t *data);
    void calculateChecksum();
    bool checksumMatches();
};

struct ProductInfo {
    char module_type[16];
    char module_batch_number[4];
    char module_serial_number[8];
    uint8_t hardware_version[2];
    char sensor_type[8];
    uint16_t sensor_width;
    uint16_t sensor_height;
    uint16_t template_size;
    uint16_t database_size;
};

class R503 {
    // at most one R503 can be used
    static R503 *instance;
    static void fallingISR();
    static void risingISR();
    
    uint32_t address;
    uint32_t password;
    long baudrate;
    
    int rxPin, txPin, touchPin;
    SoftwareSerial *serial;
    
public:
    R503(int rxPin, int txPin, int touchPin, uint32_t address = 0xFFFFFFFF, uint32_t password = 0x0, long baudrate = 57600);
    virtual ~R503();
    
    void begin();
    void sendPackage(Package const &package);
    
    /* 
     * reads the next package marked with 0xEF01
     * @package.length: size of memory block data points to
     * @package.data: pointer to memory block to write received data
     * @return: a ESP8266 confirmation code
     */
    int receivePackage(Package &package);
    
    /*
     * receives a acknowledge Package
     * @data: pointer to memory to write data to
     * @length: length of data memory block, at least 1
     * @return: the R503 confirmation code stored in data[0] or a ESP8266 confirmation code on error
     */
    int receiveAcknowledge(uint8_t *data, uint8_t length);
    
    void onFingerDown();
    void onFingerUp();
    
    int verifyPassword();
    int readProductInfo(ProductInfo &info);
    int auraControl(uint8_t control, uint8_t speed, uint8_t color, uint8_t times);
    
    #ifdef DEBUG_R503
    void printProductInfo();
    #endif
    
    bool isTouched();
    
};

#endif
