#ifndef R503_HPP
#define R503_HPP

//#define R503_DEBUG

#include <stdint.h>
#include "SoftwareSerial.h"

/*
 * Confirmation Codes
 */
// on R503 side corresponds to command execution complete
#define R503_SUCCESS 0 
// ESP8266 side confirmation codes
#define R503_ADDRESS_MISMATCH (-2)
#define R503_CHECKSUM_MISMATCH (-3)
#define R503_TIMEOUT (-4)
#define R503_PID_MISMATCH (-5)
#define R503_NOT_ENOUGH_MEMORY (-6)
// R503 side confirmation codes
#define R503_ERROR_RECEIVING_PACKAGE 0x01
#define R503_WRONG_PASSWORD 0x13
#define R503_NO_FINGER 0x02
#define R503_ERROR_TAKING_IMAGE 0x03
#define R503_IMAGE_MESSY 0x06
#define R503_FEATURE_FAIL 0x07
#define R503_NO_IMAGE 0x15
#define R503_BAD_LOCATION 0x0B
#define R503_ERROR_WRITING_FLASH 0x18


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

// timeout in ms for receiving a package
#define R503_RECEIVE_TIMEOUT 1000

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

struct SystemParameter {
    uint16_t status_register;
    uint16_t system_identifier_code;
    uint16_t finger_library_size;
    uint16_t security_level;
    uint32_t device_address;
    uint16_t data_packet_size;
    uint32_t baudrate;
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
    
    void onFingerDown();
    void onFingerUp();
    
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
    
    /*
     * you must call verifyPassword() after begin() and before any other command
     */
    int verifyPassword();
    int readProductInfo(ProductInfo &info);
    int auraControl(uint8_t control, uint8_t speed, uint8_t color, uint8_t times);
    int readSystemParameter(SystemParameter &param);
    // @info: pointer to memory block of at least 512 byte
    int readInformationPage(char *info);
    
    /* 
     * the R503 has
     *   - 1 image buffer
     *   - 6 feature buffers (refered in the datasheet as character buffers)
     *
     * to enroll a finger
     *   - get finger image
     *   - extract features
     *   - fuse 2 to 6 feature buffer into a model
     *
     * to match fingerprint
     */
    
    
    int readModelCount(uint16_t &count);
    
    /*
     * detects finger on the sensor, takes image and writes it into the image buffer
     * @return: R503_NOFINGER if no finger is on the sensor
     */
    int takeImage();
    
    /*
     * extracts the features from the finger image in the image buffer
     * and writes them into the feature buffer indentified by @featureBuffer
     * @featureBuffer: number in the range 1 to 6
     */
    int extractFeatures(uint8_t featureBuffer);
    
    /*
     * combines the features from featureBuffer 1 and featureBuffer 2 into one model,
     * which is written back into featureBuffer 1 and featureBuffer 2
     * TODO check if create model only uses the first 2 feature Buffers or more if more are available by registering different fingers
     */
    int createModel();
    
    /*
     * TODO test if @featureBuffer and featureBuffer+1 are stored
     */
    int storeModel(uint8_t featureBuffer, uint16_t location);
    
    int searchFinger(uint8_t featureBuffer, uint16_t &location, uint16_t &score);
    
    #ifdef R503_DEBUG
    int printProductInfo();
    int printSystemParameter();
    #endif
    
    bool isTouched();
    
};

#endif
