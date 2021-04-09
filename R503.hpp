#ifndef R503_HPP
#define R503_HPP

/*
 * library for ESP8266 (Arduino SDK) to interface with GROW R5XX fingerprint sensor
 * datasheet for R503: https://cdn-shop.adafruit.com/product-files/4651/4651_R503+fingerprint+module+user+manual.pdf
 * this library was inspired by the adafruit fingerprint sensor library: https://github.com/adafruit/Adafruit-Fingerprint-Sensor-Library
 * tested with ESP12 and R503
 */

/*
 * enable different debug levels by setting certain bits
 * debug messages are print via Serial.print
 * DEBUG not defined or 0: no debug messages
 * bit 0 set: print sent and received packets as hex bytes
 * bit 1 set: print error messages
 * bit 2 set: define extra functios to print information (e.g. printProductInfo(), printSystemParameter) 
 */
#define R503_DEBUG 0x04

#include <stdint.h>
#include "SoftwareSerial.h"

/*
 * Confirmation Codes
 */
// on R503 side corresponds to command execution complete
#define R503_SUCCESS 0 
// ESP8266 side confirmation codes (negative numbers)
#define R503_ADDRESS_MISMATCH (-2)
#define R503_CHECKSUM_MISMATCH (-3)
#define R503_TIMEOUT (-4)
#define R503_PID_MISMATCH (-5)
#define R503_NOT_ENOUGH_MEMORY (-6)
#define R503_SPECIFICATION_ERROR (-7)
#define R503_INVALID_BAUDRATE (-8)
#define R503_RESET_TIMEOUT (-9)
// R503 side confirmation codes (positive numbers)
#define R503_ERROR_RECEIVING_PACKAGE 0x01
#define R503_WRONG_PASSWORD 0x13
#define R503_NO_FINGER 0x02
#define R503_ERROR_TAKING_IMAGE 0x03
#define R503_IMAGE_MESSY 0x06
#define R503_FEATURE_FAIL 0x07
#define R503_NO_IMAGE 0x15
#define R503_BAD_LOCATION 0x0B
#define R503_ERROR_WRITING_FLASH 0x18
#define R503_NO_MATCH 0x08
#define R503_NO_MATCH_IN_LIBRARY 0x09
#define R503_SENSOR_ABNORMAL 0x29


// package ID
#define PID_COMMAND 0x01
#define PID_DATA 0x02
#define PID_ACKNOWLEDGE 0x07
#define PID_END 0x08

// aura control code
#define AURA_BREATH 0x01
#define AURA_FLASH 0x02
#define AURA_ON 0x03
#define AURA_OFF 0x04
#define AURA_GRADUAL_ON 0x05
#define AURA_GRADUAL_OFF 0x06

// aura color index
#define AURA_RED 0x01
#define AURA_BLUE 0x02
#define AURA_PURPLE 0x03

// timeout in ms for receiving a package
#define R503_RECEIVE_TIMEOUT 1000
// timeout in ms for handshake sign after softreset
#define R503_SOFTRESET_TIMEOUT 1000
// default time in ms to wait for incoming packages before cleaning serial buffer
#define R503_CLEAN_SERIAL_DELAY 200

/*
 * communication with R503 happens over UART through transmission of packages
 * R503 responds to command packages with acknowledge and data packages
 */
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

// structure to hold info about R503
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

// structure to hold system parameters
struct SystemParameter {
    uint16_t status_register;
    uint16_t system_identifier_code;
    uint16_t finger_library_size;
    uint16_t security_level;
    uint32_t device_address;
    uint16_t data_package_size;
    uint32_t baudrate;
};

/*
 * the R503 fingerprint sensor communicates over UART with the microcontroller
 * default baudrate is 57600 
 * since new baudrates are stored in flash you may have to try other ones if module is not in factory setting
 * 
 * you need to call the init() method after construction and before calling any other method
 *
 * error handling is done with the confirmation codes defined above
 * methods returning an integer return 
 *
 * the R503 has (in RAM)
 *   - 1 image buffer
 *   - 6 character buffer 
 *       indexed with a number from 1 to 6
 *       they are used to either store the features extracted from the image or 
 *       a finger model, a so called template
 *       a template is obtained through fusion of features from 2 to 6 character buffers
 *       and needs the space of 2 character buffers when written back
 *
 * to enroll a finger
 *   1. takeImage(): get a fingerprint image and write it into the image buffer
 *   2. extractFeatures(k): extract features from the image buffer and write them into character buffer k
 *   3. repeat 1. step and 2. step 2 to 6 times with k starting from 1
 *   4. createTemplate(): fuse the extracted features into a template, which is written back to character buffer 1 and 2
 *   5. storeTemplate(1, l): store the template from character buffer 1 and 2 at location l in the library (flash memory)
 *
 * to find fingerprint
 *   1. takeImage()
 *   2. extractFeatures(k)
 *   3. searchFinger(k, l, s): searches the fingerprint library for template mathching features in character buffer k
 *                             location of matched fingerprint is stored in reference l and matching score in reference s
 *                             if no matching fingerprint is found R503_NO_MATCH_IN_LIBRARY is returned and l and s are set to 0
 */
class R503 {
    uint32_t address;
    uint32_t password;
    long baudrate;
    
    uint16_t finger_library_size;
    uint16_t data_package_size;
    
    int rxPin, txPin, touchPin;
    SoftwareSerial *serial;
    
public:
    /*
     * @return a null terminated string containing the error message for the confirmation code @code
     */
    static char const *errorMsg(int code);

    /*
     * @rxPin: the green wire
     * @txPin: the yellow wire
     * @touchPin: the blue wire, set to -1 if not connected
     * red wire is VCC connect to 3.3V
     * black wire is GND
     * white wire connect to 3.3V
     * 
     * when sensor gets touched touchPin goes from HIGH to LOW and when released it switches back from LOW to HIGH
     * you may want to attach an interrupt serivce routine to touchPin after calling init() to handle this signal
     */
    R503(int rxPin, int txPin, int touchPin = -1, uint32_t address = 0xFFFFFFFF, uint32_t password = 0x0, long baudrate = 57600);
    virtual ~R503();
    
    /*
     * you must call init() successfully before any other method
     * initialises the serial connection
     * verifies the password
     * reads system parameters
     */
    int init();
    
    // sends @package over the serial connection
    void sendPackage(Package const &package);
    
    
    /* 
     * reads the next package marked with 0xEF01
     * @package.length: size of memory block data points to, is updated to the received amount of bytes
     * @package.data: pointer to memory block to write received data
     * @return: a ESP8266 confirmation code
     *     if @package.data is too small to write the received data R503_NOT_ENOUGH_MEMORY is returned
     */
    int receivePackage(Package &package);
    
    /*
     * receives a acknowledge Package
     * @data: pointer to memory to write received data to
     * @length: length of data memory block, at least 1, is updated to the amount of bytes written to data
     * @return: the R503 confirmation code stored in data[0] or a ESP8266 confirmation code on error
     */
    int receiveAcknowledge(uint8_t *data, uint16_t &length);
    
    /*
     * receives several data packages and the end package
     * @data: memory block to write the data to, ensure it is big enough else R503_NOT_ENOUGH_MEMORY is returned
     * @length: size of the memory block data points to, is updated to the amount of bytes written to data
     */
    int receiveData(uint8_t *data, uint16_t &length);
    
    /*
     * cleans the serial stream to restore a empty state
     *
     * flushes the serial connection
     * waits for @milliseconds
     * consumes all available bytes
     */
    void cleanSerial(unsigned long milliseconds = R503_CLEAN_SERIAL_DELAY);
    
    // verifies the password
    int verifyPassword();
    
    /*
     * sets a new baudrate
     * valid baudrates are 9600, 19200, 38400, 57600, 115200
     * the new baudrate is preserved after poweroff
     */
    int setBaudrate(long baudrate);
    
    /*
     * @level a number between 1 (inclusive) and 5 (inclusive)
     * level 1: false acceptance rate is lowest and false rejection rate is highest
     * level 5: false acceptance rate is highest and false rejection rate is lowest
     */
    int setSecurityLevel(uint8_t level);
    
    /*
     * controls the LED ring of the sensor
     * @control: AURA_ON, AURA_OFF, AURA_BREATH, AURA_FLASH, AURA_GRADUAL_ON or GRADUAL_OFF
     * @speed: effective for AURA_BREATH, AURA_FLASH, AURA_GRADUAL_ON, GRADUAL_OFF
     *     0 to 255, 0 means 5 seconds per cycle
     * @color: AURA_RED, AURA_BLUE or AURA_PURPLE
     * @times: effective for AURA_BREATH, AURA_FLASH
     *     number of cycles: 0 means infinite or 1 to 255
     */
    int auraControl(uint8_t control, uint8_t speed, uint8_t color, uint8_t times);
    
    // fills out @info structure
    int readProductInfo(ProductInfo &info);
    
    // fills out @param structure
    int readSystemParameter(SystemParameter &param);
    
    // @info: pointer to memory block of at least 512 byte
    int readInformationPage(char *info);
    
    // check if module is okay
    int handShake();
    
    // check if sensor is okay
    int checkSensor();
    
    // resets sensor and waits until it is ready
    int softReset();
    
    // writes the amount of templates stored in the library to @count
    int templateCount(uint16_t &count);
    
    /*
     * reads the index table a bitfield indicating at which location a template is stored
     * @table: a pointer to a memory block of at least 32 bytes
     * @page: the index page, a number between 0 (inclusive) and 3 (inclusive)
     *      index page 0: location 0 to 255
     *      index page 1: location 256 to 511
     *      index page 2: location 512 to 767
     *      index page 3: location 768 to 1023
     * the bit index corresponds to the location
     * bit equal 1 means template is stored at that location 
     * bit equal 0 means no template is stored at that location
     * e.g. for page 0: template at location k <=> table[k / 8] >> k % 8 & 0x01
     */
    int readIndexTable(uint8_t *table, uint8_t page = 0);
    
    /*
     * detects finger on the sensor, takes image and writes it into the image buffer
     * @return: R503_NOFINGER if no finger is on the sensor
     */
    int takeImage();
    
    /*
     * transfers content of image buffer to microcontroller
     * @image: a memory block big enough to store the content of image buffer
     * @size: size of memory block pointed to by @image
     *        the amount of bytes written to @image will be written to @size
     * you may want to determine the image size with readProductInfo()
     */
    int uploadImage(uint8_t *image, uint16_t &size);
    
    /*
     * extracts the features from the finger image in the image buffer
     * and writes them into the character buffer indentified by @characterBuffer
     * @characterBuffer: number in the range 1 to 6
     */
    int extractFeatures(uint8_t characterBuffer);
    
    /*
     * combines the features from characterBuffer 1 to k into a template
     * k is the characterBuffer where the most recent feature extraction was written to
     * the template is written back to characterBuffer 1 and 2
     *
     *
     * TODO check if create template only uses the first 2 feature buffers or more if more are available by registering different fingers
     * check how many character buffers a template needs
     */
    int createTemplate();
    
    
    /*
     * stores template located in @characterBuffer and @characterBuffer+1 to flash at @location
     * 
     * TODO test if @characterBuffer and @characterBuffer+1 are stored
     */
    int storeTemplate(uint8_t characterBuffer, uint16_t location);
    
    // deletes template located at @location
    int deleteTemplate(uint16_t location);
    
    // deletes all the templates
    int emptyLibrary();
    
    // loads template from flash in @location to @characterBuffer and @characterBuffer+1
    int loadTemplate(uint8_t characterBuffer, uint16_t location);
    
    /*
     * compares the recently extracted features with the template in character buffer 1 and 2
     * @score: reference to write the matching score
     * @return: R503_NO_MATCH if features do not match template
     *
     * TODO find out what ModelBuffer is (charaterBuffer 1 and 2?)
     */
    int matchFinger(uint16_t &score);
    
    /*
     * searches the fingerprint library for a template matching the features in @characterBuffer
     * @location: reference to write location of matching template
     * @score: reference to write matching score
     * @return: R503_NO_MATCH_IN_LIBRARY if no matching template was found
     */
    int searchFinger(uint8_t characterBuffer, uint16_t &location, uint16_t &score);
    
    /*
     * only use this method if you specified touchPin when calling the constructor
     * alternativly presence of finger on sensor can be detected with takeImage()
     */
    bool isTouched();
    
    // methods to print information about sensor with Serial.print to Serial Monitor
    #if R503_DEBUG & 0x04
    int printProductInfo();
    int printSystemParameter();
    #endif
    
};

#endif
