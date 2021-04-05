#include "R503.hpp"
#include "Arduino.h"

/* 
 * Makro to send a command that only expects a one byte confirmation code
 * @args: data bytes
 * @return: the confirmation code
 */
#define SEND_COMMAND(...) \
    uint8_t data[] = {__VA_ARGS__}; \
    sendPackage(Package(PID_COMMAND, sizeof(data), data)); \
    uint8_t confirmationCode; \
    return receiveAcknowledge(&confirmationCode, 1);

Package::Package(uint16_t length, uint8_t *data) : length(length), data(data) {}

Package::Package(uint8_t id, uint16_t length, uint8_t *data) : id(id), length(length), data(data)  {
    calculateChecksum();
}

void Package::calculateChecksum() {
    checksum = id;
    checksum += (length + 2) >> 8;
    checksum += (length + 2) & 0xFF;
    for(uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
}

bool Package::checksumMatches() {
    uint16_t original = checksum;
    calculateChecksum();
    if(original == checksum)
        return true;
    checksum = original;
    return false;
}

void R503::fallingISR() {
    //R503::instance->onFingerDown();
}

void R503::risingISR() {
    //R503::instance->onFingerUp();
}

R503::R503(int wakeupPin, uint32_t address, uint32_t password, long baudrate) : 
    wakeupPin(wakeupPin), address(address), password(password), baudrate(baudrate) {
    //R503::instance = this;
}

#ifdef DEBUG_R503
R503::R503(int wakeupPin, uint32_t address, uint32_t password, long baudrate, WiFiConsole *console) : 
    R503(wakeupPin, address, password, baudrate) {
    this->console = console;    
}
#endif

void R503::begin() {
    Serial.begin(baudrate);
    pinMode(wakeupPin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(wakeupPin), R503::fallingISR, FALLING);
    //attachInterrupt(digitalPinToInterrupt(wakeupPin), R503::risingISR, RISING);
}

void R503::sendPackage(Package const &package) {
    uint16_t length = package.length + 2;
    uint8_t bytes[] = {
        0xEF, 0x01,
        address >> 24, address >> 16, address >> 8, address,
        package.id,
        length >> 8, length
    };
    
    #ifdef DEBUG_R503
    static int packageCount = 0;
    console->printf("sending package %d: ", packageCount++);
    for(int i = 0; i < sizeof(bytes); i++) {
        console->printf("%02X ", bytes[i]);
    }
    for(int i = 0; i < package.length; i++) {
        console->printf("%02X ", package.data[i]);
    }
    console->printf("%02X %02X\n", package.checksum >> 8, package.checksum & 0xFF); 
    #endif
    
    Serial.write(bytes, sizeof(bytes));
    Serial.write(package.data, package.length);
    Serial.write(package.checksum >> 8);
    Serial.write(package.checksum);
}

int R503::receivePackage(Package &package) {
    uint8_t bytes[9];
    int ret;
    
    #ifdef DEBUG_R503
    static int packageCount = 0;
    console->printf("receiving package %d: ", packageCount++);
    #endif
    
    ret = Serial.readBytes(bytes, 9);
    if(ret != 9) {
        #ifdef DEBUG_R503
        console->printf("timeout %d \n", ret);
        #endif
        return ERROR_TIMEOUT;
    }
    #ifdef DEBUG_R503
    for(int i = 0; i < sizeof(bytes); i++) {
        console->printf("%02X ", bytes[i]);
    }
    #endif
    
    // header
    uint16_t header = bytes[0] << 8 | bytes[1];
    if(header != 0xEF01) {
        #ifdef DEBUG_R503
        console->printf("header mismatch\n");
        #endif
        return ERROR_HEADER_MISMATCH;
    }
    //address
    uint32_t address = bytes[2] << 24 | bytes[3] << 16 | bytes[4] << 8 | bytes[5];
    if(address != this->address) {
        #ifdef DEBUG_R503
        console->printf("address mismatch\n");
        #endif
        return ERROR_ADDRESS_MISMATCH;
    }
    // package id
    package.id = bytes[6];
    // package length
    uint16_t length = bytes[7] << 8 | bytes[8];
    if(package.length < length - 2) {
        #ifdef DEBUG_R503
        console->printf("not enough memory\n");
        #endif
        return ERROR_NOT_ENOUGH_MEMORY;
    }
    
    // package data
    ret = Serial.readBytes(package.data, package.length);
    if(ret != package.length) {
        #ifdef DEBUG_R503
        console->printf("timeout\n");
        #endif
        return ERROR_TIMEOUT;
    }
    #ifdef DEBUG_R503
    for(int i = 0; i < package.length; i++) {
        console->printf("%02X ", package.data[i]);
    }
    #endif
    
    
    // checksum
    ret = Serial.readBytes(bytes, 2);
    if(ret != 2) {
        #ifdef DEBUG_R503
        console->printf("timeout\n");
        #endif
        return ERROR_TIMEOUT;
    }
    package.checksum = bytes[0] << 8 | bytes[1];
    if(!package.checksumMatches()) {
        #ifdef DEBUG_R503
        console->printf("checksum mismatch\n");
        #endif
        return ERROR_CHECKSUM_MISMATCH;
    }
    #ifdef DEBUG_R503
    console->printf("%02X %02X\n", package.checksum >> 8 & 0xFF, package.checksum & 0xFF); 
    #endif
        
    return SUCCESS;
}

int R503::receiveAcknowledge(uint8_t *data, uint8_t length) {
    Package acknowledge(length, data);
    int ret = receivePackage(acknowledge);
    if(ret != SUCCESS)
        return ret;
    if(acknowledge.id != PID_ACKNOWLEDGE)
        return ERROR_PID_MISMATCH;
    return data[0];
}

void R503::onFingerDown() {
    fingerDown++;
}

void R503::onFingerUp() {
    fingerUp++;
}

int R503::verifyPassword() {
    SEND_COMMAND(0x13, password >> 24, password >> 16, password >> 8, password);
}

int R503::readProductInfo(ProductInfo &info) {
    uint8_t command[1] = {0x3C};
    sendPackage(Package(PID_COMMAND, sizeof(command) + 2, command));
    
    uint8_t data[47];
    int ret = receiveAcknowledge(data, sizeof(data));
    
    memcpy(info.module_type, &data[1], 16);
    memcpy(info.module_batch_number, &data[17], 4);
    memcpy(info.module_serial_number, &data[21], 8);
    info.hardware_version[0] = data[29];
    info.hardware_version[1] = data[30];
    memcpy(info.sensor_type, &data[31], 8);
    info.sensor_width = data[39] << 8 | data[40];
    info.sensor_height = data[41] << 8 | data[42];
    info.template_size = data[43] << 8 | data[44];
    info.database_size = data[45] << 8 | data[46];
    
    return ret;
}

int R503::auraControl(uint8_t control, uint8_t speed, uint8_t color, uint8_t times) {
    SEND_COMMAND(0x35, control, speed, color, times);
}
