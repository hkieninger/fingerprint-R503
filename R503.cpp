#include "R503.hpp"

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

R503::R503(int rxPin, int txPin, int touchPin, uint32_t address, uint32_t password, long baudrate) : 
    rxPin(rxPin), txPin(txPin), touchPin(touchPin), address(address), password(password), baudrate(baudrate) {
    //R503::instance = this;
    serial = new SoftwareSerial(rxPin, txPin);
}

R503::~R503() {
    delete serial;
}

void R503::begin() {
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    serial->begin(baudrate);
    pinMode(touchPin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(touchPin), R503::fallingISR, FALLING);
    //attachInterrupt(digitalPinToInterrupt(touchPin), R503::risingISR, RISING);
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
    Serial.printf("sending package %d: ", packageCount++);
    for(int i = 0; i < sizeof(bytes); i++) {
        Serial.printf("%02X ", bytes[i]);
    }
    for(int i = 0; i < package.length; i++) {
        Serial.printf("%02X ", package.data[i]);
    }
    Serial.printf("%02X %02X\n", package.checksum >> 8, package.checksum & 0xFF); 
    #endif
    
    serial->write(bytes, sizeof(bytes));
    serial->write(package.data, package.length);
    serial->write(package.checksum >> 8);
    serial->write(package.checksum);
}

int R503::receivePackage(Package &package) {
    #ifdef DEBUG_R503
    static int packageCount = 0;
    Serial.printf("receiving package %d: ", packageCount++);
    #endif

    unsigned long start = millis();
    int index = 0;
    uint16_t length;
    while(millis() - start < RECEIVE_TIMEOUT) {
        int byte = serial->read();
        if(byte == -1)
            continue;
        #ifdef DEBUG_R503
        Serial.printf("%02X ", byte);
        #endif
        switch(index) {
            case 0:
                if(byte != 0xEF)
                    continue;
                break;
            case 1:
                if(byte != 0x01) {
                    index = 0;
                    continue;
                }
                break;
            case 2:
            case 3:
            case 4:
            case 5:
                if(byte != ((address >> (5 - index) * 8) & 0xFF)) {
                    #ifdef DEBUG_R503
                    Serial.printf("error: address mismatch\n");
                    #endif
                    return ERROR_ADDRESS_MISMATCH;
                }
                break;
            case 6:
                package.id = byte;
                break;
            case 7:
                length = byte << 8;
                break;
            case 8:
                length |= byte;
                if(length - 2 > package.length) {
                    #ifdef DEBUG_R503
                    Serial.printf("error: not enough memory\n");
                    #endif
                    return ERROR_NOT_ENOUGH_MEMORY;
                }
                package.length = length - 2;
                break;
            default:
                if(index - 9 < package.length) {
                    package.data[index - 9] = byte;
                } else {
                    if(index - 9 == package.length) {
                        package.checksum = byte << 8;
                    } else {
                        package.checksum |= byte;
                        if(!package.checksumMatches()) {
                            #ifdef DEBUG_R503
                            Serial.printf("error: checksum mismatch\n");
                            #endif
                            return ERROR_CHECKSUM_MISMATCH;
                        } else
                            #ifdef DEBUG_R503
                            Serial.printf("\n");
                            #endif
                            return SUCCESS;
                    }
                }
        }
        index++;
    }
    #ifdef DEBUG_R503
    Serial.printf("error: timeout\n");
    #endif
    return ERROR_TIMEOUT;
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
    
}

void R503::onFingerUp() {
    
}

int R503::verifyPassword() {
    SEND_COMMAND(0x13, password >> 24, password >> 16, password >> 8, password);
}

int R503::readProductInfo(ProductInfo &info) {
    uint8_t command[1] = {0x3C};
    sendPackage(Package(PID_COMMAND, sizeof(command), command));
    
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

#ifdef DEBUG_R503
void R503::printProductInfo() {
    ProductInfo info;
    int ret = readProductInfo(info);
    if(ret == SUCCESS) {
        Serial.printf("module type: %s\nmodule batch number: %s\nmodule serial number: %s\n"
            "hardware version: %d.%d\nsensor type: %s\nsensor dimension: %dx%d\n"
            "sensor template size: %d\nsensor database size: %d\n",
            info.module_type, info.module_batch_number, info.module_serial_number,
            info.hardware_version[0], info.hardware_version[1], info.sensor_type,
            info.sensor_width, info.sensor_height, info.template_size, info.database_size);
    } else {
        Serial.printf("error retreiving product info: error code %d\n", ret);
    }
}
#endif

bool R503::isTouched() {
    return !digitalRead(touchPin);
}
