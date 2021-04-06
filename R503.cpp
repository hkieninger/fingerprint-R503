#include "R503.hpp"

/*
 *  Makro to send a command and receive the acknowledge package
 *  @ACK_SIZE: size of the acknowledge package or a larger integer 
 *      (corresponds to length of acknowledge package - 2 from data sheet, since checksum is already handled by Package class)
 *  @VA_ARGS: the command data bytes to send
 *  @creates data: array containing the acknowledge data
 *  @creates confirmationCode: confirmation code (0 success, < 0 ESP8266 side error, > 0 R503 side error)
 */
#define RECEIVE_ACK(ACK_SIZE,...) \
    uint8_t command[] = {__VA_ARGS__}; \
    sendPackage(Package(PID_COMMAND, sizeof(command), command)); \
    uint8_t data[ACK_SIZE]; \
    int confirmationCode = receiveAcknowledge(data, ACK_SIZE);

/* 
 * Makro to send a command that only expects a one byte confirmation code
 * @VA_ARGS: the command data bytes to send
 * @creates return: confirmation code (0 success, < 0 ESP8266 side error, > 0 R503 side error)
 */
#define SEND_CMD(...) \
    RECEIVE_ACK(1,__VA_ARGS__) \
    return confirmationCode;
    
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
    
    #ifdef R503_DEBUG
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
    #ifdef R503_DEBUG
    static int packageCount = 0;
    Serial.printf("receiving package %d: ", packageCount++);
    #endif

    unsigned long start = millis();
    int index = 0;
    uint16_t length;
    while(millis() - start < R503_RECEIVE_TIMEOUT) {
        int byte = serial->read();
        if(byte == -1)
            continue;
        #ifdef R503_DEBUG
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
                    #ifdef R503_DEBUG
                    Serial.printf("error: address mismatch\n");
                    #endif
                    return R503_ADDRESS_MISMATCH;
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
                    #ifdef R503_DEBUG
                    Serial.printf("error: not enough memory\n");
                    #endif
                    return R503_NOT_ENOUGH_MEMORY;
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
                            #ifdef R503_DEBUG
                            Serial.printf("error: checksum mismatch\n");
                            #endif
                            return R503_CHECKSUM_MISMATCH;
                        } else
                            #ifdef R503_DEBUG
                            Serial.printf("\n");
                            #endif
                            return R503_SUCCESS;
                    }
                }
        }
        index++;
    }
    #ifdef R503_DEBUG
    Serial.printf("error: timeout\n");
    #endif
    return R503_TIMEOUT;
}

int R503::receiveAcknowledge(uint8_t *data, uint8_t length) {
    Package acknowledge(length, data);
    int ret = receivePackage(acknowledge);
    if(ret != R503_SUCCESS)
        return ret;
    if(acknowledge.id != PID_ACKNOWLEDGE)
        return R503_PID_MISMATCH;
    return data[0];
}

void R503::onFingerDown() {
    
}

void R503::onFingerUp() {
    
}

int R503::verifyPassword() {
    SEND_CMD(0x13, password >> 24, password >> 16, password >> 8, password);
}

int R503::readProductInfo(ProductInfo &info) {
    RECEIVE_ACK(47, 0x3C);
    
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
    
    return confirmationCode;
}

int R503::auraControl(uint8_t control, uint8_t speed, uint8_t color, uint8_t times) {
    SEND_CMD(0x35, control, speed, color, times);
}

int R503::readSystemParameter(SystemParameter &param) {
    RECEIVE_ACK(17, 0x0F);
    
    param.status_register = data[1] << 8 | data[2];
    param.system_identifier_code = data[3] << 8 | data[4];
    param.finger_library_size = data[5] << 8 | data[6];
    param.security_level = data[7] << 8 | data[8];
    param.device_address = data[9] << 24 | data[10] << 16 | data[11] << 8 | data[12];
    param.data_packet_size = 32 << (data[13] << 8 | data[14]);
    param.baudrate = 9600 * (data[15] << 8 | data[16]);
    
    return confirmationCode;
}

int R503::readInformationPage(char *info) {
    RECEIVE_ACK(1, 0x16);
    if(confirmationCode != R503_SUCCESS)
        return confirmationCode;
    //TODO add code the get the data packets
}

int R503::readModelCount(uint16_t &count) {
    RECEIVE_ACK(3, 0x1D);
    count = data[1] << 8 | data[2]; 
    return confirmationCode;
}

int R503::takeImage() {
    SEND_CMD(0x01);
}

int R503::extractFeatures(uint8_t featureBuffer) {
    SEND_CMD(0x02, featureBuffer);
}

int R503::createModel() {
    SEND_CMD(0x05);
}

int R503::storeModel(uint8_t featureBuffer, uint16_t location) {
    SEND_CMD(0x06, featureBuffer, location >> 8, location);
}

int R503::searchFinger(uint8_t featureBuffer, uint16_t &location, uint16_t &score) {
    uint16_t startPage = 0;
    uint16_t pageCount = 8; //TODO adapt via capacity of sensor
    RECEIVE_ACK(5, 0x04, featureBuffer, startPage >> 8, startPage, pageCount >> 8, pageCount);
    location = data[1] << 8 | data[2];
    score = data[3] << 8 | data[4];
    return confirmationCode;
}

#ifdef R503_DEBUG
int R503::printProductInfo() {
    ProductInfo info;
    int ret = readProductInfo(info);
    if(ret == R503_SUCCESS) {
        Serial.printf("module type: %s\nmodule batch number: %s\nmodule serial number: %s\n"
            "hardware version: %d.%d\nsensor type: %s\nsensor dimension: %dx%d\n"
            "sensor template size: %d\nsensor database size: %d\n",
            info.module_type, info.module_batch_number, info.module_serial_number,
            info.hardware_version[0], info.hardware_version[1], info.sensor_type,
            info.sensor_width, info.sensor_height, info.template_size, info.database_size);
    } else {
        Serial.printf("error retreiving product info: error code %d\n", ret);
    }
    return ret;
}

int R503::printSystemParameter() {
    SystemParameter param;
    int ret = readSystemParameter(param);
    if(ret == R503_SUCCESS) {
        Serial.printf("status register: %d\nsystem identifier code: %04X\n"
            "finger library size: %d\nsecurity level: %d\ndevice address: %08X\n"
            "data packet size: %d bytes\nbaudrate: %d\n",
            param.status_register, param.system_identifier_code, param.finger_library_size,
            param.security_level, param.device_address, param.data_packet_size, param.baudrate);
    } else {
        Serial.printf("error retreiving sytem parameters: error code %d\n", ret);
    }
    return ret;
}
#endif

bool R503::isTouched() {
    return !digitalRead(touchPin);
}
