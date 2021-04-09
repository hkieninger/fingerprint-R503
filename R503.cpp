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
    uint16_t dataSize = sizeof(data); \
    int confirmationCode = receiveAcknowledge(data, dataSize);

/* 
 * Makro to send a command that only expects a one byte confirmation code
 * @VA_ARGS: the command data bytes to send
 * @creates return: confirmation code (0 success, < 0 ESP8266 side error, > 0 R503 side error)
 */
#define SEND_CMD(...) \
    RECEIVE_ACK(1,__VA_ARGS__) \
    return confirmationCode;
    
// =============================================================================
// implementation of package structure
// =============================================================================   
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

// =============================================================================
// implementation of R503 class
// =============================================================================  

char const *R503::errorMsg(int code) {
    switch(code) {
        case R503_SUCCESS: return "success";
        // ESP8266 side confirmation codes
        case R503_ADDRESS_MISMATCH: return "address mismatch";
        case R503_CHECKSUM_MISMATCH: return "checksum mismatch";
        case R503_TIMEOUT: return "timeout";
        case R503_PID_MISMATCH: return "package id mismatch";
        case R503_NOT_ENOUGH_MEMORY: return "not enough memory";
        case R503_SPECIFICATION_ERROR: return "specification error";
        case R503_INVALID_BAUDRATE: return "invalid baudrate";
        case R503_RESET_TIMEOUT: return "no handshake sign received";
        // R503 side confirmation codes
        case R503_ERROR_RECEIVING_PACKAGE: return "error receiving package";
        case R503_WRONG_PASSWORD: return "wrong password";
        case R503_NO_FINGER: return "no finger";
        case R503_ERROR_TAKING_IMAGE: return "error taking image";
        case R503_IMAGE_MESSY: return "image messy";
        case R503_FEATURE_FAIL: return "feature fail";
        case R503_NO_IMAGE: return "image buffer empty";
        case R503_BAD_LOCATION: return "invalid location (page id outside finger library)";
        case R503_ERROR_WRITING_FLASH: return "error writing flash";
        case R503_NO_MATCH: return "features do not match template";
        case R503_NO_MATCH_IN_LIBRARY: return "no matching template in fingerprint library";
        case R503_SENSOR_ABNORMAL: return "sensor abnormal";
        default:
            // reserve enough memory to print signed 16 bit or 32 bit integer
            static char str[sizeof("unknown error code ") + 1 + sizeof(int) * 8 / 3]; 
            snprintf(str, sizeof(str), "unknown error code %d", code);
            return str;
    }
}

R503::R503(int rxPin, int txPin, uint32_t address, uint32_t password, long baudrate) : 
    rxPin(rxPin), txPin(txPin), address(address), password(password), baudrate(baudrate) {
    serial = new SoftwareSerial(rxPin, txPin);
}

R503::~R503() {
    delete serial;
}

int R503::init() {
    serial->begin(baudrate);
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    
    int ret = verifyPassword();
    if(ret != R503_SUCCESS) {
        #if R503_DEBUG & 0x02
        Serial.printf("error verifying password: %s\n", R503::errorMsg(ret));
        #endif
        return ret;
    }
    
    SystemParameter param;
    ret = readSystemParameter(param);
    if(ret != R503_SUCCESS) {
        #if R503_DEBUG & 0x02
        Serial.printf("error reading system parameters: %s\n", R503::errorMsg(ret));
        #endif
        return ret;
    }
    finger_library_size = param.finger_library_size;
    data_package_size = param.data_package_size;

    return R503_SUCCESS;
}

void R503::sendPackage(Package const &package) {
    uint16_t length = package.length + 2;
    uint8_t bytes[] = {
        0xEF, 0x01,
        address >> 24, address >> 16, address >> 8, address,
        package.id,
        length >> 8, length
    };
    
    #if R503_DEBUG & 0x01
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
    #if R503_DEBUG & 0x01
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
        #if R503_DEBUG & 0x01
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
                    #if R503_DEBUG & 0x03
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
                    #if R503_DEBUG & 0x03
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
                            #if R503_DEBUG & 0x03
                            Serial.printf("error: checksum mismatch\n");
                            #endif
                            return R503_CHECKSUM_MISMATCH;
                        } else
                            #if R503_DEBUG & 0x01
                            Serial.printf("\n");
                            #endif
                            return R503_SUCCESS;
                    }
                }
        }
        index++;
    }
    #if R503_DEBUG & 0x03
    Serial.printf("error: timeout\n");
    #endif
    return R503_TIMEOUT;
}

int R503::receiveAcknowledge(uint8_t *data, uint16_t &length) {
    Package acknowledge(length, data);
    int ret = receivePackage(acknowledge);
    length = acknowledge.length;
    if(ret != R503_SUCCESS)
        return ret;
    if(acknowledge.id != PID_ACKNOWLEDGE)
        return R503_PID_MISMATCH;
    return data[0];
}

int R503::receiveData(uint8_t *data, uint16_t &length) {
    uint16_t offset = 0;
    uint8_t buffer[data_package_size];
    Package package(sizeof(buffer), buffer);
    int ret;
    do {
        ret = receivePackage(package);
        if(ret != R503_SUCCESS) {
            cleanSerial(length * 10 * 1000 / baudrate);
            return ret;
        }
        if(package.id != PID_DATA && package.id != PID_END) {
            cleanSerial(length * 10 * 1000 / baudrate);
            return R503_PID_MISMATCH;
        }
        if(offset + package.length > length) {
            cleanSerial();
            return R503_NOT_ENOUGH_MEMORY;
        }
        memcpy(data + offset, package.data, package.length);
        offset += package.length;
        package.length = sizeof(buffer);
    } while(package.id != PID_END);
    length = offset;
    return R503_SUCCESS;
}

void R503::cleanSerial(unsigned long milliseconds) {
    serial->flush();
    delay(milliseconds);
    while(serial->read() != -1);
}

int R503::verifyPassword() {
    SEND_CMD(0x13, password >> 24, password >> 16, password >> 8, password);
}

int R503::setBaudrate(long baudrate) {
    switch(baudrate) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
            {
            RECEIVE_ACK(1, 0x0E, 4, baudrate / 9600);
            if(confirmationCode == R503_SUCCESS) {
                serial->end();
                serial->begin(baudrate);
                this->baudrate = baudrate;
            }
            return confirmationCode;
            }
        default: 
            return R503_INVALID_BAUDRATE;
    }
}

int R503::setSecurityLevel(uint8_t level) {
    SEND_CMD(0x0E, 5, level);
}

int R503::auraControl(uint8_t control, uint8_t speed, uint8_t color, uint8_t times) {
    SEND_CMD(0x35, control, speed, color, times);
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

int R503::readSystemParameter(SystemParameter &param) {
    RECEIVE_ACK(17, 0x0F);
    
    param.status_register = data[1] << 8 | data[2];
    param.system_identifier_code = data[3] << 8 | data[4];
    param.finger_library_size = data[5] << 8 | data[6];
    param.security_level = data[7] << 8 | data[8];
    param.device_address = data[9] << 24 | data[10] << 16 | data[11] << 8 | data[12];
    param.data_package_size = 32 << (data[13] << 8 | data[14]);
    param.baudrate = 9600 * (data[15] << 8 | data[16]);
    
    return confirmationCode;
}

int R503::readInformationPage(char *info) {
    RECEIVE_ACK(1, 0x16);
    if(confirmationCode != R503_SUCCESS)
        return confirmationCode;
    
    uint16_t length = 512;
    return receiveData(reinterpret_cast<uint8_t *>(info), length);
}

int R503::handShake() {
    SEND_CMD(0x40);
}

int R503::checkSensor() {
    SEND_CMD(0x36);
}

int R503::softReset() {
    RECEIVE_ACK(1, 0x3D);
    if(confirmationCode != R503_SUCCESS)
        return confirmationCode;
    unsigned long start = millis();
    while(millis() - start < R503_SOFTRESET_TIMEOUT) {
        int byte = serial->read();
        if(byte == -1) {
            delay(1);
        } else if(byte == 0x55) {
            return R503_SUCCESS;
        }
    }
    return R503_RESET_TIMEOUT;
}

int R503::templateCount(uint16_t &count) {
    RECEIVE_ACK(3, 0x1D);
    count = data[1] << 8 | data[2]; 
    return confirmationCode;
}

int R503::readIndexTable(uint8_t *table, uint8_t page) {
    RECEIVE_ACK(33, 0x1F, page);
    memcpy(table, &data[1], 32);
    return confirmationCode;
}

int R503::takeImage() {
    SEND_CMD(0x01);
}

int R503::uploadImage(uint8_t *image, uint16_t &size) {
    RECEIVE_ACK(1, 0x0A);
    if(confirmationCode != R503_SUCCESS)
        return confirmationCode;
    return receiveData(image, size);
}

int R503::extractFeatures(uint8_t characterBuffer) {
    SEND_CMD(0x02, characterBuffer);
}

int R503::createTemplate() {
    SEND_CMD(0x05);
}

int R503::storeTemplate(uint8_t characterBuffer, uint16_t location) {
    SEND_CMD(0x06, characterBuffer, location >> 8, location);
}

int R503::deleteTemplate(uint16_t location) {
    SEND_CMD(0x0C, location >> 8, location, 0x00, 0x01);
}

int R503::emptyLibrary() {
    SEND_CMD(0x0D);
}

int R503::loadTemplate(uint8_t characterBuffer, uint16_t location) {
    SEND_CMD(0x07, characterBuffer, location >> 8, location);
}

int R503::matchFinger(uint16_t &score) {
    RECEIVE_ACK(3, 0x03);
    score = data[1] << 8 | data[2]; 
    return confirmationCode;
}

int R503::searchFinger(uint8_t characterBuffer, uint16_t &location, uint16_t &score) {
    uint16_t startPage = 0;
    uint16_t pageCount = finger_library_size;
    RECEIVE_ACK(5, 0x04, characterBuffer, startPage >> 8, startPage, pageCount >> 8, pageCount);
    location = data[1] << 8 | data[2];
    score = data[3] << 8 | data[4];
    return confirmationCode;
}

#if R503_DEBUG & 0x04
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
        Serial.printf("error retreiving product info: %s\n", R503::errorMsg(ret));
    }
    return ret;
}

int R503::printSystemParameter() {
    SystemParameter param;
    int ret = readSystemParameter(param);
    if(ret == R503_SUCCESS) {
        Serial.printf("status register: 0x%02X\nsystem identifier code: 0x%04X\n"
            "finger library size: %d\nsecurity level: %d\ndevice address: 0x%08X\n"
            "data package size: %d bytes\nbaudrate: %d\n",
            param.status_register, param.system_identifier_code, param.finger_library_size,
            param.security_level, param.device_address, param.data_package_size, param.baudrate);
    } else {
        Serial.printf("error retreiving sytem parameters: %s\n", R503::errorMsg(ret));
    }
    return ret;
}
#endif
