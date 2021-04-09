/*
 * programm to test fingerprint sensor functionalities, useful to manipulate the fingerprint library on the sensor
 * you may need to adapt baudarate for running this example
 */

#include <R503.hpp>

#define RX_PIN 13
#define TX_PIN 12
#define BAUDRATE 57600

R503 fps(RX_PIN, TX_PIN, 0xFFFFFFFF, 0x0, BAUDRATE); // fps: finger print sensor

void setup() {
  Serial.begin(115200);
  delay(200);
  int ret = fps.init();
  while(ret != R503_SUCCESS) {
    Serial.printf("initializing R503 failed: %s, trying again ...\n", R503::errorMsg(ret));
    delay(1000);
    ret = fps.init();
  }
  Serial.printf("R503 initialised\n");
}

void loop() {
  // ask for the action
  Serial.printf("========================================\n");
  Serial.printf("choose an action\nprint index table [p]\nregister finger [r]\ndelete finger [d]\nempty library [e]\nsearch finger [s]\nmatch finger [m]\ninput << ");
  String str;
  do {
    str = Serial.readStringUntil('\n');
  } while(str.length() < 1);
  Serial.printf("%c\n", str[0]);
  Serial.printf("----------------------------------------\n");
  char action = str[0];
  switch(action) {
    case 'p': printIndexTable(); break;
    case 'r': registerFinger(); break;
    case 'd': deleteFinger(); break;
    case 'e': emptyLibrary(); break;
    case 's': searchFinger(); break;
    case 'm': matchFinger(); break;
    default: Serial.printf("%c is not a valid action\n", action);
  }
  Serial.printf("========================================\n");
}

void printIndexTable() {
  int ret;

  uint16_t count;
  ret = fps.templateCount(count);
  if(ret == R503_SUCCESS) {
    Serial.printf("amount of templates: %d\n", count);
  } else {
    Serial.printf("reading template count failed: %s\n", R503::errorMsg(ret));
  }
  
  SystemParameter param;
  ret = fps.readSystemParameter(param);
  if(ret == R503_SUCCESS) {
    Serial.printf("fingerprint library size: %d\n", param.finger_library_size);
  } else {
    Serial.printf("reading system parameters failed: %s\n", R503::errorMsg(ret));
  }

  // only print first page, since we know library size is less than 256
  uint8_t table[32];
  ret = fps.readIndexTable(table);
  if(ret == R503_SUCCESS) {
    Serial.printf("fingerprint stored at locations: ");
    for(int i = 0; i < 32; i++) {
      for(int b = 0; b < 8; b++) {
        if(table[i] >> b & 0x01) {
          Serial.printf("%d ", i * 8 + b);
        }
      }
    }
    Serial.printf("\n");
  } else {
    Serial.printf("reading index table failed: %s\n", R503::errorMsg(ret));
  }
}

void registerFinger() {
  Serial.printf("enter location where finger should be stored\ninput << ");
  String str;
  do {
    str = Serial.readStringUntil('\n');
  } while(str.length() < 1);
  uint16_t location = str.toInt();
  Serial.printf("%d\n", location);
  int ret = enrollFinger(location, 3);
  if(ret == R503_SUCCESS) {
    Serial.printf("finger stored at location %d\n", location);
  } else {
    Serial.printf("enrolling finger failed\n");
  }
}

void deleteFinger() {
  Serial.printf("enter location of finger, which should be deleted\ninput << ");
  String str;
  do {
    str = Serial.readStringUntil('\n');
  } while(str.length() < 1);
  uint16_t location = str.toInt();
  Serial.printf("%d\n", location);
  int ret = fps.deleteTemplate(location);
  if(ret == R503_SUCCESS) {
    Serial.printf("finger at location %d deleted\n", location);
  } else {
    Serial.printf("deleting finger failed: %s\n", R503::errorMsg(ret));
  }
}

void emptyLibrary() {
  Serial.printf("do you really want to empty library yes [y] / no [n]\ninput << ");
  String str;
  do {
    str = Serial.readStringUntil('\n');
  } while(str.length() < 1);
  Serial.printf("%c\n", str[0]);
  if(str[0] == 'y') {
    int ret = fps.emptyLibrary();
    if(ret == R503_SUCCESS) {
      Serial.printf("finger library emptied\n");
    } else {
      Serial.printf("emptied library : %s\n", R503::errorMsg(ret));
    }
  } else {
    Serial.printf("aborted\n");
  }
}

void searchFinger() {
  Serial.printf("put finger on sensor\n");
  int ret;
  while(true) {
    ret = fps.takeImage();
    if(ret != R503_SUCCESS) {
      Serial.printf("taking image failed: %s, trying again ... \n", R503::errorMsg(ret));
      delay(2000);
      continue;
    }
    ret = fps.extractFeatures(1);
    if(ret != R503_SUCCESS) {
      Serial.printf("extracting features failed: %s, taking new image ... \n", R503::errorMsg(ret));
      delay(2000);
      continue;
    }
    break;
  }
  uint16_t location, score;
  ret = fps.searchFinger(1, location, score);
  if(ret == R503_NO_MATCH_IN_LIBRARY) {
    Serial.printf("no matching finger found\n");
  } else if(ret == R503_SUCCESS) {
    Serial.printf("found finger %d with score %d\n", location, score);
  } else {
    Serial.printf("search library failed: %s\n", R503::errorMsg(ret));
  }
}

void matchFinger() {
  Serial.printf("enter location of finger to compare with\ninput << ");
  String str;
  do {
    str = Serial.readStringUntil('\n');
  } while(str.length() < 1);
  uint16_t location = str.toInt();
  Serial.printf("%d\n", location);
  int ret = fps.loadTemplate(1, location);
  if(ret != R503_SUCCESS) {
    Serial.printf("failed to load template: %s\n", R503::errorMsg(ret));
    return;
  }
  
  Serial.printf("put finger on sensor\n");
  while(true) {
    ret = fps.takeImage();
    if(ret != R503_SUCCESS) {
      Serial.printf("taking image failed: %s, trying again ... \n", R503::errorMsg(ret));
      delay(2000);
      continue;
    }
    ret = fps.extractFeatures(3);
    if(ret != R503_SUCCESS) {
      Serial.printf("extracting features failed: %s, taking new image ... \n", R503::errorMsg(ret));
      delay(2000);
      continue;
    }
    break;
  }
  uint16_t score;
  ret = fps.matchFinger(score);
  if(ret == R503_NO_MATCH) {
    Serial.printf("template at location %d does not match finger\n", location);
  } else if(ret == R503_SUCCESS) {
    Serial.printf("finger match with score %d\n", score);
  } else {
    Serial.printf("matching finger failed: %s\n", R503::errorMsg(ret));
  }
}

/*
 * enrolls the finger present on the sensor and stores it at @location
 * template is created through fusion of @features
 * @return: R503_NO_FINGER if no finger is present on the sensor
 */
int enrollFinger(uint16_t location, uint8_t features) {
  int ret;
  // take @features images, extract the features and write them to the character buffers
  for(int i = 1; i <= features; i++) {
    Serial.printf("put finger on sensor, taking image %d of %d\n", i, features);
    while(true) {
      ret = fps.takeImage();
      if(ret != R503_SUCCESS) {
        Serial.printf("taking image failed: %s, trying again ... \n", R503::errorMsg(ret));
        delay(2000);
        continue;
      }
      ret = fps.extractFeatures(i);
      if(ret != R503_SUCCESS) {
        Serial.printf("extracting features failed: %s, taking new image ... \n", R503::errorMsg(ret));
        delay(2000);
        continue;
      }
      break;
    }
    Serial.printf("lift finger from sensor\n");
    while(fps.takeImage() != R503_NO_FINGER) {
      delay(100);
    }
  }
  ret = fps.createTemplate();
  if(ret != R503_SUCCESS) {
    Serial.printf("creating template failed: %s\n", R503::errorMsg(ret));
    return ret;
  }
  ret = fps.storeTemplate(1, location);
  if(ret != R503_SUCCESS) {
    Serial.printf("storing template failed: %s\n", R503::errorMsg(ret));
    return ret;
  }
  return R503_SUCCESS;
}
