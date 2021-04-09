/*
 * test if module and sensor are okay and print content of information registers
 * check baudrate and pins
 * (make sure R503_DEBUG in R503.hpp is set to 0x04)
 */

#include <R503.hpp>

#define RX_PIN 13
#define TX_PIN 12
#define BAUDRATE 57600

R503 fps(RX_PIN, TX_PIN, 0xFFFFFFFF, 0x0, BAUDRATE);

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
  Serial.printf("\n\n");
  int ret;
  // check if module is ok
  ret = fps.handShake();
  if(ret == R503_SUCCESS) {
    Serial.printf("module is okay, it responds to handshake\n");
  } else {
    Serial.printf("handshake failed: %s\n", R503::errorMsg(ret));
  }
  //check if sensor is ok
  ret = fps.checkSensor();
  if(ret == R503_SUCCESS) {
    Serial.printf("sensor is okay\n");
  } else {
    Serial.printf("sensor check failed: %s\n", R503::errorMsg(ret));
  }
  //print product information
  Serial.printf("\nproduct info:\n");
  fps.printProductInfo();
  Serial.printf("\nsystem parameters:\n");
  fps.printSystemParameter();

  delay(3000);
}
