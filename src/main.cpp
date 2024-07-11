#include <Arduino.h>

// #define LASER

#ifdef LASER

#include <ModbusMaster.h>
#include <SoftwareSerial.h>

ModbusMaster node;
SoftwareSerial mySerial(12, 13); //Define soft serial port, port 13 is TX, port 12 is RX


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Starting Modbus Master test");

  mySerial.begin(115200);  // Initialize Serial1 for Seeed Xiao nRF52840
  node.begin(2, mySerial);
}

void loop() {
  uint8_t result = node.writeSingleRegister(0x50 , 0x1234);
  if (result != node.ku8MBSuccess) {
    Serial.println("Write failed!");
  }

  result = node.readHoldingRegisters(0, 1);
  if (result != node.ku8MBSuccess) {
    Serial.println("Read failed!");
    Serial.print("Modbus error code: ");
    Serial.println(result);
  }

  delay(500);
}

#else
/*
       @File  : DFRobot_Distance_A01.ino
       @Brief : This example use A01NYUB ultrasonic sensor to measure distance
                With initialization completed, We can get distance value
       @Copyright [DFRobot](https://www.dfrobot.com),2016
                  GUN Lesser General Pulic License
       @version V1.0
       @data  2019-8-28
*/

#include <SoftwareSerial.h>

SoftwareSerial mySerial(11, 10); // RX, TX
unsigned char data[4] = {};
float distance;

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);
  Serial.println("DFRobot Ultrasonic Distance Sensor!");
}

void loop()
{
  do {
    for (int i = 0; i < 4; i++)
    {
      data[i] = mySerial.read();
    }
  } while (mySerial.read() == 0xff);

  mySerial.flush();

  if (data[0] == 0xff)
  {
    int sum;
    sum = (data[0] + data[1] + data[2]) & 0x00FF;
    if (sum == data[3])
    {
      distance = (data[1] << 8) + data[2];
      if (distance > 280)
      {
        Serial.print("distance=");
        Serial.print(distance / 10);
        Serial.println("cm");
      } else
      {
        Serial.println("Below the lower limit");
      }
    } else Serial.println("ERROR");
  }
  delay(150);
  // Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}

#endif