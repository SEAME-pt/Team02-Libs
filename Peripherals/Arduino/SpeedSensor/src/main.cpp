
#include <SPI.h>          
#include <mcp_can.h>

MCP_CAN CAN(9);
byte data[1] = {20};

const bool METRIC = true; // Set to false for mph

const int sensorPin = 3;  
const float wheelDiameter = 0.67;  // Diameter in meters

const int pulsesPerRevolution = 20; // Pulses per wheel revolution

unsigned long pulseCount = 0;
unsigned long previousMillis = 0;
const long interval = 500;  

float circumference = PI * wheelDiameter;
float units = (METRIC) ? 1000.000 : 1609.344;

void countPulse() {

  pulseCount++;

}

void setup()
{
  Serial.begin(9600);

  Serial.println("CAN transmitter test");
   
  if(CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) 
    Serial.println("MCP2515 Initialized Successfully!");
  else 
    Serial.println("Error Initializing MCP2515...");
  
  CAN.setMode(MCP_NORMAL);

  pinMode(sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulse, FALLING); 
}

void loop()
{
  static int count = 0;                  
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();



  if (currentMillis - previousMillis >= interval) { 
    previousMillis = currentMillis;
    
    float rps = (float)pulseCount/ (interval / 1000.0); // RPS = Pulses / seconds
    float rpm = rps * 60; // Convert to RPM

    int speed = (int) rpm;
    
    float speed_kmph = (rpm * circumference * 60) / units;

    Serial.print("RPM: ");
    Serial.println(rpm);
    Serial.print("Speed (km/h): ");
    Serial.println(speed_kmph);


    pulseCount = 0;

    byte data[8];
    data[0] = (speed >> 24) & 0xFF; // Highest byte
    data[1] = (speed >> 16) & 0xFF; // High byte
    data[2] = (speed >> 8) & 0xFF;  // Low byte
    data[3] = speed & 0xFF;
    data[4] = 0 & 0xFF;
    data[5] = 0 & 0xFF;
    data[6] = 0 & 0xFF;
    data[7] = 0 & 0xFF;
             // Lowest byte

    for (int i = 0; i < 8; i++)
    {
      Serial.print(data[i]);
      Serial.print(", ");
    }
      Serial.println("");




    //byte sndStat = 
    CAN.sendMsgBuf(0x1, 0, 8, data);
    CAN.sendMsgBuf(0x2, 0, 8, data);
    // if (sndStat == CAN_OK) {
    //   Serial.println("Message Sent Successfully!");
    // } else {
    //   Serial.println("Error Sending Message...");
    // }

    count++;
  }
}


