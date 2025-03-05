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

float alpha = 0.1; // Smoothing factor (0 < alpha <= 1)
float smoothedSpeed = 0;
unsigned long prev_time = 0;
unsigned long curr_time = 0;
unsigned long time_passed = 0;

float circumference = PI * wheelDiameter;
float units = (METRIC) ? 1000.000 : 1609.344;

void countPulse() {

  pulseCount++;

}

void countTime() {
  prev_time = curr_time;
  if (pulseCount == 25)
  {
      curr_time = micros();
      time_passed = curr_time - prev_time;
      pulseCount = 0;
  }

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
  //attachInterrupt(digitalPinToInterrupt(sensorPin), countPulse, FALLING); 
  attachInterrupt(digitalPinToInterrupt(sensorPin), countTime, FALLING); 
}

void loop()
{


  	delay(100);
    float rps = (float)(1 / time_passed) * 1000000.0;
    smoothedSpeed = alpha * rps + (1 - alpha) * smoothedSpeed;
    rps = smoothedSpeed;
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
    CAN.sendMsgBuf(0x1, 0, 8, data);
}