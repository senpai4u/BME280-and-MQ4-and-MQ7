int i=0;
#include "MQ7.h"
MQ7 mq7(A0,5.0);

#include <Wire.h>
#include "SparkFunBME280.h"
BME280 mySensor;

#include <SPI.h>   //Library for SPI interface 
#include <Wire.h>  //Library for I2C interface 
int gas_sensor = A1;                             //Sensor pin
float m = -0.318;                                //Slope
float b = 1.133;                                 //Y-Intercept
float R0 = 11.820;  


void setup() {  
  
Serial.begin(9600);
pinMode(13,OUTPUT); //FOR MQ7

Wire.begin();//for bme
if (mySensor.beginI2C() == false) //Begin communication over I2C
{
   Serial.println("The sensor did not respond. Please check wiring.");
   while(1); //Freeze
}

}

void loop() {
  
if(i<60){
  analogWrite(13,255);
  Serial.println("MQ7 is high" );
}
else if (i<150){
  analogWrite(13,72);
  Serial.println("MQ7 is low" );
}
else if(i=150){ 
  Serial.print("MQ7 ppm is ");
  digitalWrite(13,HIGH);
  delay(50);
  Serial.println(mq7.getPPM());
  i=0;
} 
  //BME
  Serial.print("Humidity: ");
  Serial.print(mySensor.readFloatHumidity(), 0);

  Serial.print(" Pressure: ");
  Serial.print(mySensor.readFloatPressure(), 0);

  Serial.print(" Alt: ");
  //Serial.print(mySensor.readFloatAltitudeMeters(), 1);
  Serial.print(mySensor.readFloatAltitudeMeters(), 1);

  Serial.print(" Temp: ");
  //Serial.print(mySensor.readTempC(), 2);
  Serial.println(mySensor.readTempC(), 2);

//mq4
float sensor_volt;                             //Define variable for sensor voltage
  float RS_gas;                                  //Define variable for sensor resistance
  float ratio;                                   //Define variable for ratio
  float sensorValue = analogRead(gas_sensor);    //Read analog values of sensor
  sensor_volt = sensorValue * (5.0 / 1023.0);    //Convert analog values to voltage
  RS_gas = ((5.0 * 10.0) / sensor_volt) - 10.0;  //Get value of RS in a gas
  ratio = RS_gas / R0;                           // Get ratio RS_gas/RS_air
  double ppm_log = (log10(ratio) - b) / m;       //Get ppm value in linear scale according to the the ratio value
  double ppm = pow(10, ppm_log);                 //Convert ppm value to log scale
  double percentage = ppm / 10000;               //Convert to percentage
  Serial.print("MQ4 ppm is ");
  Serial.println(ppm);
Serial.println();
delay(1000);
    i++;
}
