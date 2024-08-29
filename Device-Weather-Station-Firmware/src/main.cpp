#include <NodeControllerCore.h>
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MAX31865.h>

#define NODE_ID 0xAB

#define KPH_PER_SWITCH_CLOSE 2.4
#define MM_RAIN_PER_SWITCH_CLOSE 0.2794

#define WIND_VANE_RESISTOR 9998

#define SEALEVELPRESSURE_HPA (1013.25)

#define WIND_VANE_PIN 4
#define I2C_SDA 1
#define I2C_SCL 0
#define RAIN_GAUGE_PIN 2
#define ANEMOMETER_PIN 5

#define RTD_CLK 8
#define RTD_CS 3
#define RTD_DI 10
#define RTD_DO 9

NodeControllerCore core;

Adafruit_BME280 bme;

Adafruit_MAX31865 thermo = Adafruit_MAX31865(RTD_CS, RTD_DI, RTD_DO, RTD_CLK);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

void rain_gauge_interrupt();
void anemometer_interrupt();

//Temperature
#define TEMPERATURE_MEASUREMENT_INTERVAL 1000
#define TEMPERATURE_AVERAGE_INTERVAL 60 * 1000

uint32_t temperatureAverageIntervalCount = 0;
float temperatureBMEAverageIntervalSum = 0;
float temperatureRTDAverageIntervalSum = 0;

//Humidity
#define HUMIDITY_MEASUREMENT_INTERVAL 1000
#define HUMIDITY_AVERAGE_INTERVAL 60 * 1000
uint32_t humidityAverageIntervalCount = 0;
float humidityAverageIntervalSum = 0;

//Pressure
#define PRESSURE_MEASUREMENT_INTERVAL 1000
#define PRESSURE_AVERAGE_INTERVAL 60 * 1000
uint32_t pressureAverageIntervalCount = 0;
float pressureAverageIntervalSum = 0;

//Wind speed
#define WIND_SPEED_MEASUREMENT_INTERVAL 100
#define WIND_SPEED_AVERAGE_INTERVAL 60 * 1000
#define MIN_WIND_SPEED_DELTA 10000

uint32_t anemometerLastPulse = 0;
uint32_t anemometerCurrentTimeDelta = 0;

float windSpeedAverageIntervalSum = 0;
uint16_t windSpeedAverageIntervalCount = 0;
float windSpeedIntervalMax = 0;

//Wind Direction
#define WIND_DIRECTION_MEASUREMENT_INTERVAL 100
#define WIND_DIRECTION_AVERAGE_INTERVAL 60 * 1000

uint32_t windDirectionAverageDirectionCounts[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t windDirectionAverageIntervalCount = 0;
uint64_t windDirectionMaxDirection = 0;

uint64_t windDirectionToMVMap[8][2] = {{0,2533},
                                      {45,1487},
                                      {90,300},
                                      {135,595},
                                      {180,926},
                                      {225,2031},
                                      {270,3046},
                                      {315,2859}};

//Rain gauge
#define RAIN_GAUGE_AVERAGE_INTERVAL 60 * 1000
#define MM_RAIN_PER_SWITCH_CLOSE 0.2794
#define RAIN_GAUGE_DEBOUNCE_TIME 500

uint64_t rainGaugeLastSwitchTime = 0;
float rainGaugeAverageIntervalSum = 0;

void IRAM_ATTR rain_gauge_interrupt(){
  if(millis() < rainGaugeLastSwitchTime + RAIN_GAUGE_DEBOUNCE_TIME){
    return;
  }
  rainGaugeAverageIntervalSum += MM_RAIN_PER_SWITCH_CLOSE;
  rainGaugeLastSwitchTime = millis();
}

void IRAM_ATTR anemometer_interrupt(){
  if(millis() < anemometerLastPulse + 10){
    return;
  }
  //ets_printf("Anemometer interrupt %d\n", millis());
  anemometerCurrentTimeDelta = millis() - anemometerLastPulse;
  anemometerLastPulse = millis();
}

void updateWindDirection(void *parameters){
  while(1){
    delay(WIND_DIRECTION_MEASUREMENT_INTERVAL);
    uint32_t windVaneValue = analogReadMilliVolts(WIND_VANE_PIN);
    //Serial.println("Wind Vane voltage: " + String(windVaneValue));
    uint8_t windDirection = 0;
    for(int i = 0; i < 8; i++){
      if(windVaneValue >= windDirectionToMVMap[i][1] - 100 && windVaneValue <= windDirectionToMVMap[i][1] + 100){
        windDirection = i;
        break;
      }
    }
    windDirectionAverageDirectionCounts[windDirection]++;
    windDirectionAverageIntervalCount++;
  }
}

void updateWindDirectionAverage(void *parameters){
  while(1){
    delay(WIND_DIRECTION_AVERAGE_INTERVAL);
    windDirectionMaxDirection = 0;
    for(int i = 0; i < 8; i++){
      if(windDirectionAverageDirectionCounts[i] > windDirectionAverageDirectionCounts[windDirectionMaxDirection]){
        windDirectionMaxDirection = i;
      }
      float windDirectionAverage = (float)windDirectionAverageDirectionCounts[i]/(float)windDirectionAverageIntervalCount * 100.0;
      Serial.println("Wind direction " + String(windDirectionToMVMap[i][0]) + "°: " + windDirectionAverage + "%");
      core.sendMessage(0xA501 + i, &windDirectionAverage);
    }
    //float windDirectionMax = windDirectionToMVMap[windDirectionMaxDirection][0];
    Serial.println("Wind direction max: " + String(windDirectionToMVMap[windDirectionMaxDirection][0]) + "°");
    core.sendMessage(0xA500, &windDirectionToMVMap[windDirectionMaxDirection][0]);
    windDirectionAverageIntervalCount = 0;
    for(int i = 0; i < 8; i++){
      windDirectionAverageDirectionCounts[i] = 0;
    } 
  }
}

void updateWindSpeed(void *parameters){
  while (1)
  {
    delay(WIND_SPEED_MEASUREMENT_INTERVAL);
    float currentWindSpeed = 0;
    if(((millis() - anemometerLastPulse) <= MIN_WIND_SPEED_DELTA) && (anemometerCurrentTimeDelta != 0)){
       currentWindSpeed = 1000.0 / anemometerCurrentTimeDelta * 2.4;
    }else{
      currentWindSpeed = 0;
    }
    windSpeedAverageIntervalCount++;
    windSpeedAverageIntervalSum += currentWindSpeed;
    if(currentWindSpeed > windSpeedIntervalMax){
      windSpeedIntervalMax = currentWindSpeed;
    }
  }
}

void updateWindSpeedAverage(void *parameters){
  while(1){
    delay(WIND_SPEED_AVERAGE_INTERVAL);
    float windSpeedAverage = windSpeedAverageIntervalSum / windSpeedAverageIntervalCount;
    Serial.println("Wind speed average: " + String(windSpeedAverage) + " km/h");
    Serial.println("Wind speed max: " + String(windSpeedIntervalMax) + " km/h");
    core.sendMessage(0xA400, &windSpeedAverage);
    core.sendMessage(0xA401, &windSpeedIntervalMax);
    windSpeedAverageIntervalCount = 0;
    windSpeedAverageIntervalSum = 0;
    windSpeedIntervalMax = 0;
  }
}

void updateTemperature(void *parameters){
  while (1)
  {
    delay(TEMPERATURE_MEASUREMENT_INTERVAL);
    uint8_t fault = thermo.readFault();
    if(!fault){
      temperatureBMEAverageIntervalSum += bme.readTemperature();
      temperatureRTDAverageIntervalSum += thermo.temperature(RNOMINAL, RREF);
      temperatureAverageIntervalCount++; 
    }else{
      Serial.print("Fault 0x"); Serial.println(fault, HEX);
      core.sendMessage(0x4000, (uint64_t*)&fault);
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold"); 
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold"); 
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias"); 
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage"); 
      }
      thermo.clearFault();
    }
  }
}

void updateTemperatureAverage(void *parameters){
  while (1)
  {
    delay(TEMPERATURE_AVERAGE_INTERVAL);
    float temperatureBMEAverage = temperatureBMEAverageIntervalSum / temperatureAverageIntervalCount;
    float temperatureRTDAverage = temperatureRTDAverageIntervalSum / temperatureAverageIntervalCount;
    Serial.println("Temperature BME average: " + String(temperatureBMEAverage) + " °C");
    Serial.println("Temperature RTD average: " + String(temperatureRTDAverage) + " °C");
    core.sendMessage(0xA100, &temperatureBMEAverage);
    core.sendMessage(0xA101, &temperatureRTDAverage);
    temperatureAverageIntervalCount = 0;
    temperatureBMEAverageIntervalSum = 0;
    temperatureRTDAverageIntervalSum = 0;
  }
}

void updateHumidity(void *parameters){
  while(1){
    delay(HUMIDITY_MEASUREMENT_INTERVAL);
    humidityAverageIntervalSum += bme.readHumidity();
    humidityAverageIntervalCount++;
  }
}

void updateHumidityAverage(void *parameters){
  while(1){
    delay(HUMIDITY_AVERAGE_INTERVAL);
    float humidityAverage = humidityAverageIntervalSum / humidityAverageIntervalCount;
    Serial.println("Humidity average: " + String(humidityAverage) + " %");
    core.sendMessage(0xA200, &humidityAverage);
    humidityAverageIntervalCount = 0;
    humidityAverageIntervalSum = 0;
  }
}

void updatePressure(void *parameters){
  while(1){
    delay(PRESSURE_MEASUREMENT_INTERVAL);
    pressureAverageIntervalSum += bme.readPressure();
    pressureAverageIntervalCount++;
  }
}

void updatePressureAverage(void *parameters){
  while(1){
    delay(PRESSURE_AVERAGE_INTERVAL);
    float pressureAverage = pressureAverageIntervalSum / pressureAverageIntervalCount;
    Serial.println("Pressure average: " + String(pressureAverage / 100.0F) + " hPa");
    core.sendMessage(0xA300, &pressureAverage);
    pressureAverageIntervalCount = 0;
    pressureAverageIntervalSum = 0;
  }
}

void updateRainGaugeAverage(void *parameters){
  while(1){
    delay(RAIN_GAUGE_AVERAGE_INTERVAL);
    Serial.println("Rain gauge average: " + String(rainGaugeAverageIntervalSum) + " mm");
    core.sendMessage(0xA600, &rainGaugeAverageIntervalSum);
    rainGaugeAverageIntervalSum = 0;
  }
}

void receivedCANBUSMessage(uint8_t nodeID, uint16_t messageID, uint64_t data){
  //Serial.println("Received message from node " + String(nodeID) + " with message ID " + String(messageID) + " and data " + String(data));
}

void setup() {
  // put your setup code here, to run once:
  pinMode(WIND_VANE_PIN,INPUT);
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  // Start the Node Controller
  core = NodeControllerCore();
  if(core.Init(receivedCANBUSMessage, NODE_ID)){
      Serial.println("Node Controller Core Started");
  } else {
      Serial.println("Node Controller Core Failed to Start");
  }

  //BME280
  unsigned status;
    
  // default settings
  status = bme.begin(0x77,&Wire); 
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }

  //MAX31865
  thermo.begin(MAX31865_3WIRE);

  //Interrupts
  pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), rain_gauge_interrupt, FALLING);

  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(ANEMOMETER_PIN, anemometer_interrupt, RISING);

  //Tasks

  //Wind speed
  xTaskCreate(
    updateWindSpeed,
    "updateWindSpeed",
    10000,
    NULL,
    10,
    NULL
  );
  xTaskCreate(
    updateWindSpeedAverage,
    "updateWindSpeedAverage",
    10000,
    NULL,
    20,
    NULL
  );

  //Temperature
  xTaskCreate(
    updateTemperature,
    "updateTemperature",
    10000,
    NULL,
    10,
    NULL
  );
  xTaskCreate(
    updateTemperatureAverage,
    "updateTemperatureAverage",
    10000,
    NULL,
    20,
    NULL
  );

  //Humidity
  xTaskCreate(
    updateHumidity,
    "updateHumidity",
    10000,
    NULL,
    10,
    NULL
  );
  xTaskCreate(
    updateHumidityAverage,
    "updateHumidityAverage",
    10000,
    NULL,
    20,
    NULL
  );

  //Pressure

  xTaskCreate(
    updatePressure,
    "updatePressure",
    10000,
    NULL,
    10,
    NULL
  );

  xTaskCreate(
    updatePressureAverage,
    "updatePressureAverage",
    10000,
    NULL,
    20,
    NULL
  );

  //Rain gauge
  xTaskCreate(
    updateRainGaugeAverage,
    "updateRainGaugeAverage",
    10000,
    NULL,
    20,
    NULL
  );

  //Wind direction
  xTaskCreate(
    updateWindDirection,
    "updateWindDirection",
    10000,
    NULL,
    10,
    NULL
  );
  xTaskCreate(
    updateWindDirectionAverage,
    "updateWindDirectionAverage",
    10000,
    NULL,
    20,
    NULL
  );

}

void loop() {
  // put your main code here, to run repeatedly:
  //int wind_vane_value = analogRead(WIND_VANE_PIN);

}

