#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MAX31865.h>

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
#define TEMPERATURE_AVERAGE_INTERVAL 10000

uint32_t temperatureAverageIntervalCount = 0;
float temperatureBMEAverageIntervalSum = 0;
float temperatureRTDAverageIntervalSum = 0;

//Humidity
#define HUMIDITY_MEASUREMENT_INTERVAL 1000
#define HUMIDITY_AVERAGE_INTERVAL 10000
uint32_t humidityAverageIntervalCount = 0;
float humidityAverageIntervalSum = 0;

//Pressure
#define PRESSURE_MEASUREMENT_INTERVAL 1000
#define PRESSURE_AVERAGE_INTERVAL 10000
uint32_t pressureAverageIntervalCount = 0;
float pressureAverageIntervalSum = 0;

//Wind speed
#define WIND_SPEED_MEASUREMENT_INTERVAL 100
#define WIND_SPEED_AVERAGE_INTERVAL 10000

uint32_t anemometerLastPulse = 0;
uint32_t anemometerCurrentTimeDelta = 0;

float windSpeedAverageIntervalSum = 0;
uint16_t windSpeedAverageIntervalCount = 0;
float windSpeedIntervalMax = 0;

void IRAM_ATTR rain_gauge_interrupt(){
  Serial.println("Rain gauge interrupt");
  //delay(15);
}

void IRAM_ATTR anemometer_interrupt(){
  if(millis() < anemometerLastPulse + 10){
    return;
  }
  //ets_printf("Anemometer interrupt %d\n", millis());
  anemometerCurrentTimeDelta = millis() - anemometerLastPulse;
  anemometerLastPulse = millis();
}

void updateWindSpeed(void *parameters){
  while (1)
  {
    delay(WIND_SPEED_MEASUREMENT_INTERVAL);
    float currentWindSpeed = 1000.0 / anemometerCurrentTimeDelta * 2.4;
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
    windSpeedAverageIntervalCount = 0;
    windSpeedAverageIntervalSum = 0;
    windSpeedIntervalMax = 0;
  }
}

void updateTemperature(void *parameters){
  while (1)
  {
    delay(TEMPERATURE_MEASUREMENT_INTERVAL);
    temperatureBMEAverageIntervalSum += bme.readTemperature();
    temperatureRTDAverageIntervalSum += thermo.temperature(RNOMINAL, RREF);
    temperatureAverageIntervalCount++; 
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
    pressureAverageIntervalCount = 0;
    pressureAverageIntervalSum = 0;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(WIND_VANE_PIN,INPUT);
  Serial.begin(115200);
  while(!Serial); 

  Wire.begin(I2C_SDA, I2C_SCL);

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

}

void loop() {
  // put your main code here, to run repeatedly:
  //int wind_vane_value = analogRead(WIND_VANE_PIN);

}

