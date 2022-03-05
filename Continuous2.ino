// SLAVE SPI
// Arduino PRO MINI 3V3 8MHz
#include <Arduino.h>
#include <CRC16.h>
#include <SPI.h>
#include <Wire.h>

#include "DebugSerial.h"
#include "SensorTypes.h"

uint8_t const XshutPins[] = {2, 3, 4, 5, 6, 7, 8, 9};

long currentTime = 0;
long sensorsCheckedTime = 0;
long updateOutputDataTime = 0;
bool keepSensorsDisabled = false;

void startSensorMeasurement(uint8_t index) {
  sensors[index].sensor.setDistanceMode(VL53L1X::Short);
  sensors[index].sensor.setMeasurementTimingBudget(500000);
  sensors[index].sensor.startContinuous(500);
}

bool initSensor(uint8_t index) {
  sensors[index].sensor.setTimeout(500);
  if (!sensors[index].sensor.init()) {
    xFDebug("Fail init sensor ");
    xDebugLn(index);
    return false;
  }
  sensors[index].sensor.setAddress(SENSOR_I2C_ADDRESS + 1 + index);

  startSensorMeasurement(index);
  return true;
}

void initSensors() {
  for (uint8_t i = 0; i < SensorCount; ++i) {
    pinMode(sensors[i].xshutPin, OUTPUT);
    digitalWrite(sensors[i].xshutPin, LOW);
  }
  for (uint8_t i = 0; i < SensorCount; ++i) {
    digitalWrite(sensors[i].xshutPin, HIGH);
    if (!initSensor(i)) {
      digitalWrite(sensors[i].xshutPin, LOW);
    }
  }
}

void disableSensor(uint8_t index) {
  digitalWrite(sensors[index].xshutPin, LOW);
  sensors[index].sensor.setAddressNoSend(SENSOR_I2C_ADDRESS);
  sensors[index].lastValidCnt = 0;
}

void disableSensorsOnProblem() {
  for (uint8_t i = 0; i < SensorCount; ++i) {
    if (sensors[i].sensor.last_status == 2) {
      disableSensor(i);
    }
  }
}

void resetAndDisableSensors() {
  for (uint8_t i = 0; i < SensorCount; ++i) {
    disableSensor(i);
    sensors[i].sensor.last_status = 2;
    sensors[i].status = SensorReadStatus::disabled;
    sensors[i].distance = 0;
  }
}

void reinitSensorOnProblem() {
  for (uint8_t i = 0; i < SensorCount; ++i) {
    if (sensors[i].sensor.last_status == 2 && sensors[i].sensor.getAddress() == SENSOR_I2C_ADDRESS) {
      digitalWrite(sensors[i].xshutPin, HIGH);
      if (!initSensor(i)) {
        digitalWrite(sensors[i].xshutPin, LOW);
      }
    }
  }
}

void readDistanceFromSensors() {
  for (uint8_t i = 0; i < SensorCount; ++i) {
    sensors[i].distance = sensors[i].sensor.readRangeContinuousMillimeters(false);
    if (sensors[i].lastValidCnt < 0xFF) {
      sensors[i].lastValidCnt++;
    }
    if (sensors[i].sensor.timeoutOccurred()) {
      sensors[i].status = SensorReadStatus::timeout;
    } else if (sensors[i].sensor.last_status == 2) {
      sensors[i].status = SensorReadStatus::i2cError;
    } else if (sensors[i].sensor.last_status == 0 &&
               sensors[i].sensor.ranging_data.range_status == VL53L1X::SignalFail &&
               sensors[i].sensor.ranging_data.peak_signal_count_rate_MCPS < 1.0) {
      sensors[i].status = SensorReadStatus::invalLowSignal;
    } else {
      sensors[i].lastValidCnt = 0;
      sensors[i].status = SensorReadStatus::ok;
    }
  }
}

void updateOutputData() {
  CRC16 crc;
  crc.setPolynome(0x8005);
  crc.setStartXOR(0xFFFF);
  uint8_t oidx = 1 - dataOutputPicker;
  for (uint8_t i = 0; i < SensorCount; ++i) {
    output[oidx][i].lastValidCnt = sensors[i].lastValidCnt;
    crc.add(sensors[i].lastValidCnt);
    output[oidx][i].status = sensors[i].status;
    crc.add(static_cast<uint8_t>(sensors[i].status));
    output[oidx][i].distance = sensors[i].distance;
    crc.add(GET_LEFT_BYTE(sensors[i].distance));
    crc.add(GET_RIGHT_BYTE(sensors[i].distance));
  }
  sensorDataCrc[oidx] = crc.getCRC();
  dataOutputPicker = oidx;
}

void serialPrintSensorsDistance() {
#if defined(ENABLE_DEBUG_SERIAL)
  for (uint8_t i = 0; i < SensorCount; ++i) {
    xDebug('\t');
    xDebug(sensors[i].distance);
  }
  xDebugLn("");

  auto& sensor = sensors[0].sensor;
  xFDebug("r=");
  xDebug(sensor.ranging_data.range_mm);
  xFDebug("\tst=");
  xDebug(sensor.ranging_data.range_status);
  xFDebug("\tpeak=");
  xDebug(sensor.ranging_data.peak_signal_count_rate_MCPS);
  xFDebug("\tambient=");
  xDebug(sensor.ranging_data.ambient_count_rate_MCPS);
  xFDebug("\tistatus=");
  xDebug(sensor.last_status);

  xDebugLn("");
#endif
}

void initSpi() {
  pinMode(MISO, OUTPUT);  // have to send on master in so it set as output
  SPCR |= _BV(SPE);       // turn on SPI in slave mode
  //  SPI.setClockDivider(SPI_CLOCK_DIV128);
  //    SPI.setDataMode(SPI_MODE0);
  SPI.attachInterrupt();  // turn on interrupt
  pinMode(SS, INPUT_PULLUP);
}

void setup() {
  xSerialBegin(115200);
  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C

  for (uint8_t i = 0; i < SensorCount; ++i) {
    sensors[i].xshutPin = XshutPins[i];
  }

  initSensors();
  initSpi();
}

void loop() {
  currentTime = millis();
  if (!keepSensorsDisabled && currentTime - sensorsCheckedTime >= 500) {
    sensorsCheckedTime = currentTime;

    readDistanceFromSensors();
    serialPrintSensorsDistance();
    reinitSensorOnProblem();
    disableSensorsOnProblem();
  }
  if (requestDataOutputChange) {
    updateOutputData();
    requestDataOutputChange = false;
    updateOutputDataTime = currentTime;
  }
  if (currentTime - updateOutputDataTime > 5000) {
    if (!keepSensorsDisabled) {
      resetAndDisableSensors();
    }
    keepSensorsDisabled = true;
  } else {
    keepSensorsDisabled = false;
  }
}

ISR(SPI_STC_vect) {        // SPI interrupt routine
  uint8_t request = SPDR;  // read byte from SPI Data Register
  request--;
  if (request < sizeof(output[0])) {
    if (request == 0) {
      currentDataOutput = dataOutputPicker;
      requestDataOutputChange = true;
    }
    switch (request % 4) {
      case 0: {
        request = output[currentDataOutput][request / 4].lastValidCnt;
        break;
      }
      case 1: {
        request = static_cast<uint8_t>(output[currentDataOutput][request / 4].status);
        break;
      }
      case 2: {
        request = GET_LEFT_BYTE(output[currentDataOutput][request / 4].distance);
        break;
      }
      case 3: {
        request = GET_RIGHT_BYTE(output[currentDataOutput][request / 4].distance);
        break;
      }
    }  // switch
  } else if (request < sizeof(output[0]) + 1) {
    request = GET_LEFT_BYTE(sensorDataCrc[currentDataOutput]);
  } else if (request < sizeof(output[0]) + 2) {
    request = GET_RIGHT_BYTE(sensorDataCrc[currentDataOutput]);
  } else {
    request = 0;
  }
  SPDR = request;
}
