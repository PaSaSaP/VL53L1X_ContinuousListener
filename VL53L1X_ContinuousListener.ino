// SLAVE SPI
// stm8s103f3
// without serial (-DNO_SERIAL) and analog input/output (-DNO_ANALOG_IN -DNO_ANALOG_OUT)
// SPI NSS pin (alternate) need to be set by ST Visual Programmer
// STM32CubeProg does support only STM32 boards as I know...
#include <CRC16_c.h>
#include <I2C_tiny.h>
#include "SensorTypes.h"

// PD1 == SWIM, use or not use?
uint8_t const XshutPins[] = {PD4, PD5, PD6, PA1, PA2, PD3, PD2, PC4, PC3};

long currentTime = 0;
long sensorsCheckedTime = 0;
long updateOutputDataTime = 0;
bool keepSensorsDisabled = false;

void startSensorMeasurement(uint8_t index) {
  VL53L1X_setDistanceMode(&sensors[index].sensor);
  VL53L1X_setMeasurementTimingBudget(&sensors[index].sensor);
  VL53L1X_startContinuous(&sensors[index].sensor, 500);
}

bool initSensor(uint8_t index) {
  //  VL53L1X_setTimeout(&sensors[index].sensor, 500); // set as default value
  if (!VL53L1X_init(&sensors[index].sensor)) {
    return false;
  }
  VL53L1X_setAddress(&sensors[index].sensor, SENSOR_I2C_ADDRESS + 1 + index);
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
  VL53L1X_setAddressNoSend(&sensors[index].sensor, SENSOR_I2C_ADDRESS);
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
    sensors[i].status = SensorReadStatus_disabled;
    sensors[i].distance = 0;
  }
}

void reinitSensorOnProblem() {
  for (uint8_t i = 0; i < SensorCount; ++i) {
    if (sensors[i].sensor.last_status == 2
        && VL53L1X_getAddress(&sensors[i].sensor) == SENSOR_I2C_ADDRESS) {
      digitalWrite(sensors[i].xshutPin, HIGH);
      if (!initSensor(i)) {
        digitalWrite(sensors[i].xshutPin, LOW);
      }
    }
  }
}

void readDistanceFromSensors() {
  for (uint8_t i = 0; i < SensorCount; ++i) {
    sensors[i].distance = VL53L1X_readRangeContinuousMillimeters(&sensors[i].sensor);
    if (sensors[i].lastValidCnt < 0xFF) {
      sensors[i].lastValidCnt++;
    }
    if (VL53L1X_timeoutOccurred(&sensors[i].sensor)) {
      sensors[i].status = SensorReadStatus_timeout;
    } else if (sensors[i].sensor.last_status == 2) {
      sensors[i].status = SensorReadStatus_i2cError;
    } else if (sensors[i].sensor.last_status == 0 &&
               sensors[i].sensor.ranging_data.range_status == SignalFail &&
               sensors[i].sensor.ranging_data.peak_signal_count_rate_MCPS < 0x80) {
      sensors[i].status = SensorReadStatus_invalLowSignal;
    } else {
      sensors[i].lastValidCnt = 0;
      sensors[i].status = SensorReadStatus_ok;
    }
  }
}

void updateOutputData() {
  // CRC-16/AUG-CCITT (currently set directly in library)
  //  CRC16_setPolynome(0x1021);
  //  CRC16_setStartXOR(0x1D0F);
  CRC16_restart();
  uint8_t oidx = 1 - dataOutputPicker;
  for (uint8_t i = 0; i < SensorCount; ++i) {
    output[oidx][i].lastValidCnt = sensors[i].lastValidCnt;
    CRC16_add(sensors[i].lastValidCnt);
    output[oidx][i].status = sensors[i].status;
    CRC16_add((uint8_t)sensors[i].status);
    output[oidx][i].distance = sensors[i].distance;
    CRC16_add(GET_RIGHT_BYTE(sensors[i].distance));
    CRC16_add(GET_LEFT_BYTE(sensors[i].distance));
  }
  sensorDataCrc[oidx] = CRC16_getCRC();
  dataOutputPicker = oidx;
}

void initSpi() {
  /* Enable SPI clock, it makes mess for slave... */
  //  CLK_DeInit();
  //  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);

  // It matches for example SdFat settings (MSB, CPO, CPH)
  SPI_DeInit();
  SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_8, SPI_MODE_SLAVE,
           SPI_CLOCKPOLARITY_LOW, SPI_CLOCKPHASE_1EDGE,
           SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_HARD, (uint8_t)0x07);
  //  SPI_BiDirectionalLineConfig(SPI_DIRECTION_RX);
  SPI_ITConfig(SPI_IT_RXNE, ENABLE);
  //  SPI_ITConfig(SPI_IT_TXE, ENABLE); // Not needed? I think so
  //  ITC_SetSoftwarePriority(ITC_IRQ_SPI , ITC_PRIORITYLEVEL_3);
  SPI_Cmd(ENABLE);
}

void setup() {
  //  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV2); // 8 MHz

  I2C_begin();

  for (uint8_t i = 0; i < SensorCount; ++i) {
    sensors[i].xshutPin = XshutPins[i];
  }

  initSensors();
  initSpi();
  CRC16_reset();
}

void loop() {
#if 0
  // example of polling in main loop
  //   SPI polling
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)) {
    // maybe this line is not necessary, I didn't have the time to test without it yet
    while (SPI_GetFlagStatus(SPI_FLAG_BSY));
    uint8_t data = SPI_ReceiveData();
    while (SPI_GetFlagStatus(SPI_FLAG_TXE));
    SPI_SendData(data + 5);
    SPI->SR = 0;
  }
#endif

  currentTime = millis();
  // read data from sensors every ~500ms
  if (!keepSensorsDisabled && currentTime - sensorsCheckedTime >= 500) {
    sensorsCheckedTime = currentTime;

    readDistanceFromSensors();
    reinitSensorOnProblem();
    disableSensorsOnProblem();
  }
  
  // change output buffer
  if (requestDataOutputChange) {
    updateOutputData();
    requestDataOutputChange = false;
    updateOutputDataTime = currentTime;
  }

  // if master doesn't request data for 5 seconds then shut down sensors
  // maybe some sleep mode?
  if (currentTime - updateOutputDataTime > 5000) {
    if (!keepSensorsDisabled) {
      resetAndDisableSensors();
    }
    keepSensorsDisabled = true;
  } else {
    keepSensorsDisabled = false;
  }
}
