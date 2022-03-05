#ifndef SENSOR_TYPES_H_
#define SENSOR_TYPES_H_

#include <VL53L1X.h>
#define SENSOR_I2C_ADDRESS 0x29

enum class SensorReadStatus : uint8_t {
  disabled,
  ok,
  timeout,
  i2cError,
  invalLowSignal,
};

struct SensorData {
  VL53L1X sensor;
  uint8_t xshutPin = 0;
  uint8_t lastValidCnt = 0;
  SensorReadStatus status = SensorReadStatus::disabled;
  uint16_t distance = 0;
};

struct DataOutput {
  uint8_t lastValidCnt = 0;
  SensorReadStatus status = SensorReadStatus::disabled;
  uint16_t distance = 0;
};

const uint8_t SensorCount = 6;
SensorData sensors[SensorCount];
DataOutput output[2][SensorCount];
uint16_t sensorDataCrc[2] = {0x5EEF, 0x5EEF};
volatile bool requestDataOutputChange = 0;
volatile uint8_t currentDataOutput = 0;
uint8_t dataOutputPicker = 0;

#define GET_LEFT_BYTE(v) ((v >> 8) & 0xFF)
#define GET_RIGHT_BYTE(v) (v & 0xFF)

#endif  // SENSOR_TYPES_H_
