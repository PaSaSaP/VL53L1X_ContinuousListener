// MASTER SPI
// arduino pro mini as master
#include <CRC16.h>
#include <SPI.h>

#define XDEBUG_SERIAL Serial
#define xSerialBegin(v) XDEBUG_SERIAL.begin(v)
#define xSerial(...) XDEBUG_SERIAL.print(__VA_ARGS__)
#define xSerialLn(...) XDEBUG_SERIAL.println(__VA_ARGS__)
#define xFSerial(v) XDEBUG_SERIAL.print(F(v))
#define xFSerialLn(v) XDEBUG_SERIAL.println(F(v))
#define xTrace(v) xFSerial(v)
#define xTraceLn(v) xFSerialLn(v)
#define S(v)      \
  xFSerial(#v);   \
  xFSerial(": "); \
  xSerialLn(v)

const int HeightSensorsCount = 9;

enum class SensorReadStatus : uint8_t {
  ok,
  timeout,
  i2cError,
  invalLowSignal,
};

struct SensorData {
  uint8_t lastValidCnt;
  SensorReadStatus status;
  uint16_t distance;
} sensors[HeightSensorsCount];

CRC16 expectedCrc;
uint16_t actualCrc;

void readSensorsData() {
  expectedCrc.restart();
  actualCrc = 0;
  uint8_t r{};
  uint8_t* sensorBuff = reinterpret_cast<uint8_t*>(sensors);
  SPI.transfer(0x01);
  for (size_t i = 0x02; i <= sizeof(sensors) + 3; ++i) {
    // Arduino as slave needed that delay to work in 2MHz and with that rather slow speed there were errors in 5% rate
    // with low quality wire but!
    // STM8S as slave with hardware interrupt worked without below delay at 8MHz (SPISettings(8000000, ...)) with 0% error rate
    // and STM8S does not have I2C pins in random location on board...
    // update - first test was with pro mini 8MHz as master so it was 4MHz, not 8MHz and delay
    // was not needed here;
    // Now tested with STM32F401 72MHz and there is 4us delay with 9MHz SPI clock and CRC is OK;
    // 3us delay makes CRC sometimes does not pass
    // delayMicroseconds(10);
    r = SPI.transfer(i);
    if (i <= sizeof(sensors) + 1) {
      sensorBuff[i - 2] = r;
      expectedCrc.add(r);
    } else if (i == sizeof(sensors) + 2) {
      actualCrc |= r;
    } else if (i == sizeof(sensors) + 3) {
      actualCrc |= r << 8;
    }
  }
}

void receive() {
  SPI.beginTransaction(SPISettings(8000000UL, MSBFIRST, SPI_MODE0));

  digitalWrite(SS, LOW);
  memset(sensors, 0, sizeof(sensors));

  readSensorsData();

  uint8_t* buf = (uint8_t*)sensors;
  Serial.print(expectedCrc.getCRC(), HEX); Serial.print("/"); Serial.print(actualCrc, HEX);
  Serial.print(" ;");
  Serial.print(bool(expectedCrc.getCRC() == actualCrc));
  Serial.print(") ");
  for (int i = 0; i < 24; ++i) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  digitalWrite(SS, HIGH);
  SPI.endTransaction();
   for (size_t i = 0; i < HeightSensorsCount; ++i) {
     Serial.print(i); Serial.print(") ");
     Serial.print("LVCnt="); Serial.print(sensors[i].lastValidCnt);
     Serial.print("; St="); Serial.print((int)sensors[i].status);
     Serial.print("; d="); Serial.println(sensors[i].distance);
   }
}

uint8_t buf[32];

// to test dummy data, in slave return (request+5)
void strReceive() {
  SPI.beginTransaction(SPISettings(8000000UL, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);
  SPI.transfer(0x01);
  uint8_t r;
  int valid = 0;
  for (uint8_t i = 2; i <= 25; ++i) {
    // delayMicroseconds(10);
    r = SPI.transfer(i);
    buf[i - 2] = r;
    if (r == i + 4) {
      valid++;
      // Serial.print("VALID=");Serial.println(i);
    }
  }
  Serial.print(";");
  Serial.print(valid);
  Serial.print("/24) ");
  for (int i = 0; i < 24; ++i) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  delay(500);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
}

void setup() {
  SPI.begin();
  Serial.begin(57600);

  expectedCrc.setPolynome(0x1021);
  expectedCrc.setStartXOR(0x1D0F);
}

void loop() {
  receive();
  delay(500);
}
