uint8_t request;

INTERRUPT_HANDLER(SPI_IRQHandler, 10) {
  request = SPI->DR - 1;  // read byte from SPI Data Register
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
          request = (uint8_t)output[currentDataOutput][request / 4].status;
          break;
        }
      case 2: {
          request = GET_RIGHT_BYTE(output[currentDataOutput][request / 4].distance);
          break;
        }
      case 3: {
          request = GET_LEFT_BYTE(output[currentDataOutput][request / 4].distance);
          break;
        }
    }  // switch
  } else if (request < sizeof(output[0]) + 1) {
    request = GET_RIGHT_BYTE(sensorDataCrc[currentDataOutput]);
  } else if (request < sizeof(output[0]) + 2) {
    request = GET_LEFT_BYTE(sensorDataCrc[currentDataOutput]);
  } else {
    request = 0; // some invalid request
  }
  SPI->DR = request;
}
