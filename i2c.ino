
//void i2cScanner() {
//  byte error, address; //variable for error and I2C address
//  int nDevices;
//
//  Serial.println(F("Scanning..."));
//
//  nDevices = 0;
//  for (address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    error = Wire.endTransmission();
//
//    if (error == 0)
//    {
//      Serial.print(F("I2C device found at address 0x"));
//      if (address < 16)
//        Serial.print("0");
//      Serial.print(address, HEX);
//      Serial.println("  !");
//      nDevices++;
//    }
//    else if (error == 4)
//    {
//      Serial.print(F("Unknown error at address 0x"));
//      if (address < 16)
//        Serial.print("0");
//      Serial.println(address, HEX);
//    }
//  }
//  if (nDevices == 0)
//    Serial.println(F("No I2C devices found\n"));
//  else
//    Serial.println(F("done\n"));
//}
