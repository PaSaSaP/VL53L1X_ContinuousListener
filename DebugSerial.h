#ifndef DEBUG_SERIAL_H_
#define DEBUG_SERIAL_H_

#define ENABLE_DEBUG_SERIAL 1

#if defined(ENABLE_DEBUG_SERIAL)
#define xSerialBegin(baud) Serial.begin(baud)
#define xDebug(...) Serial.print(__VA_ARGS__)
#define xDebugLn(...) Serial.print(__VA_ARGS__)
#define xFDebug(s) Serial.print(F(s))
#define xFDebugLn(s) Serial.print(F(s))
#else
#define xSerialBegin(baud)
#define xDebug(...)
#define xDebugLn(...)
#define xFDebug(s)
#define xFDebugLn(s)
#endif

#endif  // DEBUG_SERIAL_H_
