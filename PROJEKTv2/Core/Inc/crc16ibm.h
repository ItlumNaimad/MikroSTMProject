#ifndef INC_CRC16IBM_H_
#define INC_CRC16IBM_H_
#include "main.h"
#include "frame.h"
#include <stdbool.h>

uint16_t takeCheckSumFrom(const Frame_raw_structure *inputFrame);
uint16_t calculateCRC(uint8_t *data, uint16_t length);
uint8_t hexDigitToByte(uint8_t hex);
uint16_t hexStringToWord(const uint8_t *hex);
uint8_t hexStringToByte(const uint8_t *hex);
bool isChecksumValid(const Frame_raw_structure *frame);
#endif /* INC_CRC16IBM_H_ */
