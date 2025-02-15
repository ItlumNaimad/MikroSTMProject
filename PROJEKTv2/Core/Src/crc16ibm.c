/*
 * crc16ibm.c
 *
 *  Created on: Dec 15, 2024
 *      Author: naima
 */

#include "crc16ibm.h"
uint8_t hexDigitToByte(uint8_t hex) {
	switch (hex){
		case '0' ... '9': return hex - '0';
		case 'A' ... 'F': return hex - 'A' + 10;
		//case 'a' ... 'f': return hex - 'a' + 10;
	}
	return 0xff;
}

// Już nie będzie potrzebna
// Funkcja do konwersji ciągu znaków na wartość bajtową (2 znaki)
uint8_t hexStringToByte(const uint8_t *hex) {
	uint8_t byte0 = hexDigitToByte(hex[0]);
	uint8_t byte1 = hexDigitToByte(hex[1]);
	uint8_t sumOfBytes = byte1 + (byte0 * 0x10);
	return sumOfBytes;

}
// Już nie będzie potrzebna
// Funkcja do konwersji ciągu znaków na wartość 16-bitową (4 znaki)
uint16_t hexStringToWord(const uint8_t *hex) {
	uint8_t byte0 = hexDigitToByte(hex[0]);
	uint8_t byte1 = hexDigitToByte(hex[1]);
	uint8_t byte2 = hexDigitToByte(hex[2]);
	uint8_t byte3 = hexDigitToByte(hex[3]);
	uint16_t sumOfBytes = byte3 + (byte2 * 0x10) + (byte1 * 0x100) + (byte0 * 0x1000);
	return sumOfBytes;
}

uint16_t calculateCRC(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF; // Inicjalizacja CRC z wartością startową
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR z danymi
        for (uint8_t j = 0; j < 8; j++) { // Przetwarzanie każdego bitu
            if (crc & 0x0001) { // Sprawdzenie LSB
                crc >>= 1; // Przesunięcie w prawo
                crc ^= 0xA001; // XOR z wielomianem
            } else {
                crc >>= 1; // Przesunięcie w prawo
            }
        }
    }
    return crc; // Zwrócenie obliczonego CRC
}

uint16_t takeCheckSumFrom(const Frame_raw_structure *frame) {
    uint16_t crc = calculateCRC(frame, sizeof * frame); // Oblicz CRC dla nagłówka
    return crc;
}

bool isChecksumValid(const Frame_raw_structure *frame) {
    uint16_t calculated_crc = takeCheckSumFrom(frame); // Oblicz CRC na podstawie danych w ramce (wraz z polem checksum)

    // Porównaj obliczone CRC z wartością otrzymaną w ramce
    return calculated_crc == frame->checksum;
}
