#include "main.h"
#include "frame.h"
#include "crc16ibm.h"  // Używamy funkcji z tego modułu
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define FRAME_START '^' // ASCII 94
#define FRAME_END   '#' // ASCII 35

/* Długości poszczególnych pól (w surowej postaci – ciągi hex) */
#define LEN_SENDER        2
#define LEN_RECEIVER      2
#define LEN_FUNCTION      2
#define LEN_ARGUMENT      8   // 8 znaków dla 32-bitowego argumentu
#define LEN_ARCHIVE_INDEX 4
#define LEN_CHECKSUM      4

/* Stan odbioru poszczególnych pól */
typedef enum {
    FRAME_WAIT_START,
    FRAME_READ_SENDER,
    FRAME_READ_RECEIVER,
    FRAME_READ_FUNCTION,
    FRAME_READ_ARGUMENT,
    FRAME_READ_ARCHIVE,
    FRAME_READ_CHECKSUM,
    FRAME_COMPLETE
} FrameState;

/* Globalne zmienne dla odbioru ramki */
static FrameState current_state = FRAME_WAIT_START;
static uint8_t field_index = 0;  // Licznik znaków dla bieżącego pola
static Frame_raw_structure raw_frame;  // Bufor surowych danych (hex)
static Frame_structure parsed_frame;   // Struktura z przetworzonymi danymi
extern UART_HandleTypeDef huart2;

/* Funkcja konwersji 8 znaków hex na 32-bitową wartość.
   Używa funkcji hexDigitToByte() z crc16ibm.h */
static uint32_t hexStringToDword(uint8_t *str, uint8_t len) {
    uint32_t value = 0;
    for(uint8_t i = 0; i < len; i++){
        uint8_t nibble = hexDigitToByte(str[i]);
        if(nibble == 0xFF)
            return 0;  // lub obsłużyć błąd
        value = (value << 4) | nibble;
    }
    return value;
}

/* Funkcja przetwarzająca odebrane pole – w zależności od bieżącego stanu */
static void processReceivedField(FrameState state, uint8_t *buffer) {
    switch(state) {
        case FRAME_READ_SENDER:
            parsed_frame.sender = hexStringToByte(buffer); // używamy funkcji z crc16ibm.h
            break;
        case FRAME_READ_RECEIVER:
            parsed_frame.receiver = hexStringToByte(buffer);
            break;
        case FRAME_READ_FUNCTION:
            parsed_frame.function = hexStringToByte(buffer);
            break;
        case FRAME_READ_ARGUMENT:
            parsed_frame.argument = hexStringToDword(buffer, LEN_ARGUMENT);
            break;
        case FRAME_READ_ARCHIVE:
            parsed_frame.archive_index = hexStringToWord(buffer); // funkcja z crc16ibm.h
            break;
        case FRAME_READ_CHECKSUM:
            parsed_frame.checksum = hexStringToWord(buffer);
            break;
        default:
            break;
    }
}

/* Funkcja obliczająca sumę kontrolną na podstawie pól ramki.
   Tu obliczamy sumę jako prostą sumę (mod 0x10000) wszystkich pól od sender do archive_index. */
static uint16_t calculateParsedChecksum(Frame_structure *frame) {
    uint16_t sum = 0;
    sum += frame->sender;
    sum += frame->receiver;
    sum += frame->function;
    sum += (uint16_t)((frame->argument >> 16) & 0xFFFF);
    sum += (uint16_t)(frame->argument & 0xFFFF);
    sum += frame->archive_index;
    return sum;
}

static uint8_t isParsedChecksumValid(Frame_structure *frame) {
    return (calculateParsedChecksum(frame) == frame->checksum);
}

/* Funkcja odbioru ramki – przetwarzanie bajt po bajcie */
void ReceiveFrame(void) {
    uint8_t RX_byte;
    while(UART_RXisNotEmpty()) { // Zakładamy, że te funkcje są dostępne
        RX_byte = USART_getchar();

        /* Reset, gdy pojawi się znak startu */
        if(RX_byte == FRAME_START) {
            current_state = FRAME_READ_SENDER;
            field_index = 0;
            memset(&raw_frame, 0, sizeof(raw_frame));
            continue;
        }

        if(current_state == FRAME_WAIT_START)
            continue;

        /* Jeśli pojawi się znak końca w nieodpowiednim miejscu, resetujemy stan */
        if(RX_byte == FRAME_END) {
            current_state = FRAME_WAIT_START;
            continue;
        }

        switch(current_state) {
            case FRAME_READ_SENDER: {
                static uint8_t temp_buf[LEN_SENDER];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_SENDER) {
                    processReceivedField(FRAME_READ_SENDER, temp_buf);
                    field_index = 0;
                    current_state = FRAME_READ_RECEIVER;
                }
                break;
            }
            case FRAME_READ_RECEIVER: {
                static uint8_t temp_buf[LEN_RECEIVER];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_RECEIVER) {
                    processReceivedField(FRAME_READ_RECEIVER, temp_buf);
                    field_index = 0;
                    current_state = FRAME_READ_FUNCTION;
                }
                break;
            }
            case FRAME_READ_FUNCTION: {
                static uint8_t temp_buf[LEN_FUNCTION];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_FUNCTION) {
                    processReceivedField(FRAME_READ_FUNCTION, temp_buf);
                    field_index = 0;
                    current_state = FRAME_READ_ARGUMENT;
                }
                break;
            }
            case FRAME_READ_ARGUMENT: {
                static uint8_t temp_buf[LEN_ARGUMENT];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_ARGUMENT) {
                    processReceivedField(FRAME_READ_ARGUMENT, temp_buf);
                    field_index = 0;
                    current_state = FRAME_READ_ARCHIVE;
                }
                break;
            }
            case FRAME_READ_ARCHIVE: {
                static uint8_t temp_buf[LEN_ARCHIVE_INDEX];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_ARCHIVE_INDEX) {
                    processReceivedField(FRAME_READ_ARCHIVE, temp_buf);
                    field_index = 0;
                    current_state = FRAME_READ_CHECKSUM;
                }
                break;
            }
            case FRAME_READ_CHECKSUM: {
                static uint8_t temp_buf[LEN_CHECKSUM];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_CHECKSUM) {
                    processReceivedField(FRAME_READ_CHECKSUM, temp_buf);
                    field_index = 0;
                    current_state = FRAME_COMPLETE;
                }
                break;
            }
            default:
                break;
        }

        if(current_state == FRAME_COMPLETE) {
            if(isParsedChecksumValid(&parsed_frame)) {
                uint8_t command = parsed_frame.function;
                uint32_t arg = parsed_frame.argument;
                uint16_t arch_index = parsed_frame.archive_index;
                // Przykładowa obsługa komend:
                // if (command == 0x01) { ustaw interwał czasowy = arg; }
                // else if (command == 0x03) { odczytaj archiwalny wpis o indeksie arch_index; }

                char response[32];
                sprintf(response, "Cmd: %02X Arg: %08X Arch: %04X\r\n", command, arg, arch_index);
                HAL_UART_Transmit_IT(&huart2, (uint8_t *)response, strlen(response));
            } else {
                HAL_UART_Transmit_IT(&huart2, (uint8_t *)"Checksum error\r\n", 17);
            }
            current_state = FRAME_WAIT_START;
        }
    }
}


/*
 * POZOSTAŁOŚCI PO PRACY Z BIBLIOTEKĄ OD PRODUCENTA
// Czesc funkcji od czujnika
void init_bme280_dev() {
    dev.intf = BME280_I2C_INTF;  // Wybór interfejsu I2C
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;

    // Adres urządzenia (np. 0x76)
    static uint8_t i2c_address = BME280_I2C_ADDR_PRIM;
    dev.intf_ptr = &i2c_address;
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;  // Adres I2C urządzenia
    HAL_StatusTypeDef status;

    // Wyślij adres rejestru do odczytu
    status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, &reg_addr, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1;  // Błąd transmisji
    }

    // Odczytaj dane z urządzenia
    status = HAL_I2C_Master_Receive(&hi2c1, dev_addr << 1, data, len, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1;  // Błąd odczytu
    }

    return 0;  // Sukces
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;  // Adres I²C urządzenia
    HAL_StatusTypeDef status;

    // Bufor z adresem rejestru i danymi do zapisu
    uint8_t buf[len + 1];
    buf[0] = reg_addr;  // Pierwszy bajt to adres rejestru
    memcpy(&buf[1], data, len);  // Kopiuj dane do bufora

    // Wyślij dane do urządzenia
    status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, buf, len + 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1;  // Błąd transmisji
    }

    return 0;  // Sukces
}

uint8_t user_delay_us(uint32_t period, void *intf_ptr) {
	unit32_t temporary_ticks = 0;
    if(SysTick_Ticks - temporary_ticks >= period)  // Jeśli Ticki beda wieksze od wyznaczonej wartosci
    {
    	temporary_ticks = SysTick_Ticks;
    	return 1;
    }
    else
    	return 0;
}
*/

/*
 * KOMENDY:
 * 06 00 00 00 01 - temperatura
 * 06 00 00 00 02 - ciśnienie
 * 06 00 00 00 03 - wilgotność
 * 01 00 (wartość) - interwał czasowy
 * 02 00 00 00 00 - aktualny interwał czasowy
 * 03 00 00 00 00 - ostatni zapisany odczyt archiwalny
 * 04 00 (wartość) - odczyt konkretnej wartości archiwalnej
 * 05 00 00 00 00 - kasowanie archiwalnych wartości
 */
