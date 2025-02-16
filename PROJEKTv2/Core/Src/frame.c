#include "main.h"
#include "frame.h"
#include "crc16ibm.h"  // Używamy funkcji hexDigitToByte, hexStringToByte, hexStringToWord z tego modułu
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define FRAME_START '^' // ASCII 94
#define FRAME_END   '#' // ASCII 35

/* Definicje długości poszczególnych pól (surowe, jako ciągi hex) */
#define LEN_SENDER        2
#define LEN_RECEIVER      2
#define LEN_DATALENGTH    2  // 1 bajt = 2 znaki hex
#define LEN_CHECKSUM      4  // 2 bajty = 4 znaki hex

/* Stan maszyny odbioru */
typedef enum {
    FRAME_WAIT_START,
    FRAME_READ_SENDER,
    FRAME_READ_RECEIVER,
    FRAME_READ_DATALENGTH,
    FRAME_READ_PAYLOAD,    // odczytujemy payload – długość = data_length * 2 znaki
    FRAME_READ_CHECKSUM,
    FRAME_READ_END,        // Oczekujemy znaku FRAME_END
    FRAME_COMPLETE
} FrameState;

/* Globalne zmienne */
static FrameState current_state = FRAME_WAIT_START;
static uint16_t field_index = 0;  // licznik znaków w bieżącym polu
static uint16_t expected_payload_hex_length = 0; // (data_length bajtów * 2 znaków)
static Frame_raw_structure raw_frame;   // Bufor surowej ramki
static Frame_structure parsed_frame;    // Przetworzona ramka

extern UART_HandleTypeDef huart2;

/* Funkcja konwersji ciągu hex (o podanej długości) na 32-bitową wartość */
static uint32_t hexStringToDword(uint8_t *str, uint8_t len) {
    uint32_t value = 0;
    for(uint8_t i = 0; i < len; i++){
        uint8_t nibble = hexDigitToByte(str[i]);
        if(nibble == 0xFF)
            return 0; // lub obsłużyć błąd
        value = (value << 4) | nibble;
    }
    return value;
}

/* Funkcja przetwarzająca odebrane pole.
 * W zależności od bieżącego stanu konwertujemy dane z formatu hex do wartości binarnych.
 */
static void processReceivedField(FrameState state, uint8_t *buffer, uint16_t len) {
    switch(state) {
        case FRAME_READ_SENDER:
            memcpy(raw_frame.sender, buffer, LEN_SENDER);
            parsed_frame.sender = hexStringToByte(buffer);
            break;
        case FRAME_READ_RECEIVER:
            memcpy(raw_frame.receiver, buffer, LEN_RECEIVER);
            parsed_frame.receiver = hexStringToByte(buffer);
            break;
        case FRAME_READ_DATALENGTH: {
            memcpy(raw_frame.data_length, buffer, LEN_DATALENGTH);
            uint8_t dl = hexStringToByte(buffer);
            parsed_frame.data_length = dl;
            expected_payload_hex_length = dl * 2;  // Każdy bajt to 2 znaki
            break;
        }
        case FRAME_READ_PAYLOAD:
            memcpy(raw_frame.payload, buffer, expected_payload_hex_length);
            /* Kopiujemy payload do parsed_frame.payload.
             * Interpretacja payloadu: zawsze pierwszy bajt to kod komendy.
             * Jeśli parsed_frame.data_length > 10, oznacza to, że oprócz 1 bajtu kodu i 8 bajtów argumentu,
             * pojawia się dodatkowo pole archiwum (np. 3 bajty, lub więcej).
             * W tej implementacji przekazujemy całość, a dalsza interpretacja pozostaje w gestii aplikacji.
             */
            for(uint16_t i = 0; i < (expected_payload_hex_length / 2); i++){
                // Przetwarzamy pary znaków, by uzyskać pojedynczy bajt payloadu
                parsed_frame.payload[i] = hexStringToByte(&buffer[i*2]);
            }
            break;
        case FRAME_READ_CHECKSUM:
            memcpy(raw_frame.checksum, buffer, LEN_CHECKSUM);
            parsed_frame.checksum = hexStringToWord(buffer);
            break;
        default:
            break;
    }
}

/* Funkcja obliczająca sumę kontrolną na podstawie pól ramki (od sender do końca payloadu).
 * Tutaj stosujemy prosty algorytm – sumujemy wartości bajtów.
 */
static uint16_t calculateParsedChecksum(Frame_structure *frame) {
    uint16_t sum = 0;
    sum += frame->sender;
    sum += frame->receiver;
    sum += frame->data_length;
    for(uint8_t i = 0; i < frame->data_length; i++){
        sum += frame->payload[i];
    }
    return sum;
}

static uint8_t isParsedChecksumValid(Frame_structure *frame) {
    return (calculateParsedChecksum(frame) == frame->checksum);
}

/* Funkcja odbioru ramki – przetwarzanie bajt po bajcie */
void ReceiveFrame(void) {
    uint8_t RX_byte;
    while(UART_RXisNotEmpty()) {  // Zakładamy, że funkcja UART_RXisNotEmpty() sprawdza, czy są dane
        RX_byte = USART_getchar();

        /* Jeśli otrzymamy znak startu, resetujemy stan */
        if(RX_byte == FRAME_START) {
            current_state = FRAME_READ_SENDER;
            field_index = 0;
            expected_payload_hex_length = 0;
            memset(&raw_frame, 0, sizeof(raw_frame));
            continue;
        }

        if(current_state == FRAME_WAIT_START)
            continue;

        /* Jeśli pojawi się znak FRAME_END przedwcześnie, resetujemy odbiór */
        if(RX_byte == FRAME_END && current_state != FRAME_READ_END) {
            current_state = FRAME_WAIT_START;
            continue;
        }

        switch(current_state) {
            case FRAME_READ_SENDER: {
                static uint8_t temp_buf[LEN_SENDER];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_SENDER) {
                    processReceivedField(FRAME_READ_SENDER, temp_buf, LEN_SENDER);
                    field_index = 0;
                    current_state = FRAME_READ_RECEIVER;
                }
                break;
            }
            case FRAME_READ_RECEIVER: {
                static uint8_t temp_buf[LEN_RECEIVER];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_RECEIVER) {
                    processReceivedField(FRAME_READ_RECEIVER, temp_buf, LEN_RECEIVER);
                    field_index = 0;
                    current_state = FRAME_READ_DATALENGTH;
                }
                break;
            }
            case FRAME_READ_DATALENGTH: {
                static uint8_t temp_buf[LEN_DATALENGTH];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_DATALENGTH) {
                    processReceivedField(FRAME_READ_DATALENGTH, temp_buf, LEN_DATALENGTH);
                    field_index = 0;
                    current_state = FRAME_READ_PAYLOAD;
                }
                break;
            }
            case FRAME_READ_PAYLOAD: {
                static uint8_t temp_buf[28]; // maksymalna liczba znaków payload
                temp_buf[field_index++] = RX_byte;
                if(field_index >= expected_payload_hex_length) {
                    processReceivedField(FRAME_READ_PAYLOAD, temp_buf, expected_payload_hex_length);
                    field_index = 0;
                    current_state = FRAME_READ_CHECKSUM;
                }
                break;
            }
            case FRAME_READ_CHECKSUM: {
                static uint8_t temp_buf[LEN_CHECKSUM];
                temp_buf[field_index++] = RX_byte;
                if(field_index >= LEN_CHECKSUM) {
                    processReceivedField(FRAME_READ_CHECKSUM, temp_buf, LEN_CHECKSUM);
                    field_index = 0;
                    current_state = FRAME_READ_END;
                }
                break;
            }
            case FRAME_READ_END: {
                /* Oczekujemy, aż otrzymamy znak FRAME_END */
                if(RX_byte == FRAME_END) {
                    current_state = FRAME_COMPLETE;
                } else {
                    current_state = FRAME_WAIT_START;
                }
                break;
            }
            default:
                break;
        }

        if(current_state == FRAME_COMPLETE) {
            if(isParsedChecksumValid(&parsed_frame)) {
                /* Tutaj interpretujemy odebrany payload.
                   Jeśli parsed_frame.data_length <= 10 – payload zawiera tylko komendę i argument.
                   Jeśli > 10 – payload zawiera dodatkowo pole archiwum (od bajtu 10 wzwyż).
                */
                char response[64];
                sprintf(response, "Cmd: %02X DL: %02X Payload:", parsed_frame.payload[0], parsed_frame.data_length);
                HAL_UART_Transmit_IT(&huart2, (uint8_t *)response, strlen(response));
                memset(response, 0, sizeof(response));
                for(uint8_t i = 0; i < parsed_frame.data_length; i++){
                    sprintf(response, " %02X", parsed_frame.payload[i]);
                    HAL_UART_Transmit_IT(&huart2, (uint8_t *)response, strlen(response));
                    memset(response, 0, sizeof(response));
                }
                sprintf(response, " CRC: %04X\r\n", parsed_frame.checksum);
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
