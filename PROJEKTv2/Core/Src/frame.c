#include "main.h"
#include "frame.h"
#include "crc16ibm.h"  // Używamy funkcji hexDigitToByte, hexStringToByte, hexStringToWord oraz calculateCRC z tego modułu
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define FRAME_START '^' // ASCII 94
#define FRAME_END   '#' // ASCII 35

/* Definicje długości poszczególnych pól (w surowej postaci – ciągi hex) */
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
    FRAME_READ_PAYLOAD,    // Odczytujemy payload – długość = data_length * 2 znaków
    FRAME_READ_CHECKSUM,
    FRAME_READ_END,        // Oczekujemy znaku FRAME_END
    FRAME_COMPLETE
} FrameState;

/* Globalne zmienne */
static FrameState current_state = FRAME_WAIT_START;
static uint16_t field_index = 0;  // Licznik znaków w bieżącym polu
static uint16_t expected_payload_hex_length = 0; // (data_length bajtów * 2 znaków)
static Frame_raw_structure raw_frame;   // Bufor surowej ramki (w formie hex)
static Frame_structure parsed_frame;    // Przetworzona ramka (wartości binarne)

extern UART_HandleTypeDef huart2;

void ProcessCommand(Frame_structure *cmd_frame) {
    if(cmd_frame == NULL) return;

    uint8_t command = cmd_frame->payload[0];
    // Parametr może mieć długość 4 bajtów, jeśli występuje; kopiujemy go do zmiennej.
    uint32_t param = 0;
    if(cmd_frame->data_length > 1) {
        // Parametr jest 4 bajtowy, interpretujemy jako liczbę z systemu big-endian
        param = (cmd_frame->payload[1] << 24) | (cmd_frame->payload[2] << 16) |
                (cmd_frame->payload[3] << 8)  | (cmd_frame->payload[4]);
    }

    switch(command) {
        case 0x06: // Komenda GET: pobierz pomiar
            if(param == 0x00000001) { // GET temperature
            {
                float temp = 0;
                GetLatestData(&temp, NULL, NULL);
                // Odpowiedź: kod 0x16, parametr to temperatura (przemnożona przez 100)
                SendResponseFrame(0x16, (uint8_t*)&temp, sizeof(temp)); // Możesz dostosować format
            }
            break;
            }
        case 0x01: // Ustaw interwał
            if(cmd_frame->data_length != 5) { // powinien być 1 bajt komendy + 4 bajty parametru
                SendErrorResponseFrame(0x30); // Błąd formatu
                return;
            }
            if(param < 0x00000001 || param > 0x00002710) { // 1 do 10000 ms
                SendErrorResponseFrame(0x21); // Błąd interwału
                return;
            }
            if(SetInterval(param) != 0) {
                SendErrorResponseFrame(0x21);
                return;
            }
            // Odpowiedź: kod 0x11, parametr to ustawiony interwał (np. 2 bajty – tu przekazujemy pełny 32-bit)
            {
                uint32_t current_int = GetCurrentInterval();
                SendResponseFrame(0x11, (uint8_t*)&current_int, 4);
            }
            break;
        case 0x02: // Podaj aktualny interwał – nie może mieć parametru
            if(cmd_frame->data_length != 1) {
                SendErrorResponseFrame(0x30);
                return;
            }
            {
                uint32_t current_int = GetCurrentInterval();
                SendResponseFrame(0x11, (uint8_t*)&current_int, 4);
            }
            break;
        case 0x03: // Podaj indeks ostatniego odczytu archiwalnego – nie może mieć parametru
            if(cmd_frame->data_length != 1) {
                SendErrorResponseFrame(0x30);
                return;
            }
            {
                uint32_t last_idx = GetLastArchiveIndex(); // Zakładamy 32-bit, ale wysyłamy 3 bajty
                // W tym przypadku konwertujemy do 3 bajtowej reprezentacji
                uint8_t idx_bytes[3];
                idx_bytes[0] = (last_idx >> 16) & 0xFF;
                idx_bytes[1] = (last_idx >> 8) & 0xFF;
                idx_bytes[2] = last_idx & 0xFF;
                SendResponseFrame(0x13, idx_bytes, 3);
            }
            break;
        case 0x04: // Podaj wartość archiwalną dla zadanego indeksu – parametr 4 bajtowy
            if(cmd_frame->data_length != 5) {
                SendErrorResponseFrame(0x30);
                return;
            }
            {
                // Sprawdzamy, czy indeks mieści się w zakresie zapisów archiwalnych
                uint32_t idx = param;
                uint32_t last_idx = GetLastArchiveIndex();
                if(idx > last_idx) {
                    SendErrorResponseFrame(0x23); // Nie istniejący indeks
                    return;
                }
                float temp = 0, hum = 0, pres = 0;
                GetHistoricalData(idx, &temp, &hum, &pres);
                // Odpowiedź zależy od typu pomiaru – tutaj przykładowo wysyłamy archiwalny odczyt temperatury
                // Można rozbudować: payload[0] – typ pomiaru, payload[1..] – wartość
                uint8_t payload[5];
                payload[0] = 0x00; // typ np. 0x00 oznacza temperaturę
                // Zakładamy, że temperatura jest przechowywana jako int32_t (wartość * 100)
                int32_t temp_val = (int32_t)roundf(temp * 100);
                payload[1] = (temp_val >> 16) & 0xFF;
                payload[2] = (temp_val >> 8) & 0xFF;
                payload[3] = temp_val & 0xFF;
                payload[4] = 0x00; // opcjonalnie dodatkowy bajt
                SendResponseFrame(0x04, payload, 5);
            }
            break;
        case 0x05: // Kasuje archiwalne zapisy – nie może mieć parametru
            if(cmd_frame->data_length != 1) {
                SendErrorResponseFrame(0x30);
                return;
            }
            DeleteHistoricalData();
            // Odpowiedź: np. wysyłamy potwierdzenie z kodem 0x05 i zerowym parametrem
            SendResponseFrame(0x05, NULL, 0);
            break;
        default:
            SendErrorResponseFrame(0x30); // Nie rozpoznany parametr
            break;
    }
}

/*
   Łączy pola: sender, receiver, data_length oraz payload (o długości data_length bajtów)
   i oblicza CRC16 IBM, używając calculateCRC() z crc16ibm.c.
*/
static uint16_t calculateParsedChecksum(Frame_structure *frame) {
    uint8_t buf[256];
    uint16_t idx = 0;
    buf[idx++] = frame->sender;
    buf[idx++] = frame->receiver;
    buf[idx++] = frame->data_length;
    for(uint8_t i = 0; i < frame->data_length; i++){
         buf[idx++] = frame->payload[i];
    }
    return calculateCRC(buf, idx);
}

static uint8_t isParsedChecksumValid(Frame_structure *frame) {
    return (calculateParsedChecksum(frame) == frame->checksum);
}

/* Funkcja przetwarzająca odebrane pole – konwersja danych z formatu hex do wartości binarnych */
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
            expected_payload_hex_length = dl * 2;  // Każdy bajt = 2 znaki
            break;
        }
        case FRAME_READ_PAYLOAD: {
            memcpy(raw_frame.payload, buffer, expected_payload_hex_length);
            // Konwertujemy payload: dla każdej pary znaków wywołujemy hexStringToByte
            for(uint16_t i = 0; i < (expected_payload_hex_length / 2); i++){
                parsed_frame.payload[i] = hexStringToByte(&buffer[i*2]);
            }
            break;
        }
        case FRAME_READ_CHECKSUM:
            memcpy(raw_frame.checksum, buffer, LEN_CHECKSUM);
            parsed_frame.checksum = hexStringToWord(buffer);
            break;
        default:
            break;
    }
}

/* Funkcja odbioru ramki – przetwarzanie bajt po bajcie */
void ReceiveFrame(void) {
    uint8_t RX_byte;
    while(UART_RXisNotEmpty()) {  // Zakładamy, że funkcje UART_RXisNotEmpty() i USART_getchar() są dostępne
        __disable_irq();
        RX_byte = USART_getchar();
        __enable_irq();

        if(RX_byte == FRAME_START) {
            current_state = FRAME_READ_SENDER;
            field_index = 0;
            expected_payload_hex_length = 0;
            memset(&raw_frame, 0, sizeof(raw_frame));
            continue;
        }

        if(current_state == FRAME_WAIT_START)
            continue;

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
                static uint8_t temp_buf[256]; // maksymalnie MAX_PAYLOAD_HEX znaków
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
                ProcessCommand(&parsed_frame);
            }
            current_state = FRAME_WAIT_START;
        }
    }
}

/* --- Funkcje tworzące ramkę zwrotną --- */

/* Funkcja tworząca ramkę odpowiedzi.
   Parametry:
      resp_code – kod odpowiedzi (np. 0x11, 0x16, 0x17, 0x18, 0x13, 0x04, 0x05),
      param – wskaźnik do danych odpowiedzi (binarnie),
      param_len – długość parametru (w bajtach).
   Ramka zwrotna: sender, receiver, data_length, payload, checksum.
   Przyjmujemy: sender = 'B', receiver = 'P'.
*/
void SendResponseFrame(uint8_t resp_code, uint8_t *param, uint8_t param_len) {
    Frame_structure resp;
    resp.sender = 'B';
    resp.receiver = 'P';
    resp.payload[0] = resp_code;
    if(param != NULL && param_len > 0) {
        memcpy(&resp.payload[1], param, param_len);
        resp.data_length = 1 + param_len;
    } else {
        resp.data_length = 1;
    }
    resp.checksum = calculateParsedChecksum(&resp);

    // Konwersja pól ramki do postaci hex (C-string)
    char raw_sender[3], raw_receiver[3], raw_datalen[3];
    char raw_payload[MAX_PAYLOAD_HEX+1], raw_checksum[5];
    byte_to_hex_chars(resp.sender, raw_sender); raw_sender[2] = '\0';
    byte_to_hex_chars(resp.receiver, raw_receiver); raw_receiver[2] = '\0';
    byte_to_hex_chars(resp.data_length, raw_datalen); raw_datalen[2] = '\0';
    uint16_t payload_hex_len = resp.data_length * 2;
    for(uint8_t i = 0; i < resp.data_length; i++){
        byte_to_hex_chars(resp.payload[i], &raw_payload[i*2]);
    }
    raw_payload[payload_hex_len] = '\0';
    byte_to_hex_chars((resp.checksum >> 8) & 0xFF, raw_checksum);
    byte_to_hex_chars(resp.checksum & 0xFF, &raw_checksum[2]);
    raw_checksum[4] = '\0';

    char final_frame[64]; // Maksymalna ramka jest krótka (około 2+2+2+(max_payload)+4+2 znaków)
    sprintf(final_frame, "^%s%s%s%s%s#", raw_sender, raw_receiver, raw_datalen, raw_payload, raw_checksum);
    USART_fsend("%s", final_frame);
}

/* Funkcja wysyłająca ramkę błędu.
   error_code – kod błędu (np. 0x21, 0x30, 0x23).
   Parametr błędu to 4 bajty z wartością 0xFF.
*/
void SendErrorResponseFrame(uint8_t error_code) {
    uint8_t error_param[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    SendResponseFrame(error_code, error_param, 4);
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
