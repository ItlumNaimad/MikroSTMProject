#include "main.h"
#include "frame.h"
#include "crc16ibm.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define FRAME_START '^' // ASCII 94
#define FRAME_END   '#' // ASCII 35

/* Rozmiary pól w wersji raw (jako ciągi hex) */
#define LEN_SENDER        2
#define LEN_RECEIVER      2
#define LEN_DATALENGTH    2  // 1 bajt = 2 znaki hex
#define LEN_CHECKSUM      4  // 2 bajty = 4 znaki hex

/* Stan odbioru poszczególnych pól */
typedef enum {
    FRAME_WAIT_START,
    FRAME_READ_SENDER,
    FRAME_READ_RECEIVER,
    FRAME_READ_DATALENGTH,
    FRAME_READ_PAYLOAD,    // Odczyt payloadu – długość zależna od data_length
    FRAME_READ_CHECKSUM,
    FRAME_COMPLETE
} FrameState;

/* Globalne zmienne dla odbioru ramki */
static FrameState current_state = FRAME_WAIT_START;
static uint8_t field_index = 0;  // Licznik odebranych znaków dla bieżącego pola
static uint16_t expected_payload_hex_length = 0; // liczba znaków, którą trzeba odebrać dla payloadu (data_length * 2)
static Frame_raw_structure raw_frame;   // Bufor surowej ramki
static Frame_structure parsed_frame;    // Struktura z przekonwertowanymi wartościami
extern UART_HandleTypeDef huart2;

/* Funkcja konwersji 8 znaków hex na 32-bitową wartość */
static uint32_t hexStringToDword(uint8_t *str, uint8_t len) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t nibble = hexDigitToByte(str[i]);
        if (nibble == 0xFF)
            return 0; // lub można obsłużyć błąd
        value = (value << 4) | nibble;
    }
    return value;
}

/*
 * Funkcja przetwarzająca odebrane pole – oprócz konwersji,
 * kopiujemy dane do raw_frame z tymczasowego buffora.
 * Tutaj również wywołujemy funkcję hexStringToByte z pliku crc16ibm.c
 * Zwrócone wartości przekazujemy do sturktury parsed_frame, która jest
 * od przetworzonej ramki.
 * */
static void processReceivedField(FrameState state, uint8_t *buffer, uint8_t len) {
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
            expected_payload_hex_length = dl * 2; // Każdy bajt = 2 znaki hex
            break;
        }
        case FRAME_READ_PAYLOAD:
            memcpy(raw_frame.payload, buffer, expected_payload_hex_length);
            /* Konwersja payloadu – tutaj rozbijamy na poszczególne bajty.
             * Aplikacja interpretująca komendę powinna wykorzystać parsed_frame.payload,
             * która to jest tablicą bajtów uzyskaną z każdej pary znaków.
             * Przykładowo: pierwszy bajt to kod komendy, kolejne 4 bajty to argument,
             * następne 3 bajty to indeks archiwalny.
             * Jeśli payload jest krótszy, aplikacja musi to rozpoznać po data_length.
            */
            for (uint8_t i = 0; i < parsed_frame.data_length; i++) {
                uint8_t pos = i * 2;
                parsed_frame.payload[i] = hexStringToByte(&buffer[pos]);
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

/* Funkcja sprawdzająca sumę kontrolną ramki.
 * Obliczamy CRC16 IBM (z funkcji calculateCRC) na podstawie danych:
 * sender, receiver, data_length oraz payload (czyli wszystkie pola przed checksumem).
 * Następnie porównujemy z wartością odebraną w checksum.
*/
static uint8_t isFrameChecksumValid(void) {
    /* Obliczamy łączną długość danych w bajtach, które będziemy przeliczać z wersji raw:
     * LEN_SENDER + LEN_RECEIVER + LEN_DATALENGTH + (data_length bajtów) – pamiętaj, że w wersji raw każdy bajt to 2 znaki.
     */
    uint16_t raw_length = LEN_SENDER + LEN_RECEIVER + LEN_DATALENGTH + expected_payload_hex_length;
    uint8_t binary_buffer[raw_length / 2];  // Każde 2 znaki = 1 bajt
    uint8_t index = 0;

    binary_buffer[index++] = hexStringToByte(raw_frame.sender);
    binary_buffer[index++] = hexStringToByte(raw_frame.receiver);
    binary_buffer[index++] = hexStringToByte(raw_frame.data_length);
    for (uint8_t i = 0; i < expected_payload_hex_length; i += 2) {
        binary_buffer[index++] = hexStringToByte(&raw_frame.payload[i]);
    }
    uint16_t calculated_crc = calculateCRC(binary_buffer, index);
    uint16_t received_crc = hexStringToWord(raw_frame.checksum);
    return (calculated_crc == received_crc);
}

/* Funkcja odbioru ramki – przetwarzanie bajt po bajcie.
 *  Używamy funkcji UART_RXisNotEmpty() zamiast UART_isNotEmpty() zgodnie z wymaganiami.
 */
void ReceiveFrame(void) {
    uint8_t RX_byte;
    while (UART_RXisNotEmpty()) {
        RX_byte = USART_getchar();

        /* Jeśli otrzymamy znak startu, resetujemy stan odbioru */
        if (RX_byte == FRAME_START) {
            current_state = FRAME_READ_SENDER;
            field_index = 0;
            expected_payload_hex_length = 0;
            /*
             * memset jest tutaj użyte do wyzerowania całej struktury ramki.
             * Jest to inicjalizacja struktury. Użyta tylko przy odbiorze znaku początku ramki
             */
            memset(&raw_frame, 0, sizeof(raw_frame));
            continue;
        }

        if (current_state == FRAME_WAIT_START)
            continue;

        /* Jeśli RX_byte to FRAME_END pojawi się nie tam, gdzie powinno, resetujemy odbiór.
           Jeżeli jednak ramka jest poprawnie skonstruowana, odebranie FRAME_END nie nastąpi, bo
           odbieramy dokładnie określoną liczbę znaków zgodnie z data_length.
        */
        if (RX_byte == FRAME_END) {
            /* Można opcjonalnie dodać logikę rozpoznania, czy znak ten pojawił się przedwcześnie.
               W tej implementacji zakładamy, że FRAME_END pojawia się tylko poza odbiorem.
            */
            current_state = FRAME_WAIT_START;
            continue;
        }

        switch (current_state) {
            case FRAME_READ_SENDER: {
                static uint8_t temp_buf[LEN_SENDER];
                temp_buf[field_index++] = RX_byte;
                if (field_index >= LEN_SENDER) {
                    processReceivedField(FRAME_READ_SENDER, temp_buf, LEN_SENDER);
                    field_index = 0;
                    current_state = FRAME_READ_RECEIVER;
                }
                break;
            }
            case FRAME_READ_RECEIVER: {
                static uint8_t temp_buf[LEN_RECEIVER];
                temp_buf[field_index++] = RX_byte;
                if (field_index >= LEN_RECEIVER) {
                    processReceivedField(FRAME_READ_RECEIVER, temp_buf, LEN_RECEIVER);
                    field_index = 0;
                    current_state = FRAME_READ_DATALENGTH;
                }
                break;
            }
            case FRAME_READ_DATALENGTH: {
                static uint8_t temp_buf[LEN_DATALENGTH];
                temp_buf[field_index++] = RX_byte;
                if (field_index >= LEN_DATALENGTH) {
                    processReceivedField(FRAME_READ_DATALENGTH, temp_buf, LEN_DATALENGTH);
                    field_index = 0;
                    current_state = FRAME_READ_PAYLOAD;
                }
                break;
            }
            case FRAME_READ_PAYLOAD: {
                static uint8_t temp_buf[MAX_PAYLOAD_HEX];
                temp_buf[field_index++] = RX_byte;
                if (field_index >= expected_payload_hex_length) {
                    processReceivedField(FRAME_READ_PAYLOAD, temp_buf, expected_payload_hex_length);
                    field_index = 0;
                    current_state = FRAME_READ_CHECKSUM;
                }
                break;
            }
            case FRAME_READ_CHECKSUM: {
                static uint8_t temp_buf[LEN_CHECKSUM];
                temp_buf[field_index++] = RX_byte;
                if (field_index >= LEN_CHECKSUM) {
                    processReceivedField(FRAME_READ_CHECKSUM, temp_buf, LEN_CHECKSUM);
                    field_index = 0;
                    current_state = FRAME_COMPLETE;
                }
                break;
            }
            default:
                break;
        }
        // STAN ZROBIONY DLA DEBUGOWANIA - ODPOWIEDZI BĘDĄ W FORMIE RAMEK
        if (current_state == FRAME_COMPLETE) {
            if (isFrameChecksumValid()) {
                /* Po walidacji możemy przekazać odebrane dane do aplikacji.
                   Payload interpretuje się według ustalonego formatu – np. pierwszy bajt to kod komendy.
                */
                uint8_t command = parsed_frame.payload[0];
                // Przykładowa interpretacja: komenda 16, 17, 18 oznacza odczyt temperatury, wilgotności lub ciśnienia
                // oraz dodatkowo w payload może być argument (dla ustawienia interwału) lub odczyt archiwalny.
                char response[64];
                sprintf(response, "Cmd: %02X DL: %02X Payload:", command, parsed_frame.data_length);
                while(!UART_TXisNotEmpty()); // czekamy, aż bufor TX będzie gotowy
                HAL_UART_Transmit_IT(&huart2, (uint8_t *)response, strlen(response));
                memset(response, 0, sizeof(response));
                for (uint8_t i = 0; i < parsed_frame.data_length; i++) {
                    sprintf(response, " %02X", parsed_frame.payload[i]);
                    while(!UART_TXisNotEmpty());
                    HAL_UART_Transmit_IT(&huart2, (uint8_t *)response, strlen(response));
                    memset(response, 0, sizeof(response));
                }
                sprintf(response, " CRC: %04X\r\n", parsed_frame.checksum);
                while(!UART_TXisNotEmpty());
                HAL_UART_Transmit_IT(&huart2, (uint8_t *)response, strlen(response));
            } else {
                while(!UART_TXisNotEmpty());
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
