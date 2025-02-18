#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include "main.h"

#define MAX_PAYLOAD_BYTES 14
#define MAX_PAYLOAD_HEX   (MAX_PAYLOAD_BYTES * 2)

/*
 * Ramka przetworzona – wartości binarne.
 * Kolejność pól:
 *   sender (1 bajt)
 *   receiver (1 bajt)
 *   data_length (1 bajt) – długość pola payload (w bajtach)
 *   payload (zmienna, max MAX_PAYLOAD_BYTES bajtów)
 *   checksum (2 bajty)
 */
typedef struct {
    uint8_t sender;
    uint8_t receiver;
    uint8_t data_length; // liczba bajtów payloadu
    uint8_t payload[MAX_PAYLOAD_BYTES];
    uint16_t checksum;
} Frame_structure;

/*
 * Ramka surowa – odebrane dane jako ciągi znaków hex.
 * Kolejność pól (ilości znaków):
 *   sender:         2 znaki + '\0'
 *   receiver:       2 znaki + '\0'
 *   data_length:    2 znaki + '\0'
 *   payload:        zmienna, max (MAX_PAYLOAD_BYTES*2) znaków + '\0'
 *   checksum:       4 znaki + '\0'
 */
typedef struct {
    uint8_t sender[3];
    uint8_t receiver[3];
    uint8_t data_length[3];
    uint8_t payload[MAX_PAYLOAD_HEX + 1];
    uint8_t checksum[5];
} Frame_raw_structure;

/* Funkcje przetwarzania komend i wysyłania odpowiedzi */
void ProcessCommand(Frame_structure *cmd_frame);
void SendResponseFrame(uint8_t resp_code, uint8_t *param, uint8_t param_len);
void SendErrorResponseFrame(uint8_t error_code);

void ReceiveFrame();

#endif /* INC_FRAME_H_ */

/* Pozostałości po pracy z biblioteką oryginalną od producenta czujnika

	// *bme280_read_fptr_t   		bme280_defs.h
	int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
	// *bme280_write_fptr_t 		bme280_defs.h
	int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
	// *bme280_delay_us_fptr_t   	bme280_defs.h
	void user_delay_us(uint32_t period, void *intf_ptr);

	// Funkcja inicjalizujaca wartosci do struktury czujnika
	void init_bme280_conf();
*/

