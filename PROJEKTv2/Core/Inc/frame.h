#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include "main.h"

/*
 * Definicje:
 * Maksymalny payload to 12 bajtów, gdzie:
 *   - 1 bajt: kod komendy,
 *   - 4 bajty: argument (32-bit, np. temperatura mnożona przez 100),
 *   - 3 bajty: indeks archiwalny.
 */
#define MAX_PAYLOAD_BYTES 12
#define MAX_PAYLOAD_HEX   (MAX_PAYLOAD_BYTES * 2)

/*
 * Struktura ramki (przetworzona – wartości binarne).
 * Kolejność pól:
 *   sender         (1 bajt)
 *   receiver       (1 bajt)
 *   data_length    (1 bajt) – długość payloadu w bajtach
 *   payload        (zmienna, maks. MAX_PAYLOAD_BYTES bajtów)
 *   checksum       (2 bajty) – CRC16 IBM (0xA001)
 */
typedef struct {
    uint8_t sender;
    uint8_t receiver;
    uint8_t data_length;  // Długość payloadu (ilość bajtów, np. 1, 3, 12)
    uint8_t payload[MAX_PAYLOAD_BYTES];
    uint16_t checksum;
} Frame_structure;

/*
 * Ramka surowa – odebrane dane jako znaki hex.
 * Kolejność pól (ilość znaków):
 *   sender:         2 znaki
 *   receiver:       2 znaki
 *   data_length:    2 znaki
 *   payload:        zmienna, maks. MAX_PAYLOAD_HEX znaków
 *   checksum:       4 znaki
 */
typedef struct {
    uint8_t sender[2];
    uint8_t receiver[2];
    uint8_t data_length[2];
    uint8_t payload[MAX_PAYLOAD_HEX];
    uint8_t checksum[4];
} Frame_raw_structure;

void ReceiveFrame(void);


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
#endif /* INC_FRAME_H_ */
