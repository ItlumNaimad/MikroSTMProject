#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include "main.h"

/*
 * Struktura ramki przetworzonej – wartości binarne.
 * Kolejność pól:
 *   sender (1 bajt)
 *   receiver (1 bajt)
 *   data_length (1 bajt) – długość payloadu (w bajtach)
 *   payload (zmienna długość, max 14 bajtów)
 *   checksum (2 bajty)
 *   FRAME_END (1 bajt) – nie jest przechowywany w strukturze, ale ramka jest poprawna tylko, gdy pojawi się znak końca.
 */
typedef struct {
    uint8_t sender;
    uint8_t receiver;
    uint8_t data_length;  // długość payloadu w bajtach
    uint8_t payload[14];  // maksymalnie 14 bajtów (może zawierać: kod, argument, opcjonalnie pole archiwalne)
    uint16_t checksum;
} Frame_structure;

/*
 * Struktura ramki surowej – odebrane dane jako ciągi hex.
 * Kolejność pól (ilości znaków):
 *   sender:         2 znaki
 *   receiver:       2 znaki
 *   data_length:    2 znaki
 *   payload:        zmienna, max 28 znaków (czyli do 14 bajtów)
 *   checksum:       4 znaki
 *   FRAME_END:      1 znak (na końcu, nie wliczany w tę strukturę)
 */
typedef struct {
    uint8_t sender[2];
    uint8_t receiver[2];
    uint8_t data_length[2];
    uint8_t payload[28];  // max 14 bajtów * 2 znaki
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
