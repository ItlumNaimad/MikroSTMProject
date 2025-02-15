#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include "main.h"


// * Struktury *
typedef struct {
    uint8_t sender;             // Adres nadawcy (1 bajt)
    uint8_t receiver;           // Adres odbiorcy (1 bajt)
    uint8_t function;           // Kod komendy (1 bajt)
    uint32_t argument;          // Argument komendy (4 bajty, 32-bit)
    uint16_t archive_index;     // Indeks archiwalnego odczytu (2 bajty)
    uint16_t checksum;          // Suma kontrolna (2 bajty)
} Frame_structure;

typedef struct {
    uint8_t sender[2];          // 2 znaki heksadecymalne
    uint8_t receiver[2];        // 2 znaki heksadecymalne
    uint8_t function[2];        // 2 znaki heksadecymalne
    uint8_t argument[8];        // 8 znaków heksadecymalnych (dla 32-bitowej wartości)
    uint8_t archive_index[4];   // 4 znaki heksadecymalne (dla 16-bitowej wartości)
    uint8_t checksum[4];        // 4 znaki heksadecymalne (dla 16-bitowej wartości)
} Frame_raw_structure;

// * Prototypy *
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
