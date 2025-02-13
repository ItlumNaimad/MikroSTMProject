#ifndef INC_FRAME_H_
#define INC_FRAME_H_

#include "main.h"

typedef struct {
    uint8_t sender;                    	   // Adres nadawcy
    uint8_t receiver;                	   // Adres odbiorcy
    uint8_t function;               	   // komenda do wykonania przez mikroprocesor
    uint16_t argument;    				   // Argument komendy
    uint16_t checksum;                     // Suma kontrolna
} Frame_structure;

typedef struct {
    uint8_t sender[2];                    	   // Adres nadawcy
    uint8_t receiver[2];                	   // Adres odbiorcy
    uint8_t function[2];                 	   // komenda do wykonania przez mikroprocesor
    uint8_t argument[8];    				   // Argument komendy
    uint8_t checksum[4];                       // Suma kontrolna
} Frame_raw_structure;

void ReceiveFrame();


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
