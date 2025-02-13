#include "main.h"
#include "frame.h"

#define FRAME_START 94 //^
#define FRAME_END 35 //#

/*
 * Zmienne potrzebne do odczytywania ramki
 */
extern UART_HandleTypeDef huart2;

typedef enum{
	 WAIT_FOR_FRAME_START,
//	 READ_SENDER,
//	 READ_RECEIVER,
//	 READ_FUNCTION,
//	 READ_ARGUMENT_HIGH, // Dodano dla argumentu 16-bitowego
//	 READ_ARGUMENT_LOW,  // Dodano dla argumentu 16-bitowego
//	 READ_CHECKSUM_HIGH, // Dodano dla sumy kontrolnej 16-bitowej
//	 READ_CHECKSUM_LOW,  // Dodano dla sumy kontrolnej 16-bitowej
	 WAIT_FOR_FRAME_END,
	 VALIDATE_FRAME,
	 PROCESS_FRAME,
	 READ_FIELDS_START,  // Pierwszy bajt nagłówka ramki
	 READ_FIELDS_END = READ_FIELDS_START + sizeof(Frame_raw_structure),
	 // Ostatni bajt nagłówka ramki
}FrameStates;

void ReceiveFrame(){
	static Frame_structure taken_frame; // Struktura do przechowywania odebranej ramki
	static Frame_raw_structure receiving_frame; // Struktura do przechowywania odebranych danych ramki
	static FrameStates current_state = WAIT_FOR_FRAME_START;
	uint8_t RX_temporary;
	while (UART_isNotEmpty()) { // Sprawdzamy, czy bufor nie jest pusty
	    __disable_irq();
		RX_temporary = USART_getchar(); // Pobieramy znak z bufora
		__enable_irq();

		if(RX_temporary == FRAME_START)
		{
			current_state = READ_FIELDS_START;
			continue;
		}else
		if(RX_temporary == FRAME_END && current_state != WAIT_FOR_FRAME_END)
		{
			current_state = WAIT_FOR_FRAME_START;
			continue;
		} else
		if(current_state == WAIT_FOR_FRAME_START)
		{
			continue;
		} else
		if(RX_temporary == FRAME_END && current_state == WAIT_FOR_FRAME_END)
		{
			if(isChecksumValid(taken_frame))
			{
				uint8_t command = taken_frame.function;
				uint16_t argument = taken_frame.argument;
				// TODO: Parsowanie komend

			}
		} else if (current_state == VALIDATE_FRAME) {
			// Odczyt liczb szestnastkowych do taken_frame
			taken_frame.sender = hexStringToByte(receiving_frame.sender);
			taken_frame.receiver = hexStringToByte(receiving_frame.receiver);
			taken_frame.function = hexStringToByte(receiving_frame.function);
			taken_frame.argument = hexStringToWord(receiving_frame.argument);
			taken_frame.checksum = hexStringToWord(receiving_frame.checksum);
		}
	    if (current_state >= READ_FIELDS_START && current_state < READ_FIELDS_END) {
			if(RX_temporary == FRAME_END)
			{
				current_state = VALIDATE_FRAME;
			} else {
				if(hexDigitToByte(RX_temporary) == 0xff)
				{
					current_state = WAIT_FOR_FRAME_START;
					continue;
				}
				((uint8_t*)&receiving_frame)[current_state - READ_FIELDS_START] = RX_temporary;
				current_state += 1;
				if (current_state == READ_FIELDS_END) {
					current_state = VALIDATE_FRAME;
					HAL_UART_Transmit_IT(&huart2, "Validated", 10);
				}
			}
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
