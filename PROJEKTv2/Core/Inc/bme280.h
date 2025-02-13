#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "main.h"

#ifndef BME280_H
#define BME280_H

#include "stm32f4xx_hal.h"

// Adres BME280 – przyjmujemy, że sensor ma adres 0x76 (może być też 0x77); pamiętaj o przesunięciu 1 bit w lewo, gdy korzystasz z HAL.
#define BME280_ADDR (0x77 << 1)

// Rejestry BME280
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_DATA_START 0xF7  // Dane zaczynają się tutaj: [P_MSB, P_LSB, P_XLSB, T_MSB, T_LSB, T_XLSB, H_MSB, H_LSB]

// Stan maszyny pomiarowej
typedef enum {
    BME280_STATE_IDLE,
    BME280_STATE_CONFIGURE,
    BME280_STATE_TRIGGER_MEAS,
    BME280_STATE_WAIT_CONVERSION,
    BME280_STATE_READ_RAW,
    BME280_STATE_PROCESS,
    // Ewentualnie: BME280_STATE_POWEROFF – dla trybu oszczędzania energii, lecz w trybie forced sensor sam przechodzi w stan sleep
} BME280_State;

// Struktura surowych danych (8 bajtów)
typedef struct {
    uint8_t data[8];
} BME280_RawData;

// Struktura wyniku pomiaru – po przetworzeniu surowych danych
typedef struct {
    float temperature; // w stopniach Celsjusza
    float pressure;    // np. w hPa
    float humidity;    // % RH
} BME280_Measurement;

// Uchwyt dla modułu BME280
typedef struct {
    I2C_HandleTypeDef *hi2c; // uchwyt I2C
    BME280_State state;      // aktualny stan maszyny stanów
    uint32_t tick_start;     // znacznik czasu dla odmierzania czasu konwersji
    uint32_t last_measurement_tick; // czas ostatniego pomiaru
    BME280_RawData raw;      // surowe dane odczytane z czujnika
    BME280_Measurement measurement; // przetworzony wynik pomiaru
    uint8_t configured;      // flaga konfiguracji czujnika
} BME280_HandleTypeDef;

// Deklaracja funkcji
void BME280_Init(BME280_HandleTypeDef* bme, I2C_HandleTypeDef* hi2c);
void BME280_Task(BME280_HandleTypeDef* bme);
uint32_t BME280_CalculateMeasurements(BME280_HandleTypeDef* bme);
void BME280_ProcessData(BME280_HandleTypeDef* bme);

// Callbacki obsługi I2C
void BME280_I2C_TxCpltCallback(BME280_HandleTypeDef* bme);
void BME280_I2C_RxCpltCallback(BME280_HandleTypeDef* bme);

#endif /* BME280_H */

#endif /* INC_BME280_H_ */
