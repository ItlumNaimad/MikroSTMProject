#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "main.h"
#include "stm32f4xx_hal.h"


// Adres BME280 – przyjmujemy, że sensor ma adres 0x76 (Bo to zalęży od łączenia pinu SDO, którego w swoim modelu nie mam);
// pamiętać o przesunięciu 1 bit w lewo przy użyciu HAL bo potrzebujemy wartości 8 bitowych.
#define BME280_ADDR (0x77 << 1)

// Rejestry BME280
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_DATA_START 0xF7  // Dane zaczynają się tutaj: [P_MSB, P_LSB, P_XLSB, T_MSB, T_LSB, T_XLSB, H_MSB, H_LSB]
#define BME280_REG_TEMP_PRESS_CALIB_DATA 0x88
#define BME280_LEN_TEMP_PRESS_CALIB_DATA 26
#define BME280_REG_HUMIDITY_CALIB_DATA 0xE1
#define BME280_LEN_HUMIDITY_CALIB_DATA 7

/* Ustalony rozmiar bufora – co najmniej 750 wpisów */
#define BUFFER_SIZE 750

extern volatile uint32_t ST_Ticks;


// Stan maszyny pomiarowej
typedef enum {
    BME280_STATE_IDLE,
    BME280_STATE_CONFIGURE,
    BME280_STATE_TRIGGER_MEAS,
    BME280_STATE_WAIT_CONVERSION,
    BME280_STATE_READ_RAW,
    BME280_STATE_PROCESS
} BME280_State;

// Struktura surowych danych (8 bajtów)
typedef struct {
    uint8_t data[8];
} BME280_RawData;

// Struktura wyniku pomiaru – po przetworzeniu surowych danych
typedef struct {
    float temperature; // w °C
    float pressure;    // w hPa (lub Pa)
    float humidity;    // w %RH
} BME280_Measurement;

// Skopiowane z bme280_defs.h
struct bme280_calib_data
{
    /*! Calibration coefficient for the temperature sensor */
    uint16_t dig_t1;
    /*! Calibration coefficient for the temperature sensor */
    int16_t dig_t2;
    /*! Calibration coefficient for the temperature sensor */
    int16_t dig_t3;
    /*! Calibration coefficient for the pressure sensor */
    uint16_t dig_p1;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p2;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p3;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p4;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p5;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p6;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p7;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p8;
    /*! Calibration coefficient for the pressure sensor */
    int16_t dig_p9;
    /*! Calibration coefficient for the humidity sensor */
    uint8_t dig_h1;
    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h2;
    /*! Calibration coefficient for the humidity sensor */
    uint8_t dig_h3;
    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h4;
    /*! Calibration coefficient for the humidity sensor */
    int16_t dig_h5;
    /*! Calibration coefficient for the humidity sensor */
    int8_t dig_h6;
    /*! Variable to store the intermediate temperature coefficient */
    int32_t t_fine;
};

// Uchwyt dla modułu BME280 – rozszerzony o dane kalibracyjne
typedef struct {
    I2C_HandleTypeDef *hi2c; // uchwyt I2C
    BME280_State state;      // aktualny stan maszyny stanów
    uint32_t tick_start;     // znacznik czasu (ST_Ticks) dla odmierzania czasu konwersji
    uint32_t last_measurement_tick; // czas ostatniego pomiaru (ST_Ticks)
    BME280_RawData raw;      // surowe dane odczytane z czujnika
    BME280_Measurement measurement; // przetworzony wynik pomiaru
    uint8_t configured;      // flaga konfiguracji czujnika
    struct bme280_calib_data calib_data; // Dane kalibracyjne (odczytane z czujnika)
} BME280_HandleTypeDef;


/* Prototypy funkcji */
// Bufor kołowy dla zapisów archiwalnych pomiarów:
void AppendMeasurement(BME280_Measurement meas);
void GetLatestMeasurement(BME280_Measurement *meas);
int GetHistoricalMeasurement(uint16_t index, BME280_Measurement *meas);
void ClearMeasurementBuffer(void);
uint32_t GetLastArchiveIndex(void);

void BME280_Init(BME280_HandleTypeDef* bme, I2C_HandleTypeDef* hi2c);
void BME280_Task(BME280_HandleTypeDef* bme);
void BME280_ProcessData(BME280_HandleTypeDef* bme);

// Callbacki I2C
void BME280_I2C_TxCpltCallback(BME280_HandleTypeDef* bme);
void BME280_I2C_RxCpltCallback(BME280_HandleTypeDef* bme);

// Uchwyt do struktury czujnik
extern BME280_HandleTypeDef bme_handle;

#endif /* INC_BME280_H_ */
