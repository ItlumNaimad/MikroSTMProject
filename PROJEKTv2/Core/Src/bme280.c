#include "bme280.h"
#include "stdio.h"

// Globalny uchwyt – przykładowo
BME280_HandleTypeDef bme_handle;

// Zewnętrzne zmienne
uint32_t read_interval = 1000;

/**
 * @brief Inicjalizacja czujnika BME280.
 * Ustawia uchwyt I2C, stan początkowy oraz flagę konfiguracji.
 */
void BME280_Init(BME280_HandleTypeDef* bme, I2C_HandleTypeDef* hi2c) {
    bme->hi2c = hi2c;
    bme->state = BME280_STATE_IDLE;
    bme->last_measurement_tick = HAL_GetTick();
    bme->configured = 0;
}

/**
 * @brief Główna funkcja zadaniowa czujnika BME280.
 * Realizuje cykl pomiarowy: konfiguracja, wyzwolenie pomiaru, oczekiwanie na konwersję,
 * odczyt danych i przetwarzanie.
 */
void BME280_Task(BME280_HandleTypeDef* bme) {
    uint32_t currentTick = HAL_GetTick();

    switch(bme->state) {
    case BME280_STATE_IDLE:
        if (!bme->configured) {
            // Konfiguracja czujnika:
            // Ustaw rejestr CTRL_HUM (0xF2) – np. oversampling x1: 0x01
            uint8_t ctrl_hum = 0x01;
            if (HAL_I2C_Mem_Write_IT(bme->hi2c, BME280_ADDR,
                                     BME280_REG_CTRL_HUM, I2C_MEMADD_SIZE_8BIT,
                                     &ctrl_hum, 1) == HAL_OK) {
                bme->state = BME280_STATE_CONFIGURE;
                // Można wysłać debug przez UART, np. "Konfiguracja: CTRL_HUM"
            }
        } else if ((currentTick - bme->last_measurement_tick) >= read_interval) {
            // Rozpocznij pomiar: w trybie forced, zapisz CTRL_MEAS (0xF4).
            // Ustaw oversampling x1 dla temperatury i ciśnienia oraz tryb forced (0x01)
            uint8_t ctrl_meas = (0x01 << 5) | (0x01 << 2) | 0x01;  // osrs_t=1, osrs_p=1, mode=forced
            if (HAL_I2C_Mem_Write_IT(bme->hi2c, BME280_ADDR,
                                      BME280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT,
                                      &ctrl_meas, 1) == HAL_OK) {
                bme->state = BME280_STATE_TRIGGER_MEAS;
            }
        }
        break;

    case BME280_STATE_CONFIGURE:
        // Po ustawieniu rejestru CTRL_HUM przechodzimy do konfiguracji rejestru CONFIG (0xF5)
        {
            // Ustaw rejestr CONFIG – np. standby time, filter (dla uproszczenia ustawiamy 0x00)
            uint8_t config = 0x00;
            if (HAL_I2C_Mem_Write_IT(bme->hi2c, BME280_ADDR,
                                      BME280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT,
                                      &config, 1) == HAL_OK) {
                bme->configured = 1;
                bme->state = BME280_STATE_IDLE;
            }
        }
        break;

    case BME280_STATE_TRIGGER_MEAS:
        // W trybie forced sensor zaczyna pomiar natychmiast po zapisie CTRL_MEAS.
        bme->tick_start = currentTick;
        bme->state = BME280_STATE_WAIT_CONVERSION;
        break;

    case BME280_STATE_WAIT_CONVERSION: {
        // Czas konwersji zależy od oversamplingu. Dla osrs=1 zwykle wystarczy około 10 ms.
        if ((currentTick - bme->tick_start) >= 10) {
            // Odczytujemy 8 bajtów danych od rejestru DATA (0xF7)
            if (HAL_I2C_Mem_Read_IT(bme->hi2c, BME280_ADDR,
                                     BME280_REG_DATA_START, I2C_MEMADD_SIZE_8BIT,
                                     bme->raw.data, 8) == HAL_OK) {
                bme->state = BME280_STATE_READ_RAW;
            }
        }
        break;
    }

    case BME280_STATE_READ_RAW:
        // Stan ten jest zmieniany przez callback RX, po zakończeniu odczytu.
        break;

    case BME280_STATE_PROCESS:
        // Przetwarzamy surowe dane – wywołujemy funkcję kompensacji.
        BME280_ProcessData(bme);

        /*
        // Zapisujemy wynik do bufora kołowego (sekcja krytyczna).
        __disable_irq();
        bufferSensor_put(&SENSOR_buffer, bme->measurement);
        __enable_irq();
		*/

        // Aktualizujemy znacznik ostatniego pomiaru.
        bme->last_measurement_tick = currentTick;
        bme->state = BME280_STATE_IDLE;
        break;

    default:
        break;
    }
}

/**
 * @brief Przetwarzanie surowych danych BME280.
 *
 * Funkcja konwertuje odczytane surowe dane z rejestrów czujnika na wartości fizyczne:
 * temperaturę, ciśnienie i wilgotność. (Tutaj zastosowano uproszczoną wersję – w praktyce
 * należy użyć pełnych algorytmów kompensacji zgodnych z dokumentacją.)
 */
void BME280_ProcessData(BME280_HandleTypeDef* bme) {
    // Surowe dane:
    // Pressure: bme->raw.data[0] (MSB), [1] (LSB), [2] (XLSB)
    // Temperature: [3] (MSB), [4] (LSB), [5] (XLSB)
    // Humidity: [6] (MSB), [7] (LSB)
    // Dla uproszczenia złożymy bajty i przeliczmy wartości przybliżone.

    // Odczyt temperatury (przykładowo – bez kompensacji):
    int32_t adc_T = ((int32_t)bme->raw.data[3] << 12) |
                    ((int32_t)bme->raw.data[4] << 4) |
                    ((int32_t)(bme->raw.data[5] >> 4));
    // Odczyt ciśnienia:
    uint32_t adc_P = ((uint32_t)bme->raw.data[0] << 12) |
                     ((uint32_t)bme->raw.data[1] << 4) |
                     ((uint32_t)(bme->raw.data[2] >> 4));
    // Odczyt wilgotności:
    uint32_t adc_H = ((uint32_t)bme->raw.data[6] << 8) |
                     ((uint32_t)bme->raw.data[7]);

    // Uproszczone przeliczenie (w praktyce należy użyć algorytmów kompensacji):
    bme->measurement.temperature = adc_T / 100.0f; // przykładowa konwersja
    bme->measurement.pressure = adc_P / 256.0f;      // przykładowa konwersja
    bme->measurement.humidity = adc_H / 1024.0f;       // przykładowa konwersja
}

/**
 * @brief Callback transmisji I2C (TX complete) dla BME280.
 */
void BME280_I2C_TxCpltCallback(BME280_HandleTypeDef* bme) {
    switch(bme->state) {
    case BME280_STATE_CONFIGURE:
        bme->configured = 1;
        bme->state = BME280_STATE_IDLE;
        break;
    case BME280_STATE_TRIGGER_MEAS:
        bme->tick_start = HAL_GetTick();
        bme->state = BME280_STATE_WAIT_CONVERSION;
        break;
    default:
        break;
    }
}

/**
 * @brief Callback odbioru I2C (RX complete) dla BME280.
 */
void BME280_I2C_RxCpltCallback(BME280_HandleTypeDef* bme) {
    if (bme->state == BME280_STATE_WAIT_CONVERSION) {
        // Odczyt surowych danych został zakończony.
        bme->state = BME280_STATE_PROCESS;
    }
}

/**
 * @brief Funkcje callback HAL I2C.
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == bme_handle.hi2c) {
        BME280_I2C_TxCpltCallback(&bme_handle);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == bme_handle.hi2c) {
        BME280_I2C_RxCpltCallback(&bme_handle);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == bme_handle.hi2c) {
        // Opcjonalnie: implementacja resetu magistrali lub obsługi błędów.
        return;
    }
}
