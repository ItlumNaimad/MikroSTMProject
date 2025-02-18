#include "bme280.h"
#include "stdio.h"

// Globalny uchwyt – przykładowo
BME280_HandleTypeDef bme_handle;


// Zewnętrzne zmienne
uint32_t read_interval = 1000; // interwał w ms

/* Ring buffer przechowujący wyniki pomiarów */
static BME280_Measurement ring_buffer[BUFFER_SIZE];
/* Indeks wskazujący miejsce do zapisu nowego wpisu (bufor cykliczny) */
static uint16_t ring_buffer_index = 0;
/* Liczba wpisów obecnie zapisanych w buforze */
static uint16_t ring_buffer_count = 0;

/* Funkcja dodająca nowy pomiar do bufora kołowego */
void AppendMeasurement(BME280_Measurement meas) {
    ring_buffer[ring_buffer_index] = meas;
    ring_buffer_index = (ring_buffer_index + 1) % BUFFER_SIZE;
    if(ring_buffer_count < BUFFER_SIZE) {
        ring_buffer_count++;
    }
}

/* Funkcja zwracająca najnowszy pomiar */
void GetLatestMeasurement(BME280_Measurement *meas) {
    if(ring_buffer_count == 0) return;
    uint16_t latest = (ring_buffer_index == 0) ? (BUFFER_SIZE - 1) : (ring_buffer_index - 1);
    *meas = ring_buffer[latest];
}

/* Funkcja pobierająca pomiar archiwalny.
   Parametr 'index' oznacza: 0 – najnowszy, 1 – poprzedni itd.
   Zwraca 0 przy sukcesie, -1 gdy indeks poza zakresem.
*/
int GetHistoricalMeasurement(uint16_t index, BME280_Measurement *meas) {
    if(ring_buffer_count == 0 || index >= ring_buffer_count)
        return -1; // brak danych lub indeks poza zakresem
    uint16_t pos = (ring_buffer_index + BUFFER_SIZE - index - 1) % BUFFER_SIZE;
    *meas = ring_buffer[pos];
    return 0;
}

/* Funkcja czyszcząca bufor kołowy */
void ClearMeasurementBuffer(void) {
    ring_buffer_count = 0;
    ring_buffer_index = 0;
    memset(ring_buffer, 0, sizeof(ring_buffer));
}

/* Funkcja zwracająca indeks ostatniego zapisanego pomiaru archiwalnego.
   Zwraca wartość od 0 do (ring_buffer_count - 1). */
uint32_t GetLastArchiveIndex(void) {
    if(ring_buffer_count == 0)
        return 0;
    return ring_buffer_count - 1;
}

/**
 * @brief Inicjalizacja czujnika BME280.
 * Ustawia uchwyt I2C, stan początkowy, pobiera ST_Ticks z SysTick oraz flagę konfiguracji.
 */
void BME280_Init(BME280_HandleTypeDef* bme, I2C_HandleTypeDef* hi2c) {
    uint8_t calib_temp[26]; // dane kalibracyjne dla temperatury i ciśnienia (0x88 do 0xA1)
    uint8_t calib_hum[7];   // dane kalibracyjne dla wilgotności (0xE1 do 0xE7)

    bme->hi2c = hi2c;
    bme->state = BME280_STATE_IDLE;
    bme->last_measurement_tick = ST_Ticks;  // Używamy zmiennej ST_Ticks
    bme->configured = 0;

    // Odczyt danych kalibracyjnych z rejestrów czujnika
    if(HAL_I2C_Mem_Read(bme->hi2c, BME280_ADDR, BME280_REG_TEMP_PRESS_CALIB_DATA, I2C_MEMADD_SIZE_8BIT,
                        calib_temp, 26, HAL_MAX_DELAY) != HAL_OK) {
        // Obsługa błędu – możesz wywołać Error_Handler() lub ustawić status błędu
    }

    if(HAL_I2C_Mem_Read(bme->hi2c, BME280_ADDR, BME280_REG_HUMIDITY_CALIB_DATA, I2C_MEMADD_SIZE_8BIT,
                        calib_hum, 7, HAL_MAX_DELAY) != HAL_OK) {
        // Obsługa błędu
    }

    // Wypełnienie danych kalibracyjnych dla temperatury i ciśnienia
    bme->calib_data.dig_t1 = (uint16_t)(calib_temp[0] | (calib_temp[1] << 8));
    bme->calib_data.dig_t2 = (int16_t)(calib_temp[2] | (calib_temp[3] << 8));
    bme->calib_data.dig_t3 = (int16_t)(calib_temp[4] | (calib_temp[5] << 8));
    bme->calib_data.dig_p1 = (uint16_t)(calib_temp[6] | (calib_temp[7] << 8));
    bme->calib_data.dig_p2 = (int16_t)(calib_temp[8] | (calib_temp[9] << 8));
    bme->calib_data.dig_p3 = (int16_t)(calib_temp[10] | (calib_temp[11] << 8));
    bme->calib_data.dig_p4 = (int16_t)(calib_temp[12] | (calib_temp[13] << 8));
    bme->calib_data.dig_p5 = (int16_t)(calib_temp[14] | (calib_temp[15] << 8));
    bme->calib_data.dig_p6 = (int16_t)(calib_temp[16] | (calib_temp[17] << 8));
    bme->calib_data.dig_p7 = (int16_t)(calib_temp[18] | (calib_temp[19] << 8));
    bme->calib_data.dig_p8 = (int16_t)(calib_temp[20] | (calib_temp[21] << 8));
    bme->calib_data.dig_p9 = (int16_t)(calib_temp[22] | (calib_temp[23] << 8));
    bme->calib_data.dig_h1 = calib_temp[25]; // ostatni bajt z pierwszego bloku

    // Wypełnienie danych kalibracyjnych dla wilgotności
    bme->calib_data.dig_h2 = (int16_t)(calib_hum[0] | (calib_hum[1] << 8));
    bme->calib_data.dig_h3 = calib_hum[2];
    bme->calib_data.dig_h4 = (int16_t)(((int8_t)calib_hum[3] << 4) | (calib_hum[4] & 0x0F));
    bme->calib_data.dig_h5 = (int16_t)(((int8_t)calib_hum[5] << 4) | (calib_hum[4] >> 4));
    bme->calib_data.dig_h6 = (int8_t)calib_hum[6];

    // Uwaga: t_fine będzie obliczane podczas przetwarzania pomiarów
}
/**
 * @brief Główna funkcja zadaniowa czujnika BME280.
 * Realizuje cykl pomiarowy: konfiguracja, wyzwolenie pomiaru, oczekiwanie na konwersję,
 * odczyt danych i przetwarzanie.
 */
void BME280_Task(BME280_HandleTypeDef* bme) {
    uint32_t currentTick = ST_Ticks; // Używamy ST_Ticks

    switch(bme->state) {
    case BME280_STATE_IDLE:
        if (!bme->configured) {
            // Konfiguracja czujnika – ustaw CTRL_HUM (0xF2) na oversampling x1 (0x01)
            uint8_t ctrl_hum = 0x01;
            if (HAL_I2C_Mem_Write_IT(bme->hi2c, BME280_ADDR,
                                     BME280_REG_CTRL_HUM, I2C_MEMADD_SIZE_8BIT,
                                     &ctrl_hum, 1) == HAL_OK) {
                bme->state = BME280_STATE_CONFIGURE;
            }
        } else if ((currentTick - bme->last_measurement_tick) >= read_interval) {
            // Rozpoczęcie pomiaru – ustaw CTRL_MEAS (0xF4) dla oversampling x1 i trybu forced (0x01)
            uint8_t ctrl_meas = (0x01 << 5) | (0x01 << 2) | 0x01; // osrs_t=1, osrs_p=1, mode=forced
            if (HAL_I2C_Mem_Write_IT(bme->hi2c, BME280_ADDR,
                                      BME280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT,
                                      &ctrl_meas, 1) == HAL_OK) {
                bme->state = BME280_STATE_TRIGGER_MEAS;
            }
        }
        break;

    case BME280_STATE_CONFIGURE:
        {
            // Ustawienie rejestru CONFIG (0xF5) – dla uproszczenia ustawiamy 0x00
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
        // W trybie forced pomiar rozpoczyna się natychmiast po zapisie CTRL_MEAS.
        bme->tick_start = ST_Ticks;
        bme->state = BME280_STATE_WAIT_CONVERSION;
        break;

    case BME280_STATE_WAIT_CONVERSION:
        // Oczekiwanie na konwersję – dla osrs=1 wystarczy około 10 ms.
        if ((currentTick - bme->tick_start) >= 10) {
            if (HAL_I2C_Mem_Read_IT(bme->hi2c, BME280_ADDR,
                                     BME280_REG_DATA_START, I2C_MEMADD_SIZE_8BIT,
                                     bme->raw.data, 8) == HAL_OK) {
                bme->state = BME280_STATE_READ_RAW;
            }
        }
        break;

    case BME280_STATE_READ_RAW:
        // Ten stan zostanie zmieniony przez callback RX po zakończeniu odczytu.
        break;

    case BME280_STATE_PROCESS:
        // Przetwarzanie danych – wywołujemy funkcję kompensacyjną.
        BME280_ProcessData(bme);
        // (Tutaj wynik pomiaru można zapisać do bufora kołowego – operacja krytyczna)
        bme->last_measurement_tick = currentTick;
        bme->state = BME280_STATE_IDLE;
        break;

    default:
        break;
    }
}

/* Struktury pomocnicze dla surowych danych i kalibracji */
struct bme280_uncomp_data {
    uint32_t temperature;
    uint32_t pressure;
    uint32_t humidity;
};

/**
 * @brief Kompensacja temperatury zgodnie z dokumentacją BME280.
 * Zwraca temperaturę w °C (double).
 */
static double compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = (((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0);
    var1 = var1 * ((double)calib_data->dig_t2);
    var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    calib_data->t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
        temperature = temperature_min;
    else if (temperature > temperature_max)
        temperature = temperature_max;
    return temperature;
}

/**
 * @brief Kompensacja ciśnienia zgodnie z dokumentacją BME280.
 * Zwraca ciśnienie w Pa (double).
 */
static double compensate_pressure(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;

    var1 = ((double)calib_data->t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)calib_data->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)calib_data->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)calib_data->dig_p4) * 65536.0);
    var3 = ((double)calib_data->dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)calib_data->dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_p1);

    if (var1 > 0.0)
    {
        pressure = 1048576.0 - (double)uncomp_data->pressure;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib_data->dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)calib_data->dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)calib_data->dig_p7)) / 16.0;

        if (pressure < pressure_min)
            pressure = pressure_min;
        else if (pressure > pressure_max)
            pressure = pressure_max;
    }
    else
        pressure = pressure_min;
    return pressure;
}

/**
 * @brief Kompensacja wilgotności zgodnie z dokumentacją BME280.
 * Zwraca wilgotność w %RH (double).
 */
static double compensate_humidity(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *calib_data)
{
    double humidity;
    double humidity_min = 0.0;
    double humidity_max = 100.0;
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    double var6;

    var1 = ((double)calib_data->t_fine) - 76800.0;
    var2 = (((double)calib_data->dig_h4) * 64.0 + (((double)calib_data->dig_h5) / 16384.0) * var1);
    var3 = uncomp_data->humidity - var2;
    var4 = ((double)calib_data->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)calib_data->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)calib_data->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)calib_data->dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max)
        humidity = humidity_max;
    else if (humidity < humidity_min)
        humidity = humidity_min;

    return humidity;
}

/**
 * @brief Przetwarzanie surowych danych BME280.
 *
 * Funkcja konwertuje odebrane surowe dane z rejestrów czujnika na wartości fizyczne
 * korzystając z algorytmów kompensacyjnych.
 */
void BME280_ProcessData(BME280_HandleTypeDef* bme) {
    struct bme280_uncomp_data uncomp;

    // Odczyt surowych danych (20-bitowe dla temperatury i ciśnienia, 16-bitowe dla wilgotności)
    uncomp.pressure = ((uint32_t)bme->raw.data[0] << 12) |
                      ((uint32_t)bme->raw.data[1] << 4) |
                      ((uint32_t)(bme->raw.data[2] >> 4));
    uncomp.temperature = ((uint32_t)bme->raw.data[3] << 12) |
                         ((uint32_t)bme->raw.data[4] << 4) |
                         ((uint32_t)(bme->raw.data[5] >> 4));
    uncomp.humidity = ((uint32_t)bme->raw.data[6] << 8) |
                      ((uint32_t)bme->raw.data[7]);

    // Kompensacja danych – wartości w double
    double temp = compensate_temperature(&uncomp, &bme->calib_data);
    double press = compensate_pressure(&uncomp, &bme->calib_data);
    double hum = compensate_humidity(&uncomp, &bme->calib_data);

    // Przypisanie wyników do struktury pomiaru (jako float, jeżeli aplikacja tego wymaga)
    bme->measurement.temperature = (float)temp;
    bme->measurement.pressure = (float)press;
    bme->measurement.humidity = (float)hum;
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
        bme->tick_start = ST_Ticks; // Używamy ST_Ticks
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
        // Opcjonalna obsługa błędów lub reset magistrali
        return;
    }
}
