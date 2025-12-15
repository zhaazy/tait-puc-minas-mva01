#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <DHT.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_master.h"
#include "lvgl.h"
#include "ads1115.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h" // Required for esp_restart()
#include <math.h>
#include <float.h>

// Pinos SPI do display
#define PIN_NUM_MISO   19
#define PIN_NUM_MOSI   23
#define PIN_NUM_SCK    18
#define PIN_NUM_CS     15
#define PIN_NUM_DC     2
//#define PIN_NUM_RST    4
#define LCD_H_RES      320
#define LCD_V_RES      240
#define LCD_HOST       SPI2_HOST

//static lv_obj_t *labels[11];
static lv_display_t *lvgl_disp;
static esp_lcd_panel_handle_t panel_handle = NULL;

// Estrutura para armazenar os ponteiros dos objetos da tela
typedef struct {
    lv_obj_t *main;
    lv_obj_t *obj0;
    lv_obj_t *humidity;
    lv_obj_t *temperature;
    lv_obj_t *atm_press;
    lv_obj_t *altitude;
    lv_obj_t *obj1;
    lv_obj_t *obj2;
    lv_obj_t *flow;
    lv_obj_t *volume;
    lv_obj_t *o2;
    lv_obj_t *obj3;
    lv_obj_t *frequencia;
    lv_obj_t *t_ins;
    lv_obj_t *t_exp;
    lv_obj_t *obj4;
    lv_obj_t *obj5;
    lv_obj_t *obj6;
    lv_obj_t *obj7;
    lv_obj_t *press_intern;
    lv_obj_t *p_max;
    lv_obj_t *peep;
    lv_obj_t *obj8;
    lv_obj_t *press_extern;
    lv_obj_t *press_high;
} objects_t;

// Instância global dos objetos
objects_t objects;

// Tag para logs
static const char *TAG = "BME280";
static const char *TAG2 = "AOF1010";
static const char *TAG3 = "DISPLAY";
//static const char *TAG4 = "SENSOR_DATA";

// Configurações do I2C
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TIMEOUT_MS   1000

// Configurações da UART
#define UART_SENSOR UART_NUM_2
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16
#define BUF_SIZE 128

// Config da UART do modulo bluetooth HC-05
#define UART_BT UART_NUM_1
#define UART_BT_TX_PIN GPIO_NUM_4
#define UART_BT_RX_PIN GPIO_NUM_5

// Comando padrão  do sensor de O2 (AOF1010)
const uint8_t query_cmd[] = { 0x11, 0x01, 0x01, 0xED };

// Endereço I2C padrão do BME280 e do AFM3000 e do ADS1115
#define BME280_ADDR             0x76
#define AFM3000_ADDR            0x40

// Configurações do ADS1115
#define ADS1115_ADDR_1           0x48
#define ADS1115_ADDR_2           0x49
ads1115_t ads_1;
ads1115_t ads_2;

// Configurações do AFM3000
#define AFM3000_CMD_MSB            0x10
#define AFM3000_CMD_LSB            0x00
#define AFM3000_OFFSET             32000
float AFM3000_COEFFICIENT = 140.0;

// Registros do BME280
#define BME280_REG_ID           0xD0
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_REG_CALIB_00     0x88
#define ALTITUDE_NIVEL_DO_MAR   1013.25f
#define TEMP_PADRAO_KELVIN      15 // 15°C (Temperatura Padrão ISA)

volatile int timer_flag_dht_bmp = 0;
volatile int timer_flag_o2 = 0;

// Pino do Sensor de humidade
#define DHT_GPIO            GPIO_NUM_32       // GPIO32 - Sensor DHT22
#define DHT_TYPE            DHT_TYPE_AM2301    // Define o tipo do sensor de umidade como DHT22

// Fatores de calibração dos sensores de pressão
#define CAL_INTERN  0.023991725
#define CAL_EXTERN     0.234953725
#define CAL_HIGH   0.000115

// Offsets (b)
float offset_intern = 0;
float offset_extern = 0;
float offset_high = 0;

// Variáveis de Resultado Globais
float press_high = 0;            // Entrada de pressão alta (externa) dada em "bar"
float press_extern = 0;              // Entrada de pressão baixa (exeterna) dada em "cmH2O"
float press_intern = 0;           // Pressão interna do circuito respiratório dada em "cmH2O"
float temperature = 0;            // Temperatura do ar ambiente dada em "°C"
float humidity = 0;               // Humidade relatativa do ar dada em "%"
float atm_press = 0;              // Pressão atmosferica dada em "hPa"
float altitude = 0;               // Altitude em relação ao nivel do mar
float o2 = 21;                     // Concentração de Oxigênio
float o2_flow = 0;                // Fluxo lido pelo sensor de O2 (AOF1010)
float o2_temperature = 0;         // Temperatura lida pelo sensor de O2 (AOF1010)
double flow = 0;                   // Valor do fluxo instantâneo
float volume = 0;                 // Volume em (mL)
float peep = 0;                   // PEPP em (cmH2O)
float p_max = 0;                  // Pressão interna de pico
float frequencia = 0;        // O valor final em BPM da frequencia respiratoria

// VARIÁVEIS PARA I:E
float tempo_insp_seg = 0.0; // Tempo Inspiratório (segundos)
float tempo_exp_seg = 0.0;  // Tempo Expiratório (segundos)
int64_t timestamp_inicio_exp = 0; // Novo: Guarda quando a expiração começou

// Variáveis Globais de calculo de volume/peep/frequencia e etc...
#define GATILHO_TROCA_FASE 1.0  // Auto Trigger: X acima da última PEEP
#define DEADZONE_FLUXO 1.0      // Ignora ruído abaixo de X LPM
#define TEMPO_MINIMO_INSP_MS 250 // Filtro para detectar inspiração corretamente
// FATOR DE AJUSTE FINO DO VOLUME
// Se o volume tiver baixo, aumente esse. (Ex: 1.05 = +5%, 1.10 = +10%)
// Isso corrige erros de amostragem do ESP32, não é "inventar" dados.
#define FATOR_CORRECAO_VOL 1.30

// Variáveis para a Média Móvel do volume
#define TAMANHO_MEDIA 15 // Média das últimas 5 respirações (Aumente para ficar mais estável)
float historico_volumes[TAMANHO_MEDIA] = {0}; // Inicializa tudo com 0
int indice_volume = 0;
bool buffer_cheio = false;
float tidal_volume_medio = 0.0;

// Variáveis Globais de calculo de volume/peep/frequencia e etc...
float rastreador_min = 100.0; 
float rastreador_max = 0.0;   
float peep_final = 0.0;
float pico_final = 0.0;
bool fase_inspiracao = false; 
float volume_acumulado = 0.0;
float tidal_volume_final = 0.0;
int64_t last_time_us = 0; // Para calcular o delta T
int64_t ultimo_inicio_insp_us = 0;
int64_t inicio_desta_insp_us = 0; // Guarda a hora exata que ESTA inspiração começou
static float last_fluxo = 0.0; // Guarda o fluxo anterior para o Trapézio


// Estrutura para os coeficientes de calibração da pressão
typedef struct {
    // Coeficientes de temperatura (necessários para cálculo de t_fine)
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    // Coeficientes de pressão
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bme280_calib_data_t;

bme280_calib_data_t calib;

// Esta função será chamada pelo sistema operacional de forma segura
static void lv_tick_task(void *arg) {
    lv_tick_inc(1); // Informa ao LVGL que passou 1ms
    timer_flag_dht_bmp++;
    timer_flag_o2++;
}

// Função para inicializar o barramento I2C
static esp_err_t i2c_master_init(void) {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &config));
    return i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);
}

// Configuração da UART
void config_uart() {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Configura UART0 com pinos padrão (TX=GPIO1, RX=GPIO3)
    uart_driver_install(UART_SENSOR, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_SENSOR, &uart_config);
    uart_set_pin(UART_SENSOR, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uint8_t data[BUF_SIZE];
}

// Inicia a UART do BT
void init_uart_bt(void)
{
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_BT, &config);

    // Aqui você escolhe os pinos!
    uart_set_pin(UART_BT,
                 UART_BT_TX_PIN,   // TX
                 UART_BT_RX_PIN,   // RX
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    uart_driver_install(UART_BT, 2048, 0, 0, NULL, 0);
}

//Função pra configurar o ADS1115
void config_ads1115(){

    // Configura a estrutura ADS1115 (Pressão Interna e Externa)
    ads_1 = ads1115_config(I2C_MASTER_NUM, ADS1115_ADDR_1);  // Endereço padrão 0x48 (Primeiro Sensor)

    ads1115_set_mode(&ads_1, ADS1115_MODE_CONTINUOUS);
    vTaskDelay(pdMS_TO_TICKS(10));
    ads1115_set_sps(&ads_1, ADS1115_SPS_128);         // 128 samples por segundo
    vTaskDelay(pdMS_TO_TICKS(10));
    ads1115_set_pga(&ads_1, ADS1115_FSR_0_256);       // ±0.256 V para alta resolução
    vTaskDelay(pdMS_TO_TICKS(10));
    ads1115_set_max_ticks(&ads_1, pdMS_TO_TICKS(50)); // tempo máximo de espera no I2C
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configura a estrutura do segundo ADS1115 (Pressão Alta)
    ads_2 = ads1115_config(I2C_MASTER_NUM, ADS1115_ADDR_2);  // Endereço padrão 0x49 (Segundo sensor)

    ads1115_set_mode(&ads_2, ADS1115_MODE_CONTINUOUS);
    vTaskDelay(pdMS_TO_TICKS(10));
    ads1115_set_sps(&ads_2, ADS1115_SPS_128);         // 128 samples por segundo
    vTaskDelay(pdMS_TO_TICKS(10));
    ads1115_set_pga(&ads_2, ADS1115_FSR_2_048);
    vTaskDelay(pdMS_TO_TICKS(10));
    ads1115_set_max_ticks(&ads_2, pdMS_TO_TICKS(50)); // tempo máximo de espera no I2C
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Função para Zerar os pensores de pressão
void zero_press(){

    printf("Aguardando para capturar offsets...");
    vTaskDelay(pdMS_TO_TICKS(100));

    //Obtem a leitura da pressao interna e externa por meio do ads1115
    // AIN2 - AIN3 → press_extern
    ads1115_set_mux(&ads_1, ADS1115_MUX_0_1);
    vTaskDelay(pdMS_TO_TICKS(100));
    offset_extern = ads1115_get_raw(&ads_1);

    // AIN0 - AIN1 → press_intern
    ads1115_set_mux(&ads_1, ADS1115_MUX_2_3);
    vTaskDelay(pdMS_TO_TICKS(100));
    offset_intern = ads1115_get_raw(&ads_1);

    // AIN0 - GND → press_hight
    ads1115_set_mux(&ads_2, ADS1115_MUX_0_GND);
    vTaskDelay(pdMS_TO_TICKS(100));
    offset_high = ads1115_get_raw(&ads_2);

    // Exibe os offsets
    printf("\nOffset HIGH: %.2f \n", offset_high);
    printf("Offset EXTERN: %.2f \n", offset_extern);
    printf("Offset INTERN: %.2f \n", offset_intern);
}

// Faz a leitura das pressões
void read_press(){

    //Obtem a leitura da pressao interna e externa por meio do ads1115

    // AIN2 - AIN3 → press_extern
    ads1115_set_mux(&ads_1, ADS1115_MUX_0_1);
    vTaskDelay(pdMS_TO_TICKS(100));
    press_extern = CAL_EXTERN * (ads1115_get_raw(&ads_1) - offset_extern);
    if(press_extern<=0){
        press_extern = 0;
    }
    
    // AIN0 - AIN1 → press_intern
    ads1115_set_mux(&ads_1, ADS1115_MUX_2_3);
    vTaskDelay(pdMS_TO_TICKS(100));
    press_intern = CAL_INTERN * (ads1115_get_raw(&ads_1) - offset_intern);
    if(press_intern<=0){
        press_intern = 0;
    }

    // AIN0 - GND → press_hight
    ads1115_set_mux(&ads_2, ADS1115_MUX_0_GND);
    vTaskDelay(pdMS_TO_TICKS(100));
    press_high = CAL_HIGH * (ads1115_get_raw(&ads_2) - offset_high);
    if(press_high<=0){
        press_high = 0;
    }
    
    // Debug print
    //printf("press_extern = %.3f | press_intern = %.3f \n", press_extern, press_intern);
    //vTaskDelay(pdMS_TO_TICKS(100));
}

// Função para ler e imprimir temperatura e umidade do DHT22
void read_dht22() {
    
    esp_err_t result = dht_read_float_data(DHT_TYPE, DHT_GPIO, &humidity, &temperature);

    if (result != ESP_OK) {
        printf("Falha ao ler DHT22! Código de erro: %d\n", result); 
    }
}

// Função para escrever um único byte em um registrador
esp_err_t bme280_write_reg(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Função para ler múltiplos bytes de um registrador
esp_err_t bme280_read_reg(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd); // reinicia para leitura
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Lê e armazena os dados de calibração da pressão
void bme280_read_calibration() {
    uint8_t calib_data[24];
    bme280_read_reg(BME280_REG_CALIB_00, calib_data, 24);

    calib.dig_P1 = (uint16_t)(calib_data[6]  | (calib_data[7]  << 8));
    calib.dig_P2 = (int16_t)(calib_data[8]  | (calib_data[9]  << 8));
    calib.dig_P3 = (int16_t)(calib_data[10] | (calib_data[11] << 8));
    calib.dig_P4 = (int16_t)(calib_data[12] | (calib_data[13] << 8));
    calib.dig_P5 = (int16_t)(calib_data[14] | (calib_data[15] << 8));
    calib.dig_P6 = (int16_t)(calib_data[16] | (calib_data[17] << 8));
    calib.dig_P7 = (int16_t)(calib_data[18] | (calib_data[19] << 8));
    calib.dig_P8 = (int16_t)(calib_data[20] | (calib_data[21] << 8));
    calib.dig_P9 = (int16_t)(calib_data[22] | (calib_data[23] << 8));
}

// --- Compensação da pressão --- Converte a leitura bruta da pressão para hPa
uint32_t bme280_compensate_pressure(int32_t adc_P, int32_t t_fine) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * calib.dig_P6;
    var2 = var2 + ((var1 * calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * calib.dig_P3) >> 8) + ((var1 * calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * calib.dig_P1 >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (calib.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (uint32_t)(p / 256);
}

// --- Compensação da temperatura (necessária para pressão) ---
int32_t bme280_compensate_t_fine(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
                    ((int32_t)calib.dig_T3)) >> 14;
    return var1 + var2;
}

// Função para configurar o BME280
void config_bme(){

    uint8_t id = 0;
    bme280_read_reg(BME280_REG_ID, &id, 1);
    ESP_LOGI(TAG, "ID do BME280: 0x%X", id);

    // Configura modo de operação: temperatura e pressão em oversampling x1, modo normal
    bme280_write_reg(BME280_REG_CTRL_MEAS, 0x27);

    // Lê calibração
    bme280_read_calibration();
}

// Faz a leitura da pressão atmosferica
void read_atm_press(){

    uint8_t data[3];
    bme280_read_reg(BME280_REG_PRESS_MSB, data, 3);

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

    int32_t t_fine = (int32_t)(temperature * 5120.0f);  // Temperatura vinda do DHT22
    uint32_t pressure = bme280_compensate_pressure(adc_P, t_fine);

    atm_press = pressure / 100.0f;
}

// Calcula a Altitude Aproximada Baseado na Pressão
void calc_altitude() {
    altitude = ((-29271.0f * (TEMP_PADRAO_KELVIN + 273.15f))*(log(atm_press / ALTITUDE_NIVEL_DO_MAR))) / 1000.0f;
}

// AFM3000 CRC calculation routine
uint8_t afm3000_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x131;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// Faz a leitura do fluxo instantaneo
void read_flow() {
    uint8_t tx_buf[2] = {AFM3000_CMD_MSB, AFM3000_CMD_LSB};
    uint8_t rx_buf[3];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AFM3000_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, tx_buf, 2, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(2));  // Pequeno delay para coleta de dados

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AFM3000_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, rx_buf, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        printf("Erro na leitura do AFM3000\n");
    }

    if (afm3000_crc(rx_buf, 2) != rx_buf[2]) {
        printf("CRC inválido no dado do AFM3000\n");
    }

    uint16_t raw_flow = (rx_buf[0] << 8) | rx_buf[1];   // Pega o valor bruto do fluxo e converte pra fluxo padrão

    // Aplica uma correção linear no coeficiente do fluxo baseado na concentração de O2
    // Quando 21% = 140.0f e quando 100% = 142.8f de acordo com fabricante
    AFM3000_COEFFICIENT = 140.0f + ((o2 - 21.0) * 0.035443);

    // Pega o valor bruto (raw) e converte para fluxo padrão em lpm
    flow = ((int32_t)raw_flow - AFM3000_OFFSET) / AFM3000_COEFFICIENT;

    // Faz uma correção no fluxo baseado em uma comparação com o AVM100 da NEOS
    const float A =  0.0000023844f;   // x^3
    const float B = -0.0005316139f;   // x^2
    const float C =  1.0601057544f;   // x^1
    const float D = -0.6865913757f;   // x^0
    flow = ((A * flow + B) * flow + C) * flow + D;

    // Converte STPD (20C, 1013mbar) para BTPS (37C, saturado)
    flow = flow * 1.128f; 
    
}

// Função de Leitura do O2 (AOF1010)
void read_aof1010(){
    
    // Envia comando de consulta
    uart_write_bytes(UART_SENSOR, (const char *)query_cmd, sizeof(query_cmd));
    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeno delay para aguardar resposta

    // Lê resposta (esperado: 12 bytes)
    uint8_t data[BUF_SIZE];
    int len = uart_read_bytes(UART_SENSOR, data, 12, pdMS_TO_TICKS(500));
    if (len == 12 && data[0] == 0x16) {
        // Verifica CRC
        uint8_t crc = 0;
        for (int i = 0; i < 11; i++) {
            crc += data[i];
        }
        crc = 256 - crc;

        if (crc == data[11]) {
            uint16_t conc_raw = (data[3] << 8) | data[4];
            uint16_t flow_raw = (data[5] << 8) | data[6];
            uint16_t temp_raw = (data[7] << 8) | data[8];
            o2 = conc_raw / 10.0f;
            o2_flow = flow_raw / 10.0f;
            o2_temperature = temp_raw / 10.0f;

            const float c3 = -0.00012957718f;
            const float c2 =  0.0219126938f;
            const float c1 =  0.102000118f;
            const float c0 = 10.6066941f;

            // Corrige o valor do O2 conforme calibração comparando com o LUFT da Arkmeds
            o2 = ((c3 * o2 + c2) * o2 + c1) * o2 + c0;

            // Limites físicos
            if (o2 < 21.0f)  o2 = 21.0f;
            if (o2 > 100.0f) o2 = 100.0f;

            //printf("\nO2 Flow: %.2f \n", o2_flow);


        } else {
            ESP_LOGW(TAG2, "CRC inválido. Recebido: 0x%02X, Esperado: 0x%02X", data[11], crc);
            uart_flush(UART_SENSOR);
        }
    } else {
        ESP_LOGW(TAG2, "Resposta inválida ou timeout (len=%d)", len);
        uart_flush(UART_SENSOR);
    }
}

// ========= FUNÇÃO FLUSH (LVGL -> ILI9341) =========
static void my_flush_cb(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp_drv);
}

// ========== CRIA UI ==========
void create_ui(void) {
    
    // Configura Tema Padrão
    // NOTA: Em LVGL v9, usa-se lv_display_t em vez de lv_disp_t
    lv_display_t *dispp = lv_display_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_display_set_theme(dispp, theme);

    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 320, 240);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            objects.obj0 = obj;
            lv_obj_set_pos(obj, 5, 184);
            lv_obj_set_size(obj, 308, 50);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_width(obj, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_color(obj, lv_color_hex(0xffe0e0e0), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    // humidity
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.humidity = obj;
                    lv_obj_set_pos(obj, -7, -6);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Humidade: 00.0%");
                }
                {
                    // temperature
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.temperature = obj;
                    lv_obj_set_pos(obj, -7, 10);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Temperatura: 00.0 C");
                }
                {
                    // atm_press
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.atm_press = obj;
                    lv_obj_set_pos(obj, 134, 10);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Pressao Atm: 000.0 hpa");
                }
                {
                    // altitude
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.altitude = obj;
                    lv_obj_set_pos(obj, 134, -6);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Altitude: 000.0 m");
                }
            }
        }
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            objects.obj1 = obj;
            lv_obj_set_pos(obj, 7, 37);
            lv_obj_set_size(obj, 306, 70);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.obj2 = obj;
                    lv_obj_set_pos(obj, 15, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_decor(obj, LV_TEXT_DECOR_UNDERLINE, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Leituras Principais");
                }
                {
                    // flow
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.flow = obj;
                    lv_obj_set_pos(obj, -9, 6);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Fluxo: 000.0 l/min");
                }
                {
                    // volume
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.volume = obj;
                    lv_obj_set_pos(obj, -10, 21);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Volume: 0000.0 ml");
                }
                {
                    // o2
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.o2 = obj;
                    lv_obj_set_pos(obj, -10, 35);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "O2: 00.0 %");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.obj3 = obj;
                    lv_obj_set_pos(obj, 180, -12);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_decor(obj, LV_TEXT_DECOR_UNDERLINE, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Tempos");
                }
                {
                    // frequencia
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.frequencia = obj;
                    lv_obj_set_pos(obj, 132, 6);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Frequencia: 00.0 rpm");
                }
                {
                    // t_ins
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.t_ins = obj;
                    lv_obj_set_pos(obj, 131, 20);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Tempo Expiratorio: 1.0 s");
                }
                {
                    // t_exp
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.t_exp = obj;
                    lv_obj_set_pos(obj, 131, 35);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Tempo Inspiratorio: 1.0 s");
                }
            }
        }
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            objects.obj4 = obj;
            lv_obj_set_pos(obj, -207, 133);
            lv_obj_set_size(obj, 151, 81);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff121212), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.obj5 = obj;
                    lv_obj_set_pos(obj, 39, -14);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xff0048ff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_decor(obj, LV_TEXT_DECOR_UNDERLINE, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Tempos");
                }
            }
        }
        {
            lv_obj_t *obj = lv_obj_create(parent_obj);
            objects.obj6 = obj;
            lv_obj_set_pos(obj, 5, 111);
            lv_obj_set_size(obj, 308, 68);
            lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_border_opa(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff000000), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
            {
                lv_obj_t *parent_obj = obj;
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.obj7 = obj;
                    lv_obj_set_pos(obj, 10, -13);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_decor(obj, LV_TEXT_DECOR_UNDERLINE, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Pressao Interna");
                }
                {
                    // press_intern
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.press_intern = obj;
                    lv_obj_set_pos(obj, -7, 4);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Corrente: 0.0 cmH2O");
                }
                {
                    // p_max
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.p_max = obj;
                    lv_obj_set_pos(obj, -7, 18);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Pico: 0.0 cmH2O");
                }
                {
                    // peep
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.peep = obj;
                    lv_obj_set_pos(obj, -7, 33);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "PEEP: 0.0 cmH2O");
                }
                {
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.obj8 = obj;
                    lv_obj_set_pos(obj, 158, -11);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_decor(obj, LV_TEXT_DECOR_UNDERLINE, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Pressao Externa");
                }
                {
                    // press_extern
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.press_extern = obj;
                    lv_obj_set_pos(obj, 133, 9);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Baixa: 0.0 cmH2O");
                }
                {
                    // press_high
                    lv_obj_t *obj = lv_label_create(parent_obj);
                    objects.press_high = obj;
                    lv_obj_set_pos(obj, 134, 26);
                    lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                    lv_obj_set_style_text_font(obj, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_obj_set_style_text_color(obj, lv_color_hex(0xffffffff), LV_PART_MAIN | LV_STATE_DEFAULT);
                    lv_label_set_text(obj, "Alta: 0.0 bar");
                }
            }
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 92, -4);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_label_set_text(obj, "MVA01");
        }
    }

    // Carrega a tela (FIX PARA LVGL v9: lv_screen_load em vez de lv_scr_load)
    lv_screen_load(objects.main);

    ESP_LOGW(TAG3, "Interface Criada com Sucesso!");
}

// Função para iniciar o display tft
void esp_lcd_init() {

    esp_lcd_panel_io_handle_t io_handle = NULL;
    void* disp;
    spi_bus_config_t buscfg = {
        .intr_flags = ESP_INTR_FLAG_IRAM,
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES,
    };
    spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 40000000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
    };
    esp_lcd_new_panel_io_spi(LCD_HOST, &io_config, &io_handle);

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = 0,
        .bits_per_pixel = 16,
    };
    esp_lcd_new_panel_ili9341((io_handle), &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    //esp_lcd_panel_mirror(panel_handle, true, false);
    // Gira o display (troca eixos X e Y)
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // Inicializa LVGL
    lv_init();

    // Cria display LVGL
    lvgl_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(lvgl_disp, my_flush_cb);

    size_t buf_size_bytes = LCD_H_RES * 80 * sizeof(lv_color_t);
    lv_color_t *buf1 = heap_caps_malloc(buf_size_bytes, MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(buf_size_bytes, MALLOC_CAP_DMA);

    lv_display_set_buffers(
        lvgl_disp,
        buf1,
        buf2,
        buf_size_bytes,
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    // Configuração do Timer de Alta Resolução
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    // Inicia o timer para disparar a cada 1000 microsegundos (1ms)
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000));

    ESP_LOGW(TAG3, "Display Iniciado com Sucesso!");

    // Cria interface
    create_ui();
}

// Esscreve os Dados no Display
void esp_lcd_write() {
    char buffer[64];

    if(objects.press_high) {
        snprintf(buffer, sizeof(buffer), "Alta: %.1f bar", press_high);
        lv_label_set_text(objects.press_high, buffer);
    }

    if(objects.press_extern) {
        snprintf(buffer, sizeof(buffer), "Baixa: %.1f cmH2O", press_extern);
        lv_label_set_text(objects.press_extern, buffer);
    }

    if(objects.press_intern) {
        snprintf(buffer, sizeof(buffer), "Corrente: %.1f cmH2O", press_intern);
        lv_label_set_text(objects.press_intern, buffer);
    }

    if(objects.temperature) {
        snprintf(buffer, sizeof(buffer), "Temperatura: %.1f C", temperature);
        lv_label_set_text(objects.temperature, buffer);
    }

    if(objects.humidity) {
        snprintf(buffer, sizeof(buffer), "Humidade: %d %%", (int)humidity);
        lv_label_set_text(objects.humidity, buffer);
    }

    if(objects.atm_press) {
        snprintf(buffer, sizeof(buffer), "Pressao Atm: %.1f hpa", atm_press);
        lv_label_set_text(objects.atm_press, buffer);
    }

    if(objects.altitude) {
        snprintf(buffer, sizeof(buffer), "Altitude: %.1f m", altitude);
        lv_label_set_text(objects.altitude, buffer);
    }

    if(objects.o2) {
        snprintf(buffer, sizeof(buffer), "O2: %.1f %%", o2);
        lv_label_set_text(objects.o2, buffer);
    }

    if(objects.flow) {
        snprintf(buffer, sizeof(buffer), "Fluxo: %.2f l/min", flow);
        lv_label_set_text(objects.flow, buffer);
    }

    if(objects.volume) {
        snprintf(buffer, sizeof(buffer), "Volume: %.1f ml", volume);
        lv_label_set_text(objects.volume, buffer);
    }

    if(objects.peep) {
        snprintf(buffer, sizeof(buffer), "PEEP: %.1f cmH2O", peep);
        lv_label_set_text(objects.peep, buffer);
    }

    
    if(objects.p_max) {
        snprintf(buffer, sizeof(buffer), "Pico: %.1f cmH2O", p_max); // p_max
        lv_label_set_text(objects.p_max, buffer);
    }
    if(objects.frequencia) {
        snprintf(buffer, sizeof(buffer), "Frequencia: %.0f rpm", frequencia);
        lv_label_set_text(objects.frequencia, buffer);
    }
    if(objects.t_ins) {
        snprintf(buffer, sizeof(buffer), "Tempo Inspiratorio: %.1f s", tempo_insp_seg);
        lv_label_set_text(objects.t_ins, buffer);
    }
    if(objects.t_exp) {
        snprintf(buffer, sizeof(buffer), "Tempo Expiratorio: %.1f s", tempo_exp_seg);
        lv_label_set_text(objects.t_exp, buffer);
    }
    

    lv_timer_handler();  // Atualiza a tela

    //ESP_LOGW(TAG3, "Display Atualizado com Sucesso!");
}

// Função pra calcular PEEP, Pressão de Pico, Volume e Frequencia
void calcular_ciclo_completo(float pressao_atual, float fluxo_atual) {
    
    int64_t now_us = esp_timer_get_time();
    if (last_time_us == 0) last_time_us = now_us;
    float dt_segundos = (now_us - last_time_us) / 1000000.0;
    last_time_us = now_us;

    // --- 1. DETECÇÃO DE INSPIRAÇÃO ---
    if (!fase_inspiracao && pressao_atual > (rastreador_min + GATILHO_TROCA_FASE)) {
        
        peep_final = rastreador_min; 
        
        // BPM
        if (ultimo_inicio_insp_us != 0) {
            float tempo_ciclo = (now_us - ultimo_inicio_insp_us) / 1000000.0;
            if (tempo_ciclo > 0.3) frequencia = 60.0 / tempo_ciclo;
        }
        ultimo_inicio_insp_us = now_us;
        inicio_desta_insp_us = now_us; 

        // I:E
        if (timestamp_inicio_exp != 0) {
             float t_exp = (now_us - timestamp_inicio_exp) / 1000000.0;
             if (t_exp > 0.1) tempo_exp_seg = t_exp;
        }

        volume_acumulado = 0.0; 
        fase_inspiracao = true;
        rastreador_max = pressao_atual; 
        last_fluxo = 0.0; // Reseta fluxo anterior
    }
    
    // --- 2. DETECÇÃO DE EXPIRAÇÃO ---
    else if (fase_inspiracao && pressao_atual < (rastreador_max - GATILHO_TROCA_FASE)) {
        
        int64_t duracao_atual_us = now_us - inicio_desta_insp_us;
        float duracao_atual_ms = duracao_atual_us / 1000.0;

        if (duracao_atual_ms >= TEMPO_MINIMO_INSP_MS) {
            
            tempo_insp_seg = duracao_atual_us / 1000000.0;
            timestamp_inicio_exp = now_us;
            pico_final = rastreador_max;
            
            if (volume_acumulado > 20.0) { 
                
                // Aplica Fator de Correção
                float vol_final = volume_acumulado * FATOR_CORRECAO_VOL;

                // Média Móvel
                historico_volumes[indice_volume] = vol_final;
                indice_volume++;
                if (indice_volume >= TAMANHO_MEDIA) {
                    indice_volume = 0;
                    buffer_cheio = true;
                }
                
                float soma = 0;
                int qtd = buffer_cheio ? TAMANHO_MEDIA : indice_volume;
                if (qtd == 0) qtd = 1;
                for (int i = 0; i < TAMANHO_MEDIA; i++) soma += historico_volumes[i];
                
                if (buffer_cheio) tidal_volume_medio = soma / TAMANHO_MEDIA;
                else tidal_volume_medio = soma / qtd;
            }
            
            fase_inspiracao = false;
            rastreador_min = pressao_atual;
        }
    }
    
    // --- 3. INTEGRAÇÃO (REGRA DO TRAPÉZIO) ---
    if (fase_inspiracao) {
        // Se o fluxo atual OU o anterior forem maiores que a deadzone, integra.
        // Isso captura melhor o momento que o fluxo começa a cair.
        if (fluxo_atual > DEADZONE_FLUXO || last_fluxo > DEADZONE_FLUXO) {
            
            // Trapézio: Média do fluxo atual com o anterior
            float fluxo_medio_lpm = (fluxo_atual + last_fluxo) / 2.0;
            
            // Converte para mL/s
            float fluxo_ml_seg = (fluxo_medio_lpm * 1000.0) / 60.0;
            
            volume_acumulado += fluxo_ml_seg * dt_segundos;
        }
        // Guarda o fluxo atual para ser o "anterior" na próxima rodada
        last_fluxo = fluxo_atual;
    }
    
    // --- 4. RASTREADORES ---
    if (fase_inspiracao) {
        if (pressao_atual > rastreador_max) rastreador_max = pressao_atual;
    } else {
        if (pressao_atual < rastreador_min) rastreador_min = pressao_atual;
    }
}

// Task do display
void display_task(void *arg) {
    while(1){
        esp_lcd_write();
        vTaskDelay(pdMS_TO_TICKS(10)); // Espera 10ms
    }
}

// Task da leitura dos sensores de o2, DHT e BME
void sensor_task(void *arg) {
    while(1){
        if(timer_flag_o2 >= 1000){
            timer_flag_o2 = 0;
            read_aof1010();
        }
        if (timer_flag_dht_bmp >= 5000) {
            timer_flag_dht_bmp = 0;
            read_dht22();    // Faz a leitura do DHT22 (temperatura e umidade)
            read_atm_press();   // Faz a leitura da pressão atmosferica (BME280)
            calc_altitude();  // Pressão padrão ao nível do mar
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task para chamar a leitura e calcular os parametros do ciclo
void calcular_ciclo_task (void){
    zero_press();          // Realiza o zero dos sensores de pressão
    while(1){
        read_flow();
        read_press();

        calcular_ciclo_completo(press_intern, flow);
        peep = peep_final;
        p_max = pico_final;
        volume = tidal_volume_medio;

       //vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Taks para Envio das leituras para o BT
void send_bt_task(void *pvParameters)
{
    char buffer[256];

    while (1)
    {
        // 1. Formata a string
        // O '\n' no final é OBRIGATÓRIO para o App Inventor não se perder
        int len = snprintf(buffer, sizeof(buffer),
                "%.1f;%.1f;%.1f;%.1f;%.0f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.0f;%.1f;%.1f\n",
                press_high,
                press_extern,
                press_intern,
                temperature,
                humidity,
                atm_press,
                altitude,
                o2,
                flow,peep,p_max,volume,frequencia,tempo_insp_seg,tempo_exp_seg);

        // 2. Envia para o HC-05 via UART
        uart_write_bytes(UART_BT, buffer, len);

        vTaskDelay(pdMS_TO_TICKS(50));
        
    }
}

// Função Principal
void app_main(void) {

    esp_lcd_init();        // Inicializa o display tft
    i2c_master_init();     // Configura o I2C
    config_uart();         // Configura a UART
    init_uart_bt();
    config_bme();          // Configura o sensor de humidade e temperatura
    config_ads1115();      // Configura o ADS1115

    // Faz a leitura inicial dos sensores
    zero_press();          // Realiza o zero dos sensores de pressão
    read_dht22();    // Faz a leitura do DHT22 (temperatura e umidade)
    read_atm_press();   // Faz a leitura da pressão atmosferica (BME280)
    calc_altitude();  // Pressão padrão ao nível do mar
    read_aof1010();

    // Cria as tasks
    xTaskCreatePinnedToCore(display_task, "DisplayTask", 4096, NULL, 20, NULL, 0);
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(calcular_ciclo_task, "CalcularCicloTask", 4096, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(send_bt_task, "send_bt_task", 4096, NULL, 15, NULL, 1);
}