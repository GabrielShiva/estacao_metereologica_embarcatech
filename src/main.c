#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"

#include "hardware/i2c.h"

#include "lwip/tcp.h"

// Inclusão dos módulos
#include "inc/button/button.h"
#include "inc/buzzer/buzzer.h"
#include "inc/display/ssd1306.h"
#include "inc/i2c_protocol/i2c_protocol.h"
#include "inc/led_matrix/ws2812b.h"
#include "inc/led_rgb/led.h"
#include "inc/sensors/aht20.h"
#include "inc/sensors/bmp280.h"

#include "secret.h" // Arquivo com os parâmetros para acessso de rede

#include "web_files/index_html.h" // Arquivo com o conteúdo da página web

// Definição de variáveis e macros importantes para o debounce dos botões
#define DEBOUNCE_TIME 260

static volatile uint32_t last_btn_a_press = 0;
static volatile uint32_t last_btn_b_press = 0;

// Define a página que é exibida no display
static volatile bool display_page = 0;

// Definições do servidor HTTP
struct http_state {
    char response[14000];
    size_t len;
    size_t sent;
};

// Definições de estrutura e variável que armazena os dados coletados pelos sensores
typedef struct sensors_data {
    float pressure;
    float temperature;
    float humidity;
    float altitude;
    float temperature_min;
    float temperature_max;
    float humidity_min;
    float humidity_max;
    float temperature_offset;
    float humidity_offset;
    float pressure_offset;
} sensors_data_t;

static sensors_data_t sensors_data = {0, 0, 0, 0, 10.0, 45.0, 20.0, 75.0, 0, 0, 0};

// Definição do protótipo das funções que serão criadas
void gpio_irq_handler(uint gpio, uint32_t events);
void checkLimitValues(sensors_data_t *sensors_data);
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static void http_server_init(void);

// Função principal do sistema
int main() {
    stdio_init_all();

    // Iniciallização dos botões
    btns_init();

    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BTN_A_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Inicialização do LED RGB
    leds_init();

    // Inicialização dos buzzers
    buzzer_setup(BUZZER_RIGHT_PIN);

    // Inicialização do barramento I2C sensores AHT20 e BMP280
    i2c_setup(I2C0_SDA, I2C0_SCL);

    // Inicializa o BMP280
    bmp280_setup(I2C0_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C0_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C0_PORT);
    aht20_setup(I2C0_PORT);

    //Inicialização do barramento I2C para o display
    i2c_setup(I2C1_SDA, I2C1_SCL);

    // Inicializa o display
    ssd1306_t ssd;
    ssd1306_setup(&ssd, WIDTH, HEIGHT, false, DISP_ADDR, I2C1_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data aht20_data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure_bmp;

    // Inicialização do módulo wifi
    if (cyw43_arch_init()) {
        printf("Falha ao inicializar a arquitetura CYW43\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 60000)) {
        printf("Falha ao conectar ao WiFi\n");
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    printf("---- INFORMACOES DO SERVIDOR ----\n");
    printf("IP: %s\n", ip_str);

    // Inicializa o servidor
    http_server_init();

    char str_tmp[5];
    char str_alt[5];
    char str_umi[5];
    char str_pres[5];

    bool color = true;
    while (true) {
        // Mantém o módulo wifi ativo
        cyw43_arch_poll();

        // Realiza leitura do AHT20
        aht20_read(I2C0_PORT, &aht20_data);
        sensors_data.humidity = aht20_data.humidity + sensors_data.humidity_offset;

        // Realiza leitura do BMP280
        bmp280_read_raw(I2C0_PORT, &raw_temp_bmp, &raw_pressure_bmp);
        sensors_data.temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        sensors_data.temperature = (sensors_data.temperature / 100.0) + sensors_data.temperature_offset;

        sensors_data.pressure = bmp280_convert_pressure(raw_pressure_bmp, raw_temp_bmp, &params);
        sensors_data.pressure = (sensors_data.pressure / 100.0) + sensors_data.pressure_offset;
        sensors_data.altitude = calculate_altitude(sensors_data.pressure * 100.0);

        // printf("Pressao: %.2f hPa\n", sensors_data.pressure);
        // printf("Temperatura: %.2f C\n", sensors_data.temperature);
        // printf("Altitude: %.2f m\n", sensors_data.altitude);
        // printf("Umidade: %.2f %%\n", sensors_data.humidity);

        if (display_page) {
            // Exibe os dados no display
            sprintf(str_tmp, "%.1f ºC", sensors_data.temperature);
            sprintf(str_alt, "%.0f m", sensors_data.altitude);
            sprintf(str_umi, "%.1f %%", sensors_data.humidity);
            sprintf(str_pres, "%.1f hPa", sensors_data.pressure);

            //  Atualiza o conteúdo do display com animações
            ssd1306_fill(&ssd, !color);
            ssd1306_rect(&ssd, 2, 2, 124, 62, true, false);
            ssd1306_draw_string(&ssd, "ESTACAO", 4, 6);
            ssd1306_draw_string(&ssd, "METEOROLOGICA", 4, 14);
            ssd1306_line(&ssd, 3, 23, 123, 23, true); // linha horizontal - primeira
            ssd1306_line(&ssd, 51, 23, 51, 63, true); // linha vertical
            ssd1306_draw_string(&ssd, "TEMP", 4, 25);
            sprintf(str_tmp, "%.1fC", sensors_data.temperature);
            ssd1306_draw_string(&ssd, str_tmp, 54, 25);
            ssd1306_draw_string(&ssd, "UMID", 4, 35);
            sprintf(str_umi, "%.1f%%", sensors_data.humidity);
            ssd1306_draw_string(&ssd, str_umi, 54, 35);
            ssd1306_draw_string(&ssd, "ALTI", 4, 45);
            sprintf(str_alt, "%.1fm", sensors_data.altitude);
            ssd1306_draw_string(&ssd, str_alt, 54, 45);
            ssd1306_draw_string(&ssd, "PRES", 4, 55);
            sprintf(str_pres, "%.1fhPa", sensors_data.pressure);
            ssd1306_draw_string(&ssd, str_pres, 54, 55);
            ssd1306_send_data(&ssd);
        } else {
            ssd1306_fill(&ssd, !color);
            ssd1306_rect(&ssd, 2, 2, 124, 62, true, false);
            ssd1306_draw_string(&ssd, "ESTACAO", 4, 6);
            ssd1306_draw_string(&ssd, "METEOROLOGICA", 4, 14);
            ssd1306_line(&ssd, 3, 23, 123, 23, true); // linha horizontal - primeira
            ssd1306_draw_string(&ssd, "IP", 4, 25);
            ssd1306_draw_string(&ssd, ip_str, 4, 33);
            ssd1306_send_data(&ssd);
        }

        // checa valores e liga/desliga alerta
        checkLimitValues(&sensors_data);

        sleep_ms(10);
    }
}

// Verifica os valores limites e emite os alertas
void checkLimitValues(sensors_data_t *data) {
    if ((data->temperature < data->temperature_min || data->temperature > data->temperature_max) ||
        (data->humidity < data->humidity_min || data->humidity > data->humidity_max)) {

        leds_turnoff();
        set_led_red();

        buzzer_play(BUZZER_RIGHT_PIN, 1000);
    } else {

        leds_turnoff();
        set_led_blue();
        buzzer_stop(BUZZER_RIGHT_PIN);
    }
}

// Função responsável por realizar o tratamento das interrupções geradas pelos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Muda a página exibida no display
    if (gpio == BTN_A_PIN && (current_time - last_btn_a_press > DEBOUNCE_TIME)) {
        last_btn_a_press = current_time;
        display_page = !display_page;
    // Coloca o raspberry no modo de BOOTSEL
    } else if (gpio == BTN_B_PIN && (current_time - last_btn_b_press > DEBOUNCE_TIME)) {
        last_btn_b_press = current_time;
        reset_usb_boot(0, 0);
    }
}

// Função responsável por enviar respostas via HTTP
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

// Função responsável por receber requisições via HTTP
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));

    if (!hs) {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }

    hs->sent = 0;

    printf("\n\n---- REQUISICAO CHEGOU ----\n\n");
    printf("%s\n", req);

    // Definição dos endpoints do servidor
    // O endpoint abaixo atualiza os valores limites dos sensores
    if (strstr(req, "GET /api/sensors/limits/update?")) {
        bool temperatureLimitsUpdated = false;
        bool humidityLimitsUpdated = false;

        printf("LIMITES CHEGARAM\n\n");

        // Obtém o conteúdo da requisição
        char *content = strstr(req, "\r\n\r\n");
        if (content) {
            content += 4;

            float temperature_min = 0, temperature_max = 0;
            float humidity_min = 0, humidity_max = 0;

            // Procura e extrai os valores da query string
            sscanf(req,
                "GET /api/sensors/limits/update?temperature_min=%f&temperature_max=%f&humidity_min=%f&humidity_max=%f",
                &temperature_min, &temperature_max, &humidity_min, &humidity_max
            );

            printf("%.2f %.2f %.2f %.2f \n\n", temperature_min, temperature_max, humidity_min, humidity_max);

            // Faz validação dos dados
            if (temperature_max >= 0 && temperature_max <= 100 && temperature_min >= 0 && temperature_min <= 100) {
                if (temperature_min < temperature_max) {
                    sensors_data.temperature_min = temperature_min;
                    sensors_data.temperature_max = temperature_max;
                    temperatureLimitsUpdated = true;
                }
            }

            // Faz validação dos dados
            if (humidity_max >= 0 && humidity_max <= 100 && humidity_min >= 0 && humidity_min <= 100) {
                if (humidity_min < humidity_max) {
                    sensors_data.humidity_min = humidity_min;
                    sensors_data.humidity_max = humidity_max;
                    humidityLimitsUpdated = true;
                }
            }
        }

        // Envia a mensagem para o cliente de acordo com o sucesso no armazenamento dos limites ou não
        if (temperatureLimitsUpdated && humidityLimitsUpdated) {
            printf("---- Limites definidos ----\n");
            printf("Temperatura Min=%d, Temperatura Max=%d\n", sensors_data.temperature_min, sensors_data.temperature_max);
            printf("Umidade Min=%d, Umidade Max=%d\n", sensors_data.humidity_min, sensors_data.humidity_max);

            const char *response_msg = "{\"status\":\"ok\",\"message\":\"Limites atualizados\"}";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: %d\r\n"
                "\r\n"
                "%s",
                (int)strlen(response_msg), response_msg
            );
        } else {
            printf("---- Falha ao atualizar os limites ----\n");

            const char *response_msg = "{\"status\":\"wrong\",\"message\":\"Falha ao atualizar os limites\"}";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 500 Internal Server Error\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: %d\r\n"
                "\r\n"
                "%s",
                (int)strlen(response_msg), response_msg
            );
        }

    // O endpoint abaixo retorna os dados do sensor para o cliente
    } else if (strstr(req, "GET /api/sensors/get-data")) {
        char json_data[500];
        snprintf(json_data, sizeof(json_data),
            "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"temperature_min\":%.2f,\"temperature_max\":%.2f,\"humidity_min\":%.2f,\"humidity_max\":%.2f}",
            sensors_data.temperature, sensors_data.humidity, sensors_data.pressure, sensors_data.altitude, sensors_data.temperature_min, sensors_data.temperature_max, sensors_data.humidity_min, sensors_data.humidity_max);

        printf("\n\n%s\n\n", json_data);
        // printf("Pressao: %.2f hPa\n", sensors_data.pressure);
        // printf("Temperatura: %.2f C\n", sensors_data.temperature);
        // printf("Altitude: %.2f m\n", sensors_data.altitude);
        // printf("Umidade: %.2f %%\n", sensors_data.humidity);

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n"
            "\r\n"
            "%s",
            (int)strlen(json_data), json_data
        );

    // O endpoint abaixo retorna a página html base
    } else if (strstr(req, "GET /api/sensors/offsets?")) {

        // Obtém o conteúdo da requisição
        char *content = strstr(req, "\r\n\r\n");
        if (content) {
            content += 4;

            // Definição de offsets
            float temperature_offset = 0.0, humidity_offset = 0.0, pressure_offset = 0.0;

            // Procura e extrai os valores da query string
            sscanf(req,
                "GET /api/sensors/offsets?temperature_offset=%f&humidity_offset=%f&pressure_offset=%f",
                &temperature_offset, &humidity_offset, &pressure_offset
            );

            printf("%.2f %.2f %.2f \n\n", temperature_offset, humidity_offset, pressure_offset);

            if (temperature_offset >= 0 && temperature_offset <= 100) {
                sensors_data.temperature_offset = temperature_offset;
            }

            if (humidity_offset >= 0 && humidity_offset <= 100) {
                sensors_data.humidity_offset = humidity_offset;
            }

            if (pressure_offset >= 0 && pressure_offset <= 100) {
                sensors_data.pressure_offset = pressure_offset;
            }

            const char *response_msg = "{\"status\":\"ok\",\"message\":\"Offsets atualizados\"}";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: %d\r\n"
                "\r\n"
                "%s",
                (int)strlen(response_msg), response_msg
            );
        }
    } else {
        printf("chegou aqui\n");
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            (int)strlen(index_html), index_html);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);

    printf("finalizou conexao\n");

    return ERR_OK;
}

// Função responsável por aceitar conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Função responsável por iniciar o servidor
static void http_server_init(void) {
    struct tcp_pcb *pcb = tcp_new();

    if (!pcb) {
        printf("Erro ao criar PCB TCP\n");
        return;
    }

    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }

    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}
