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

#define DEBOUNCE_TIME 260

static volatile uint32_t last_btn_a_press = 0;
static volatile uint32_t last_btn_b_press = 0;

// Definições do servidor HTTP
struct http_state {
    char response[10000];
    size_t len;
    size_t sent;
};

// Definições de estrutura que armazena os dados coletados pelos sensores
typedef struct weather_station {
    float pressure;
    float temperature;
    float humidity;
    float altitude;
    float temperature_min;
    float temperature_max;
    float humidity_min;
    float humidity_max;
} weather_station_t;

void gpio_irq_handler(uint gpio, uint32_t events);
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static void start_http_server(void);

int main() {
    stdio_init_all();

    sleep_ms(5500);

    // Iniciallização dos botões
    btns_init();

    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BTN_A_PIN, GPIO_IRQ_EDGE_RISE, true);

    // Inicialização do LED RGB
    leds_init();

    // Inicialização dos buzzers
    buzzer_setup(BUZZER_RIGHT_PIN, 4.0f);

    // Inicialização do barramento I2C sensores AHT20 e BMP280
    i2c_setup(I2C0_SDA, I2C0_SCL);

    // Inicialização do sensor AHT20
    aht20_reset(I2C0_PORT);
    aht20_setup(I2C0_PORT);

    // Inicialização do sensor BMP280
    bmp280_setup(I2C0_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C0_PORT, &params);

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
    printf("IP DO SERVIDOR: %s\n", ip_str);
    start_http_server();

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}

// Função responsável por realizar o tratamento das interrupções geradas pelos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (gpio == BTN_A_PIN && (current_time - last_btn_a_press > DEBOUNCE_TIME)) { // Muda a página exibida no display
        last_btn_a_press = current_time;
    } else if (gpio == BTN_B_PIN && (current_time - last_btn_b_press > DEBOUNCE_TIME)) { // Coloca o raspberry no modo de BOOTSEL
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

    if (!p)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs)
    {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    if (strstr(req, "POST /api/limits"))
    {
        char *body = strstr(req, "\r\n\r\n");
        if (body)
        {
            body += 4;

            // Extrai os valores diretamente usando sscanf
            int max_val, min_val;
            if (sscanf(body, "{\"max\":%d,\"min\":%d", &max_val, &min_val) == 2)
            {
                // Valida os valores recebidos
                if (max_val >= 0 && max_val <= 100 && min_val >= 0 && min_val <= 100)
                {
                    weather_data.maxTemperature = max_val;
                    weather_data.minTemperature = min_val;
                }
            }
        }

        printf("Novos limites: Max=%d, Min=%d\n",
               weather_data.maxTemperature,
               weather_data.minTemperature);

        // Confirma atualização
        const char *txt = "Limites atualizados";
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/plain\r\n"
                           "Content-Length: %d\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(txt), txt);
    }
    else if (strstr(req, "GET /api/weather"))
    {
        // Responde com os dados atuais
        /* printf("Altura: %.2f m\n", weather_data.altitude);
        printf("Temperatura: %.2f C\n", weather_data.temperature);
        printf("Umidade: %.2f %%\n", weather_data.humidity);
        printf("Pressão: %.2f kPa\n", weather_data.pressure);
        printf("Min Temp: %d C, Max Temp: %d C\n",
               weather_data.minTemperature, weather_data.maxTemperature); */

        char json_data[256];
        snprintf(json_data, sizeof(json_data),
                 "{\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"minTemperature\":%d,\"maxTemperature\":%d}",
                 weather_data.temperature, weather_data.humidity, weather_data.pressure, weather_data.altitude, weather_data.minTemperature, weather_data.maxTemperature);

        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(json_data), json_data);
    }
    else
    {
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(html_data), html_data);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}

// Função responsável por aceitar conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Função responsável por iniciar o servidor
static void start_http_server(void) {
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

