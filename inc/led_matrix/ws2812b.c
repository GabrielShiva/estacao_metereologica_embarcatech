#include "ws2812b.h"
#include "ws2812b.pio.h"

ws2812b_LED_t led_matrix[LED_MATRIX_SIZE];
PIO led_matrix_pio;
uint sm;

// Inicializa a máquina PIO para controle da matriz de LEDs.
void ws2812b_setup() {
    // Cria programa PIO.
    uint offset = pio_add_program(pio0, &led_matrix_program);
    led_matrix_pio = pio0;

    // Toma posse de uma máquina PIO.
    sm = pio_claim_unused_sm(led_matrix_pio, false);
    if (sm < 0)
    {
        led_matrix_pio = pio1;
        sm = pio_claim_unused_sm(led_matrix_pio, true); // Se nenhuma máquina estiver livre, panic!
    }

    // Inicia programa na máquina PIO obtida.
    led_matrix_program_init(led_matrix_pio, sm, offset, LED_MATRIX_PIN, 800000.f);

    // Limpa buffer de pixels.
    for (uint i = 0; i < LED_MATRIX_SIZE; ++i)
    {
        led_matrix[i].R = 0;
        led_matrix[i].G = 0;
        led_matrix[i].B = 0;
    }
}

// Atribui uma cor RGB a um LED.
void ws2812b_set_led(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    led_matrix[index].R = r;
    led_matrix[index].G = g;
    led_matrix[index].B = b;
}

// Limpa o buffer de pixels.
void ws2812b_clear() {
    for (uint i = 0; i < LED_MATRIX_SIZE; ++i)
        ws2812b_set_led(i, 0, 0, 0);
}

// Escreve os dados do buffer nos LEDs.
void ws2812b_write() {
    // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
    for (uint i = 0; i < LED_MATRIX_SIZE; ++i) {
        pio_sm_put_blocking(led_matrix_pio, sm, led_matrix[i].G<<24);
        pio_sm_put_blocking(led_matrix_pio, sm, led_matrix[i].R<<24);
        pio_sm_put_blocking(led_matrix_pio, sm, led_matrix[i].B<<24);
    }
    sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

// Desenha um ponto na matriz de LEDs.
void ws2812b_draw_point(uint8_t point_index, const uint8_t r, const uint8_t g, const uint8_t b) {
    ws2812b_set_led(point_index, r, g, b);

    // Atualiza a matriz de LEDs.
    ws2812b_write();
    sleep_us(100); // Espera 100us
}

// Preenche uma coluna da matriz de LEDs com uma cor específica.
void ws2812b_fill_column(uint8_t column, const uint8_t r, const uint8_t g, const uint8_t b) {
    if (column >= LED_MATRIX_COL) return;

    // Para uma matriz 5x5, mapeamento das posições na vertical:
    for (int row = 0; row < LED_MATRIX_ROW; row++) {
        int led_index;

        // Mapeamento correto para padrão snake
        if (row % 2 == 0) {
            // Linha par: esquerda para direita
            led_index = row * LED_MATRIX_ROW + column;
        } else {
            // Linha ímpar: direita para esquerda
            led_index = row * LED_MATRIX_ROW + (LED_MATRIX_ROW - 1 - column);
        }

        ws2812b_set_led(led_index, r, g, b);
    }
}

// Preenche uma linha da matriz de LEDs com uma cor específica.
void ws2812b_fill_row(uint8_t row, const uint8_t r, const uint8_t g, const uint8_t b) {
    if (row >= LED_MATRIX_ROW) return;

    // Para uma matriz 5x5, mapeamento das posições na horizontal:
    for (int col = 0; col < LED_MATRIX_COL; col++) {
        int led_index;

        // Mapeamento correto para padrão snake
        if (row % 2 == 0) {
            // Linha par: esquerda para direita
            led_index = row * LED_MATRIX_COL + col;
        } else {
            // Linha ímpar: direita para esquerda
            led_index = row * LED_MATRIX_COL + (LED_MATRIX_COL - 1 - col);
        }

        ws2812b_set_led(led_index, r, g, b);
    }
}

// Preenche toda a matriz de LEDs com uma cor específica.
void ws2812b_fill_matrix(const uint8_t r, const uint8_t g, const uint8_t b) {
    for (uint i = 0; i < LED_MATRIX_SIZE; ++i) {
        ws2812b_set_led(i, r, g, b);
    }

    // Atualiza a matriz de LEDs.
    ws2812b_write();
}
