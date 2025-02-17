#include <stdio.h>                          // Biblioteca padrão de entrada e saída
#include <string.h>                         // Biblioteca padrão do C para manipulação de strings.
#include <ctype.h>                          // Biblioteca para manipulação de caracteres
#include "pico/stdlib.h"                    // Biblioteca padrão para o Raspberry Pi Pico
#include <hardware/pio.h>                   // Biblioteca para manipulação de periféricos PIO
#include "hardware/clocks.h"                // Biblioteca para controle de relógios do hardware
#include "ws2818b.pio.h"                    // Biblioteca PIO para controle de LEDs WS2818B
#include "hardware/adc.h"                   // Biblioteca para controle do ADC (Conversor Analógico-Digital)
#include "pico/time.h"                      // Biblioteca para gerenciamento de temporizadores e alarmes.
#include "hardware/i2c.h"                   // Biblioteca para comunicação I2C.
#include "inc/ssd1306.h"                    // Biblioteca para controle do display OLED SSD1306.
#include "inc/font.h"                       // Biblioteca para uso de fontes personalizadas.
#include "hardware/pwm.h"                   // Biblioteca para controle de PWM.

// Definições de constantes
#define LED_COUNT 25                        // Número de LEDs na matriz
#define LED_PIN 7                           // Pino GPIO conectado aos LEDs
#define I2C_PORT i2c1
#define I2C_SDA 14                          // Pinos GPIO para comunicação I2C
#define I2C_SCL 15                          // Pinos GPIO para comunicação I2C
#define endereco 0x3C                       // Endereço I2C do display OLED
#define JOYSTICK_X 26                       // Pino GPIO para o eixo X do joystick
#define JOYSTICK_Y 27                       // Pino GPIO para o eixo Y do joystick

const uint LED_VERDE = 11;                  // Define o pino GPIO 11 para controlar a cor verde do LED RGB.
const uint LED_AZUL = 12;                   // Define o pino GPIO 12 para controlar a cor azul do LED RGB.
const uint LED_VERMELHO = 13;               // Define o pino GPIO 13 para controlar a cor vermelha do LED RGB.
const uint button_A = 5;                    // GPIO do botão A.
const uint button_B = 6;                    // GPIO do botão B
const uint button_joy = 22;                 // GPIO do botão Joystick

static volatile uint32_t last_time = 0;     // Armazena o tempo do último evento (em microssegundos)
static volatile bool flag_button = 0;       // Armazena o estado do botão
uint32_t elapsed_time = 10000;              // Armazena o tempo decorrido em microsegundos (Padrão: 10s)
static int32_t set_button = 0;              // Controlador de seleção das frases
static bool leds_on = true;                 // Variável para armazenar o estado dos LEDs

// Estrutura para representar um pixel com componentes RGB
struct pixel_t { 
    uint8_t G, R, B;                        // Componentes de cor: Verde, Vermelho e Azul
};

typedef struct pixel_t pixel_t;             // Alias para a estrutura pixel_t
typedef pixel_t npLED_t;                    // Alias para facilitar o uso no contexto de LEDs

npLED_t leds[LED_COUNT];                    // Array para armazenar o estado de cada LED
PIO np_pio;                                 // Variável para referenciar a instância PIO usada
uint sm;                                    // Variável para armazenar o número do state machine usado

// Função para inicializar o PIO para controle dos LEDs
void npInit(uint pin) 
{
    uint offset = pio_add_program(pio0, &ws2818b_program);      // Carregar o programa PIO
    np_pio = pio0;                                              // Usar o primeiro bloco PIO

    sm = pio_claim_unused_sm(np_pio, false);                    // Tentar usar uma state machine do pio0
    if (sm < 0)                                                 // Se não houver disponível no pio0
    {
        np_pio = pio1;                                          // Mudar para o pio1
        sm = pio_claim_unused_sm(np_pio, true);                 // Usar uma state machine do pio1
    }

    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);    // Inicializar state machine para LEDs

    for (uint i = 0; i < LED_COUNT; ++i)                        // Inicializar todos os LEDs como apagados
    {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

// Função para definir a cor de um LED específico
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) 
{
    leds[index].R = r;                                          // Definir componente vermelho
    leds[index].G = g;                                          // Definir componente verde
    leds[index].B = b;                                          // Definir componente azul
}

// Função para limpar (apagar) todos os LEDs
void npClear() 
{
    for (uint i = 0; i < LED_COUNT; ++i)                        // Iterar sobre todos os LEDs
        npSetLED(i, 0, 0, 0);                                   // Definir cor como preta (apagado)
}

// Função para atualizar os LEDs no hardware
void npWrite() 
{
    for (uint i = 0; i < LED_COUNT; ++i)                        // Iterar sobre todos os LEDs
    {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);             // Enviar componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].R);             // Enviar componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].B);             // Enviar componente azul
    }
}

// Função para inicializar o joystick
void joystickInit() 
{
    adc_init();                                                 // Inicializar o ADC
    adc_gpio_init(JOYSTICK_X);                                  // Configurar GPIO para eixo X
    adc_gpio_init(JOYSTICK_Y);                                  // Configurar GPIO para eixo Y
}

// Função para obter a posição do joystick e mapear para o índice do LED
int getJoystickLEDIndex(uint16_t *x_value, uint16_t *y_value) 
{
    adc_select_input(1);                                        // Selecionar o ADC para o eixo Y
    *y_value = adc_read();                                      // Ler valor do eixo Y

    adc_select_input(0);                                        // Selecionar o ADC para o eixo X
    *x_value = adc_read();                                      // Ler valor do eixo X

    // Calcular a intensidade do LED azul com base na posição Y
    int16_t blue_intensity = (int16_t)(*y_value) - 2048;
    if (blue_intensity < 0) blue_intensity = -blue_intensity;
    blue_intensity = blue_intensity * 255 / 2048;

    // Calcular a intensidade do LED vermelho com base na posição X
    int16_t red_intensity = (int16_t)(*x_value) - 2048;
    if (red_intensity < 0) red_intensity = -red_intensity;
    red_intensity = red_intensity * 255 / 2048;

    // Definir a intensidade dos LEDs usando PWM
    set_pwm_us(LED_AZUL, blue_intensity * 20000 / 255);
    set_pwm_us(LED_VERMELHO, red_intensity * 20000 / 255);

    // Mapear valores do joystick (0-4095) para os índices da matriz (0-24)
    int row = *x_value * 5 / 4096;                         // Mapear eixo X para as linhas
    int col = (4096 - *y_value) * 5 / 4096;                // Mapear eixo Y para as colunas (invertido)

    // Garantir que os índices estejam dentro dos limites
    if (row >= 5) row = 4;                                // Limitar valor máximo das linhas
    if (col >= 5) col = 4;                                // Limitar valor máximo das colunas

    return row * 5 + col;  
}

// Função para configurar PWM no pino especificado com um período desejado
void configure_pwm(uint gpio, float period_ms) 
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    float clkdiv = clock_get_hz(clk_sys) / (1000.0f / period_ms * 65536);
    pwm_config_set_clkdiv(&config, clkdiv);               // Define a divisão do clock
    pwm_init(slice_num, &config, true);
    pwm_set_wrap(slice_num, 65535);                       // Define o valor máximo do contador PWM
}

// Função para definir o ciclo ativo em microsegundos
void set_pwm_us(uint gpio, uint16_t us) 
{
    if (!leds_on) {
        pwm_set_gpio_level(gpio, 0);                      // Desativa o LED se leds_on for false
        return;
    }
    
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t top = pwm_hw->slice[slice_num].top;          // Valor máximo do contador PWM
    uint32_t level = (us * (top + 1)) / 20000;            // 20000us é o período de 20ms

    pwm_set_gpio_level(gpio, level);
}

// Função para lidar com a interrupção dos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    
    flag_button = true;                                                 // Ativa a flag para ignorar o botão

    uint32_t current_time = to_us_since_boot(get_absolute_time());      // Obter o tempo atual em microssegundos

    if(current_time - last_time > 300000) {                             // Ignorar eventos muito próximos
       
        last_time = current_time;
        
        if (gpio == button_B) {                                         // Avança para a próxima combinação
            
            set_button = 3;
        } 
        if (gpio == button_A) {                                         // Retrocede para a combinação anterior
            leds_on = !leds_on;
            set_button = 2;
        }
        if (gpio == button_joy) {                                       // Reseta a combinação
            gpio_put(LED_VERDE, !gpio_get(LED_VERDE));
            set_button = 1;
        }
    }
}

int main() 
{
    stdio_init_all();                                     // Inicializar a comunicação serial
    npInit(LED_PIN);                                      // Inicializar os LEDs
    npClear();                                            // Apagar todos os LEDs
    npWrite();                                            // Atualizar o estado inicial dos LEDs

    // Configura PWM para um período de 20ms (50Hz)
    configure_pwm(LED_VERDE, 20.0f);
    configure_pwm(LED_AZUL, 20.0f);
    configure_pwm(LED_VERMELHO, 20.0f);
    
    i2c_init(I2C_PORT, 400 * 1000);                                     // Inicializa o display OLED

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                          // Seta a função do pino GPIO para I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                          
    gpio_pull_up(I2C_SDA);                                              // Estabelece o pull-up na linha de dados
    gpio_pull_up(I2C_SCL);                                              // Estabelece o pull-up na linha de clock
    ssd1306_t ssd;                                                      // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);       // Inicializa o display
    ssd1306_config(&ssd);                                               // Configura o display
    ssd1306_send_data(&ssd);                                            // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Configura os pinos para o LED RGB (11, 12 e 13) como saída digital.
    gpio_init(LED_AZUL);
    gpio_set_dir(LED_AZUL, GPIO_OUT);
    gpio_init(LED_VERDE);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
    gpio_init(LED_VERMELHO);
    gpio_set_dir(LED_VERMELHO, GPIO_OUT);

    // Configura o pino dos butões como entrada digital.
    gpio_init(button_A);
    gpio_set_dir(button_A, GPIO_IN);
    gpio_init(button_B);                                  
    gpio_set_dir(button_B, GPIO_IN);
    gpio_init(button_joy);
    gpio_set_dir(button_joy, GPIO_IN);
    // Habilita o resistor pull-up interno para o pino do botão.
    // Isso garante que o pino seja lido como alto (3,3 V) quando o botão não está pressionado.
    gpio_pull_up(button_A);
    gpio_pull_up(button_B);
    gpio_pull_up(button_joy);

    //Configuração da interrupção do botão A
    gpio_set_irq_enabled_with_callback(button_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);   // Habilitar interrupção no botão A
    //Configuração da interrupção do botão B
    gpio_set_irq_enabled_with_callback(button_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);   // Habilitar interrupção no botão B
    //Configuração da interrupção do botão Joystick
    gpio_set_irq_enabled_with_callback(button_joy, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);   // Habilitar interrupção no botão Joystick

    bool cor = true;

    joystickInit();                                       // Inicializar o joystick

    while (true)                                          // Loop principal
    {
        uint16_t x_value, y_value;
        getJoystickLEDIndex(&x_value, &y_value);          // Obter os valores do joystick

        // Mapear valores do joystick (0-4095) para a resolução do display (0-127 para x, 0-63 para y)
        uint8_t x_pos = (4096 - x_value) * WIDTH / 4096;
        uint8_t y_pos = (4096 - y_value) * HEIGHT / 4096;

        // Garantir que o pixel não ultrapasse o retângulo
        if (x_pos < 3) x_pos = 6;
        if (x_pos > 122) x_pos = 119;
        if (y_pos < 3) y_pos = 6;
        if (y_pos > 58) y_pos = 55;

        // Limpar o display e desenhar o pixel na nova posição
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor);
        //ssd1306_pixel(&ssd, x_pos, y_pos, true);
        ssd1306_draw_char(&ssd, 'o', x_pos-3, y_pos-3);
        ssd1306_send_data(&ssd);
        
        if(flag_button) {                                                                    // Evento para verificar o botão foi acionado
            flag_button = false;                                                             // Reseta a flag
            // Verifica se o botão foi pressionado (nível baixo no pino) para emissão da mensagem.
            if (set_button == 1) {
                cor = !cor;
            }
            if (set_button == 2) {
                
                if(gpio_get(LED_AZUL) == 1) {
                    
                } else {
                    
                }
            }
            set_button = 0;                                                                  // Reseta o valor do botão
            // Reseta o tempo de espera para a mensagem padrão
            
        }

        sleep_ms(100);                                    // Pequeno atraso para estabilidade
    }
}