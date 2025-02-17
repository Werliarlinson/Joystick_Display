# Joystick_Display
Repositório criado para a tarefa de Microcontroladores - ADC U4C8

Grupo 2 - Subgrupo 6

Membro:

*Werliarlinson de Lima Sá Teles* - tic370101508

# Instruções de compilação

Para compilar o código, são necessárias as seguintes extensões:

*Wokwi Simulator*

*Raspberry Pi Pico*

*Cmake*

Após instalá-las basta buildar o projeto pelo CMake. A partir daí, abra o arquivo 
diagram.json e clique no botão verde para iniciar a simulação.

Enquanto na simulação, o usuário pode clicar nos botões dispostos na simulação
a fim de acender os leds conectados à placa.

O botão azul irá acionar o led azul e em decorrencia desse eveto irá emitir uma imagem na tela e terminal

O botão verde irá acionar o led verde e em decorrencia desse eveto irá emitir uma imagem na tela e terminal

Digitando um caractere alfanumérico no terminal irá ligar os Leds da Matriz WS2812B de maneira correspondente ao simbolo
Como também irá exibir uma mensagem no terminal e na tela SSD1306

Os botões operam por interrupção e a tela irá resetar por meio de um temporizador que se adequa a atividade
