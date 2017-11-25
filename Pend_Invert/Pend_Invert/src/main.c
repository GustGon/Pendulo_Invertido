/*
 * Início do Projeto_Pelletron
 * Gustavo da Silva Gonçalves
 * 23/04/2015
 */
 
#include <asf.h>
#include <time.h>
#include <string.h>
#include <conf_board.h>
#include <stdlib.h>

#define PADRAO	0xAAAAAAAA
#define ERRO	0x1
// #define LED0_G	IOPORT_CREATE_PIN(PIOA,16) //Pino do LED Verde
// #define	LED1_Y	IOPORT_CREATE_PIN(PIOA,0)//Pino do LED Amarelo
// #define	LED2_R	IOPORT_CREATE_PIN(PIOA,22)//Pino do LED Vermelho
// #define PINO_1  IOPORT_CREATE_PIN(PIOB,3) //Pino (EXT1/5) de análise do tempo de escrita
// #define	PINO_2	IOPORT_CREATE_PIN(PIOA,15)//Pino (EXT4/5) de análise do tempo de leitura
#define STRING_EOL    "\n\r"
#define STRING_HEADER "\r\n-- Gustavo Gonçalves --\r\n-- PROJETO_PELLETRON --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


/*=========================================*\
	*Declaração das funções
\*=========================================*/

void UART0_Handler(void);
static void conf_uart (void);
static void conf_TC (void);
void config_ensaio(void);
void TC0_Handler(void);
//========================================

/*==========================================================================================*\
	*Declaração das variaveis
\*==========================================================================================*/

// Variaveis para acertar o horário 
uint32_t hora, minuto, seg;

// Flag para definir quando há valor na Serial. Deve ser "volatile", porque é modificada constantemente no programa (Não funciona sem isso).
volatile bool data_rdy;


//==============================================================================================

// Função Handler da interrupção da Serial.
void UART0_Handler(void)
{
	
}

//Função de configuração da porta serial UART0.
static void conf_uart (void)
{
	static usart_serial_options_t usart_options = {
		.baudrate =   CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits =   CONF_UART_STOP_BITS
	};
	
	sysclk_enable_peripheral_clock (CONSOLE_UART_ID);
	usart_serial_init(CONF_UART, &usart_options);
	
	// Habilita a interrupção da Serial UART0 - Interrupção RXRDY.
	uart_enable_interrupt(CONF_UART, UART_IER_RXRDY);

	// Habilita a interrupção da UART0.
	NVIC_EnableIRQ(UART0_IRQn);
}

//Função de configuração do Time Counter
static void conf_TC (void){
	
	uint32_t tc_div;
	uint32_t tc_tcclks;
	uint32_t tc_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC0);

	tc_find_mck_divisor(4,tc_sysclk, &tc_div, &tc_tcclks, tc_sysclk);
	tc_init(TC0, 0, tc_tcclks | TC_CMR_CPCTRG);
	//interrupção a cada 1 segundo, o maximo mutiplicando é 2 segundos
	tc_write_rc(TC0, 0, (tc_sysclk/tc_div));
	
	NVIC_EnableIRQ(TC0_IRQn);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	tc_start(TC0, 0);
}

// config ensaio
void config_ensaio(void){
	
	//Configurar os pino para ser porta I/O
// 	ioport_set_pin_dir(LED0_G, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_dir(LED1_Y, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_dir(LED2_R, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_dir(PINO_1, IOPORT_DIR_OUTPUT);
// 	ioport_set_pin_dir(PINO_2, IOPORT_DIR_OUTPUT);
	// configure UART pins
	ioport_set_port_mode(IOPORT_PIOA, PIO_PA9A_URXD0 | PIO_PA10A_UTXD0, IOPORT_MODE_MUX_A);
	ioport_disable_port(IOPORT_PIOA, PIO_PA9A_URXD0 | PIO_PA10A_UTXD0);
	
	//Configurar a hora do teste
	rtc_get_time(RTC,&hora,&minuto,&seg);
	//Modo de aparecer as horas 24h
	rtc_set_hour_mode(RTC, 0);
	
};

//Interrupção do TC a cada 1 ms.
void TC0_Handler(void){
	
	char str1[128];
	uint8_t rx_char = 0;
	uint32_t id = 455;

	sprintf(str1,"Teste de variavel = %i \n",clock);	
	usart_serial_write_packet(CONF_UART, str1, sizeof(str1) - 1);
	//usart_write(CONF_UART,str1);
}

int main (void)
{
	sysclk_init();
	board_init();
	//delay_init(F_CPU);
//Função de configuração da UART
	conf_uart();	
//Função de configuração do ensaio	
	config_ensaio();

//Função de configuração do TC
	conf_TC();
	
	//puts(STRING_HEADER);
	
	while (1) {	
	}
}
