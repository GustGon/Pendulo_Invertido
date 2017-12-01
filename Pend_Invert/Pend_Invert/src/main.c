/*
 * In�cio do Projeto_Pelletron
 * Gustavo da Silva Gon�alves
 * 23/04/2015
 */
 
#include <asf.h>
#include <time.h>
#include <string.h>
#include <conf_board.h>
#include <stdarg.h>


void printl(const char * format, ...) {
	char buffer[256];
	va_list args;
	va_start(args, format);
	vsprintf(buffer, format, args);
	usart_serial_write_packet(CONF_UART, buffer, strlen(buffer));
	va_end(args);
}


#define PADRAO	0xAAAAAAAA
#define ERRO	0x1
#define TC_CHANNEL	0
// #define LED0_G	IOPORT_CREATE_PIN(PIOA,16) //Pino do LED Verde
// #define	LED1_Y	IOPORT_CREATE_PIN(PIOA,0)//Pino do LED Amarelo
// #define	LED2_R	IOPORT_CREATE_PIN(PIOA,22)//Pino do LED Vermelho
// #define PINO_1  IOPORT_CREATE_PIN(PIOB,3) //Pino (EXT1/5) de an�lise do tempo de escrita
// #define	PINO_2	IOPORT_CREATE_PIN(PIOA,15)//Pino (EXT4/5) de an�lise do tempo de leitura
#define STRING_EOL    "\n\r"
#define STRING_HEADER "\r\n-- Gustavo Gon�alves --\r\n-- PROJETO_PELLETRON --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


/*=========================================*\
	*Declara��o das fun��es
\*=========================================*/

void UART0_Handler(void);
static void conf_uart (void);
static void conf_TC (void);
void config_ensaio(void);
void TC0_Handler(void);
//========================================

/*==========================================================================================*\
	*Declara��o das variaveis
\*==========================================================================================*/

// Variaveis para acertar o hor�rio 
uint32_t hora, minuto, seg;

// Flag para definir quando h� valor na Serial. Deve ser "volatile", porque � modificada constantemente no programa (N�o funciona sem isso).
volatile bool data_rdy;


//==============================================================================================

// Fun��o Handler da interrup��o da Serial.
void UART0_Handler(void)
{
	
}

//Fun��o de configura��o da porta serial UART0.
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
	
	// Habilita a interrup��o da Serial UART0 - Interrup��o RXRDY.
	uart_enable_interrupt(CONF_UART, UART_IER_RXRDY);

	// Habilita a interrup��o da UART0.
	NVIC_EnableIRQ(UART0_IRQn);
}

//Fun��o de configura��o do Time Counter
static void conf_TC (void){
	
	uint32_t tc_div;
	uint32_t tc_tcclks;
	uint32_t tc_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC0);

	tc_find_mck_divisor(4,tc_sysclk, &tc_div, &tc_tcclks, tc_sysclk);
	tc_init(TC0, TC_CHANNEL , tc_tcclks | TC_CMR_CPCTRG);
	printl("div = %i ; clk = %i ; sysclk = %i \n\n",tc_div,tc_tcclks,tc_sysclk);
	//interrup��o a cada 1 segundo, o maximo mutiplicando � 2 segundos
	tc_write_rc(TC0, TC_CHANNEL, (tc_sysclk/tc_div)*2);
	
	NVIC_EnableIRQ(TC0_IRQn);
	tc_enable_interrupt(TC0, TC_CHANNEL, TC_IER_CPCS);
	tc_start(TC0, TC_CHANNEL);
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

//Interrup��o do TC a cada 1 ms.
void TC0_Handler(void){
	
	uint32_t id = 45;

	//printl("Teste de variavel = %i \n",id);
}

int main (void)
{
	sysclk_init();
	board_init();
	delay_init(F_CPU);

//Fun��o de configura��o da UART
	conf_uart();	
//Fun��o de configura��o do ensaio	
	config_ensaio();

//Fun��o de configura��o do TC
	conf_TC();
	
	//puts(STRING_HEADER);
	
	while (1) {	
	}
}
