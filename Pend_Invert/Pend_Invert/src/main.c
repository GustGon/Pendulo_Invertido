/*
 * Início do Projeto_Pelletron
 * Gustavo da Silva Gonçalves
 * 23/04/2015
 */
 
#include <asf.h>
#include <time.h>
#include <string.h>
#include <conf_board.h>
#include <stdarg.h>

#define TC_CHANNEL	0

#define ID_ENC		ID_PIOB
#define ENC_PULSE_A		PIO_PB3	// GPIO EXT 1 - 5
#define ENC_PULSE_B		PIO_PB9	// GPIO EXT 1 - 6
#define ENC_REFER		PIO_PB4	// GPIO EXT 3 - 5

#define MOTOR_P	PIO_PA0	//PWM(+) EXT - 7
#define MOTOR_N	PIO_PA1	//PWM(-) EXT - 8

#define ENC_RES		1024
#define VOLTA_COMP	360

volatile uint32_t g_ul_ms_ticks = 0;

// #define LED0_G	IOPORT_CREATE_PIN(PIOA,16) //Pino do LED Verde
// #define	LED1_Y	IOPORT_CREATE_PIN(PIOA,0)//Pino do LED Amarelo
// #define	LED2_R	IOPORT_CREATE_PIN(PIOA,22)//Pino do LED Vermelho
// #define PINO_1  IOPORT_CREATE_PIN(PIOB,3) //Pino (EXT1/5) de análise do tempo de escrita
// #define	PINO_2	IOPORT_CREATE_PIN(PIOA,15)//Pino (EXT4/5) de análise do tempo de leitura


#define STRING_EOL    "\n\r"
#define STRING_HEADER "\r\n-- Gustavo Gonçalves --\r\n-- PROJETO_PENDULO_INVERTIDO --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


/*=========================================*\
	*Declaração das funções
\*=========================================*/

void UART0_Handler(void);
static void conf_uart (void);
static void conf_TC (void);
void SysTick_Handler(void);
void pin_handler(uint32_t id, uint32_t mask);
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

// Flag para girar para esquerda oou direita
uint32_t aFlag = 0;
uint32_t bFlag = 0;

int32_t round = 0;

int32_t graus = 0;

//==============================================================================================


void SysTick_Handler(void){
	g_ul_ms_ticks++;
}

/*
* Função Handler da interrupção da Serial.
*/ 
void UART0_Handler(void)
{
	
}

/*
* Função de configuração da porta serial UART0.
*/
static void conf_uart (void)
{
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	sysclk_enable_peripheral_clock (CONSOLE_UART_ID);
	
	// Habilita a interrupção da Serial UART0 - Interrupção RXRDY.
	uart_enable_interrupt(CONF_UART, UART_IER_RXRDY);
	
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	
	// Habilita a interrupção da UART0.
	NVIC_EnableIRQ(UART0_IRQn);
}


/*
* Função de configuração do Time Counter
*/
static void conf_TC (void){
	
	uint32_t tc_div;
	uint32_t tc_tcclks;
	uint32_t tc_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC0);

	/** Configure TC for a 4Hz frequency and trigger on RC compare. */
	tc_find_mck_divisor(4,tc_sysclk, &tc_div, &tc_tcclks, tc_sysclk);
	tc_init(TC0, TC_CHANNEL , tc_tcclks | TC_CMR_CPCTRG);
	//interrupção a cada 250 milisegundos.
	tc_write_rc(TC0, TC_CHANNEL, (tc_sysclk/tc_div) / 4 );
	
	NVIC_EnableIRQ(TC0_IRQn);
	tc_enable_interrupt(TC0, TC_CHANNEL, TC_IER_CPCS);
	tc_start(TC0, TC_CHANNEL);
}


/*
* Config ensaio
*/ 
void config_ensaio(void){
	
	//Interrupt for A:
	pio_set_input(PIOB, ENC_PULSE_A, PIO_DEFAULT);
	pio_pull_down(PIOB, ENC_PULSE_A, ENABLE);
	pio_handler_set(PIOB, ID_ENC, ENC_PULSE_A, PIO_IT_RISE_EDGE, pin_handler);
	pio_enable_interrupt(PIOB, ENC_PULSE_A);
	
	//Interrupt for B:
	pio_set_input(PIOB, ENC_PULSE_B, PIO_DEFAULT);
	pio_pull_down(PIOB, ENC_PULSE_B, ENABLE);
	pio_handler_set(PIOB, ID_ENC, ENC_PULSE_B, PIO_IT_RISE_EDGE, pin_handler);
	pio_enable_interrupt(PIOB, ENC_PULSE_B);
	
	//Interrupt for C:
	pio_set_input(PIOB, ENC_REFER, PIO_DEFAULT);
	pio_pull_down(PIOB, ENC_REFER, ENABLE);
	//pio_handler_set(PIOA, ID_ENC, ENC_REFER, PIO_IT_RISE_EDGE, pin_handler);
	pio_handler_set(PIOB, ID_ENC, ENC_REFER, PIO_IT_FALL_EDGE, pin_handler);
	pio_enable_interrupt(PIOB, ENC_REFER);
	
	//Enable Interrupt GPIO:
	NVIC_DisableIRQ(PIOB_IRQn);
	NVIC_ClearPendingIRQ(PIOB_IRQn);
	NVIC_SetPriority(PIOB_IRQn, 0);
	NVIC_EnableIRQ(PIOB_IRQn);

	// configure UART pins
	ioport_set_port_mode(IOPORT_PIOA, PIO_PA9A_URXD0 | PIO_PA10A_UTXD0, IOPORT_MODE_MUX_A);
	ioport_disable_port(IOPORT_PIOA, PIO_PA9A_URXD0 | PIO_PA10A_UTXD0);
	
	//Configurar a hora do teste
	rtc_get_time(RTC,&hora,&minuto,&seg);
	//Modo de aparecer as horas 24h
	rtc_set_hour_mode(RTC, 0);
	
}

//Interrupção do TC a cada 1 ms.
void TC0_Handler(void){
	
	volatile uint32_t ul_dummy;

	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0, TC_CHANNEL);

	puts("SULAMINA");
	
	/*
	*	Inserir o algoritimo do controle aki!!
	*/
	graus = (round/ENC_RES)*VOLTA_COMP;
	
	printf("Graus = %i\n",graus);
}

void pin_handler(uint32_t id, uint32_t mask){
	if (id == ID_ENC){
		switch (mask){
			case ENC_PULSE_A:
				if (bFlag){
					gpio_toggle_pin(LED0_GPIO);
					round++;
					puts("Crescendo!!");
					} else {
					aFlag = 1;
				}
			break;
			case ENC_PULSE_B:
				if (aFlag){
					gpio_toggle_pin(LED0_GPIO);
					round--;
					puts("Decrescendo!!");
					} else {
					bFlag = 1;
				}
			break;
			case ENC_REFER:
				gpio_toggle_pin(LED0_GPIO);
				bFlag = 0;
				aFlag = 0;
				puts("Acabou o ciclo");
			break;
		}
	}
}

int main (void)
{
	sysclk_init();
	board_init();
	delay_init(F_CPU);

	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-F- Systick configuration error\r");
		while (1);
	}
	
//Função de configuração da UART
	conf_uart();	
	
//Função de configuração do ensaio	
	config_ensaio();

//Função de configuração do TC
	conf_TC();
	
	puts(STRING_HEADER);
	
	while (1) {	
	}
}
