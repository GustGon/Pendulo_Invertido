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

#define TC_CHANNEL	0
	
#define ID_ENC		ID_PIOB
#define ENC_PULSE_A		PIO_PB3	// GPIO EXT 1 - 5
#define ENC_PULSE_B		PIO_PB9	// GPIO EXT 1 - 6	
#define ENC_REFER		PIO_PB4	// GPIO EXT 3 - 5

#define MOTOR_P	PIO_PA0	//PWM(+) EXT 1 - 7
#define MOTOR_N	PIO_PA1	//PWM(-) EXT 1 - 8

#define ENC_RES		1024
#define VOLTA_COMP	360
#define SETPOINT	0;

/*
	Define controle
*/

#define ANGLE_SETPOINT 		0.0
#define ANGLE_IRRECOVERABLE 45.0

#define WINDUP_GUARD 		100


#define STRING_EOL    "\n\r"
#define STRING_HEADER "\r\n-- Gustavo Gon�alves --\r\n-- PROJETO_PENDULO_INVERTIDO --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


/*=========================================*\
	*Declara��o das fun��es
\*=========================================*/

void UART0_Handler(void);
static void conf_uart (void);
static void conf_TC (void);
void SysTick_Handler(void);
void pin_handler(uint32_t id, uint32_t mask);
void config_ensaio(void);
void TC0_Handler(void);
float control(float input);
//========================================

/*==========================================================================================*\
	*Declara��o das variaveis
\*==========================================================================================*/

// Variaveis para acertar o hor�rio 
uint32_t hora, minuto, seg;

// Flag para definir quando h� valor na Serial. Deve ser "volatile", porque � modificada constantemente no programa (N�o funciona sem isso).
volatile bool data_rdy;

// Flag para girar para esquerda oou direita
uint32_t aFlag = 0;
uint32_t bFlag = 0;

uint8_t aux = 1;

float Ci = 0;
float Cp = 0;
float Cd = 0;

unsigned long lastTime = 0;
float lastError=0;

volatile uint32_t g_ul_ms_ticks = 0;

// Ganhos de cada parte do controlador
float Kp = 1;
float Ki = 1;
float Kd = 1;

int32_t round = 0;

float graus = 0;

//==============================================================================================


void SysTick_Handler(void){
	g_ul_ms_ticks++;
}

/*
	subrotina do controlador
*/
float control(float input){
	
	//rtc_get_milliseconds(RTC);
	    uint32_t now = 0;
				
	    float dt;
	    float error;
	    float de;
	    float output;
		
		dt = (float)(now - lastTime)/1000.0f;
		
		/* Calculate delta error */
		error = SETPOINT - input;
		    
		de = error - lastError;

		/* Proportional Term */
		Cp = error;

		/* Integral Term */
		Ci += error*dt;

		Cd = 0;
		/* to avoid division by zero */
		if(dt>0)
		{
			/* Derivative term */
			Cd = de/dt;
		}

		/* Save for the next iteration */
		lastError = error;
		lastTime = now;

		/* Sum terms: pTerm+iTerm+dTerm */
		output = Cp*Kp + Ci*Ki + Cd*Kd;
		//printf("output: " + String(output));

		/* Saturation - Windup guard for Integral term do not reach very large values */
		if(output > WINDUP_GUARD){
			output = WINDUP_GUARD;
		}
		else if (output < -WINDUP_GUARD){
			output = -WINDUP_GUARD;
		}

		return output;
}


/*
* Fun��o Handler da interrup��o da Serial.
*/ 
void UART0_Handler(void)
{
	
}

/*
* Fun��o de configura��o da porta serial UART0.
*/
static void conf_uart (void)
{
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	sysclk_enable_peripheral_clock (CONSOLE_UART_ID);
	
	// Habilita a interrup��o da Serial UART0 - Interrup��o RXRDY.
	uart_enable_interrupt(CONF_UART, UART_IER_RXRDY);
	
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	
	// Habilita a interrup��o da UART0.
	NVIC_EnableIRQ(UART0_IRQn);
}


/*
* Fun��o de configura��o do Time Counter
*/
static void conf_TC (void){
	
	uint32_t tc_div;
	uint32_t tc_tcclks;
	uint32_t tc_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC0);

	/** Configure TC for a 4Hz frequency and trigger on RC compare. */
	tc_find_mck_divisor(4,tc_sysclk, &tc_div, &tc_tcclks, tc_sysclk);
	tc_init(TC0, TC_CHANNEL , tc_tcclks | TC_CMR_CPCTRG);
	//interrup��o a cada 250 milisegundos.
	tc_write_rc(TC0, TC_CHANNEL, (tc_sysclk/tc_div) / 4);
	
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
	
	//Pinos do motor
	pio_set_output(PIOA,MOTOR_N,PIO_DEFAULT,false,false);
	pio_set_output(PIOA,MOTOR_P,PIO_DEFAULT,false,false);
	
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

//Interrup��o do TC a cada 1 ms.
void TC0_Handler(void){
	
	volatile uint32_t ul_dummy;
	char log[30];
	float motorPorcent;

	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0, TC_CHANNEL);

	/*
	*	Inserir o algoritimo do controle aki!!
	*/
	graus = (round/ENC_RES)*VOLTA_COMP;

	sprintf(log,"Graus: %f\nTicks: %i\n",graus,round);
	printf("%s",log);
	
	motorPorcent = control(graus);
	
	if( 0 > motorPorcent)
	{
		pio_set_pin_high(MOTOR_P);
		pio_set_pin_low(MOTOR_N);
		
	}else if ( 0 < motorPorcent )
	{
		pio_set_pin_high(MOTOR_N);
		pio_set_pin_low(MOTOR_P);
		
	}else
	{
		pio_set_pin_low(MOTOR_N);
		pio_set_pin_low(MOTOR_P);
	}

}

void pin_handler(uint32_t id, uint32_t mask){	
	if (id == ID_ENC){
		switch (mask){
			case ENC_PULSE_A:
			puts("PULSO A");
				if (bFlag){
					gpio_toggle_pin(LED0_GPIO);
					round++;
					} else {
					aFlag = 1;
				}
			break;
			case !ENC_PULSE_B:
			puts("PULSO B");
				if (aFlag){
					gpio_toggle_pin(LED0_GPIO);
					round--;
					} else {
					bFlag = 1;
				}
			break;
			case ENC_REFER:
				puts("PULSO REF");
				gpio_toggle_pin(LED0_GPIO);
				bFlag = 0;
				aFlag = 0;
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
	
//Fun��o de configura��o da UART
	conf_uart();	
	
//Fun��o de configura��o do ensaio	
	config_ensaio();

//Fun��o de configura��o do TC
	conf_TC();
	
	puts(STRING_HEADER);
	
	while (1) {
	}
}
