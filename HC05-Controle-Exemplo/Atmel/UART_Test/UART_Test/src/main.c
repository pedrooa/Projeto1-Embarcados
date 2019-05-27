/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <string.h>

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL


#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif


#define AFEC_CHANNEL_RES_PIN 0 //PD30
#define AFEC_CHANNEL_RES_PIN1 8 //PA19

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)

//Configs button A
#define BUTA_PIO           PIOD
#define BUTA_PIO_ID        ID_PIOD
#define BUTA_PIO_IDX       11u
#define BUTA_PIO_IDX_MASK  (1u << BUTA_PIO_IDX)

//Configs button B
#define BUTB_PIO           PIOA
#define BUTB_PIO_ID        ID_PIOA
#define BUTB_PIO_IDX       6u
#define BUTB_PIO_IDX_MASK  (1u << BUTB_PIO_IDX)

//Configs button Select
#define BUTSELECT_PIO           PIOA
#define BUTSELECT_PIO_ID        ID_PIOA
#define BUTSELECT_PIO_IDX       24u
#define BUTSELECT_PIO_IDX_MASK  (1u << BUTSELECT_PIO_IDX)

//Configs button Start
#define BUTSTART_PIO           PIOA
#define BUTSTART_PIO_ID        ID_PIOA
#define BUTSTART_PIO_IDX       2u
#define BUTSTART_PIO_IDX_MASK  (1u << BUTSTART_PIO_IDX)

#define LEDA_PIO  PIOB
#define LEDA_PIO_ID ID_PIOB
#define LEDA_PIO_IDX 2u
#define LEDA_PIO_IDX_MASK (1u << LEDA_PIO_IDX)

#define LEDB_PIO PIOB
#define LEDB_PIO_ID ID_PIOB
#define LEDB_PIO_IDX 3u
#define LEDB_PIO_IDX_MASK (1u << LEDB_PIO_IDX)

#define LEDSELECT_PIO PIOC
#define LEDSELECT_PIO_ID ID_PIOC
#define LEDSELECT_PIO_IDX 30u
#define LEDSELECT_PIO_IDX_MASK (1u << LEDSELECT_PIO_IDX)

#define LEDSTART_PIO PIOC
#define LEDSTART_PIO_ID ID_PIOC
#define LEDSTART_PIO_IDX 17u
#define LEDSTART_PIO_IDX_MASK (1u << LEDSTART_PIO_IDX)


/** UART Interface */
#define CONF_UART            CONSOLE_UART
/** Baudrate setting */
#define CONF_UART_BAUDRATE   (115200UL)
/** Character length setting */
#define CONF_UART_CHAR_LENGTH  US_MR_CHRL_8_BIT
/** Parity setting */
#define CONF_UART_PARITY     US_MR_PAR_NO
/** Stop bits setting */
#define CONF_UART_STOP_BITS    US_MR_NBSTOP_1_BIT



/************************************************************************/
/* variaveis globais                                                    */
/**********************************************ta**************************/
volatile bool butA_flag = false;
volatile bool butB_flag = false;
volatile bool butSelect_flag = false;
volatile bool butStart_flag = false;

volatile long g_systimer = 0;

/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;
volatile bool g_is_res_done = false;
volatile bool g_delay = false;

volatile bool g_is_res_done1 = false;
volatile bool g_is_conversion_done1 = false;
volatile bool g_delay1 = false;

volatile uint32_t g_res_value = 0;
volatile uint32_t g_res_value1 = 0;

volatile char[80] analog_x;
volatile char[80] analog_y;



void butA_callback(void)
{
	printf("A\n");
  butA_flag = true;
}

void butB_callback(void)
{
		printf("B\n");

  butB_flag = true;
}

void butSelect_callback(void)
{
			printf("Select\n");

  butSelect_flag = true;
}

void butStart_callback(void)
{
				printf("Start\n");

  butStart_flag = true;
}

static void AFEC_Res_callback(void)
{
	g_res_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_RES_PIN);
	g_is_res_done = true;
}

static void AFEC_Res_callback1(void)
{
	g_res_value1 = afec_channel_get_value(AFEC0, AFEC_CHANNEL_RES_PIN1);
	g_is_res_done1 = true;
}


void SysTick_Handler() {
	g_systimer++;
}

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN1);
	afec_start_software_conversion(AFEC0);
	
}


void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_start_software_conversion(AFEC0);
	
}


static int32_t convert_adc_to_res(int32_t ADC_value){

  int32_t ul_vol;

  /*
   * converte bits -> tens�o (Volts)
   */
	ul_vol = ADC_value * 32768 / 4095;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  return(ul_vol);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, ((ul_sysclk) / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

static void config_ADC_TEMP_RES(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);


	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0, AFEC_Res_callback, 1);
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_8, AFEC_Res_callback1, 1);
	

	/*** Configuracao espec�fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_RES_PIN1, 0x200);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_RES_PIN, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa convers�o */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN1);
}

void set_analog_result_x(uint32_t input) {
	sprintf(analog_x, ";%d;", input);
	}
	//analog_x = convert_adc_to_res(input);
	//analog_x = 32768*input/4095;
}

void set_analog_result_y(uint32_t input) {
	sprintf(analog_y, ";%d;", input);
}

//funcao que p
void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

//funcao que preenche o buffer de entrada
int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	//para saber quando o buffer acabou, o c�digo coloca 0x00 como �ltimo valor do buffer
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	 // RX - PB0  TX - PB1 
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
char buffer_rx[128];
usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEPedro", 1000);
usart_log("hc05_server_init", buffer_rx);
usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
usart_send_command(USART0, buffer_rx, 1000, "AT+PIN5555", 1000);
usart_log("hc05_server_init", buffer_rx);

/*
usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
usart_send_command(USART0, buffer_rx, 1000, "AT+RESET", 1000);
usart_log("hc05_server_init", buffer_rx);*/
}

void pisca_led(uint LED_PIO, uint LED_IDX_MASK){
	pio_set(LED_PIO, LED_IDX_MASK);
	delay_ms(50);
	pio_clear(LED_PIO, LED_IDX_MASK);
}

// Fun��o de inicializa��o do uC
void init(void)
{

	//desativa watchdog timer
	WDT->WDT_MR = WDT_MR_WDDIS;

// Inicializa clock do periférico PIO responsavel pelo botao A
	pmc_enable_periph_clk(BUTA_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUTA_PIO, PIO_INPUT, BUTA_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	//pio_set_debounce_filter(BUTA_PIO,BUTA_PIO_IDX_MASK,200);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUTA_PIO,
                  BUTA_PIO_ID,
                  BUTA_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butA_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTA_PIO, BUTA_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTA_PIO_ID);
  NVIC_SetPriority(BUTA_PIO_ID, 4); // Prioridade 4

  //**************************************************************

  // Inicializa clock do periférico PIO responsavel pelo botao B
	pmc_enable_periph_clk(BUTB_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUTB_PIO, PIO_INPUT, BUTB_PIO_IDX_MASK, PIO_PULLUP);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUTB_PIO,
                  BUTB_PIO_ID,
                  BUTB_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butB_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTB_PIO, BUTB_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTB_PIO_ID);
  NVIC_SetPriority(BUTB_PIO_ID, 4); // Prioridade 4

  //**************************************************************

  // Inicializa clock do periférico PIO responsavel pelo botao Select
	pmc_enable_periph_clk(BUTSELECT_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUTSELECT_PIO, PIO_INPUT, BUTSELECT_PIO_IDX_MASK, PIO_PULLUP);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUTSELECT_PIO,
                  BUTSELECT_PIO_ID,
                  BUTSELECT_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butSelect_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTSELECT_PIO, BUTSELECT_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTSELECT_PIO_ID);
  NVIC_SetPriority(BUTSELECT_PIO_ID, 4); // Prioridade 4

  //**************************************************************

  // Inicializa clock do periférico PIO responsavel pelo botao Start
	pmc_enable_periph_clk(BUTSTART_PIO_ID);

  // Configura PIO para lidar com o pino do botão como entrada
  // com pull-up
	pio_configure(BUTSTART_PIO, PIO_INPUT, BUTSTART_PIO_IDX_MASK, PIO_PULLUP);

  // Configura interrupção no pino referente ao botao e associa
  // função de callback caso uma interrupção for gerada
  // a função de callback é a: but_callback()
  pio_handler_set(BUTSTART_PIO,
                  BUTSTART_PIO_ID,
                  BUTSTART_PIO_IDX_MASK,
                  PIO_IT_FALL_EDGE,
                  butStart_callback);

  // Ativa interrupção
  pio_enable_interrupt(BUTSTART_PIO, BUTSTART_PIO_IDX_MASK);

  // Configura NVIC para receber interrupcoes do PIO do botao
  // com prioridade 4 (quanto mais próximo de 0 maior)
  NVIC_EnableIRQ(BUTSTART_PIO_ID);
  NVIC_SetPriority(BUTSTART_PIO_ID, 4); // Prioridade 4
  
  //configs LED
  
  pmc_enable_periph_clk(LEDA_PIO_ID);
  pio_configure(LEDA_PIO, PIO_OUTPUT_0, LEDA_PIO_IDX_MASK, PIO_DEFAULT);
  
  pmc_enable_periph_clk(LEDB_PIO_ID);
  pio_configure(LEDB_PIO, PIO_OUTPUT_0, LEDB_PIO_IDX_MASK, PIO_DEFAULT);
  
  pmc_enable_periph_clk(LEDSTART_PIO_ID);
  pio_configure(LEDSTART_PIO, PIO_OUTPUT_0, LEDSTART_PIO_IDX_MASK, PIO_DEFAULT);
  
  pmc_enable_periph_clk(LEDSELECT_PIO);
  pio_configure(LEDSELECT_PIO_ID, PIO_OUTPUT_0, LEDSELECT_PIO_IDX_MASK, PIO_DEFAULT);
	
}

void send_command(char buttonStart, char buttonA, char buttonB, char analog_x, char analog_y, char eof ){
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, buttonStart);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, buttonA);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, buttonB);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, analog_x);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, analog_y);
			while(!usart_is_tx_ready(UART_COMM));
			usart_write(UART_COMM, eof);
}

int main (void)
{
	board_init();
	sysclk_init(); 
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_ADC_TEMP_RES();
	configure_console();
	init();
	
	
	TC_init(TC0, ID_TC1, 1, 7);
	TC_init(TC0, ID_TC0, 0, 5);
	
/*
		usart_put_string(USART1, "Inicializando...\r\n");
		usart_put_string(USART1, "Config HC05 Server...\r\n");
		hc05_config_server();
		hc05_server_init();
		*/
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	char buttonA = '0';
	char buttonB = '0';
	char buttonSelect = '0';
	char buttonStart = '0';
	char eof = 'X';
	char buffer[1024];
	

	while(1) {
		if(butA_flag) {
			buttonA = '1';
			pisca_led(LEDA_PIO, LEDA_PIO_IDX_MASK);
			//butA_flag = false;
		} else {
			buttonA = '0';
		}
		
		if(butB_flag){
			buttonB = '1';
			pisca_led(LEDB_PIO, LEDB_PIO_IDX_MASK);
			//butB_flag = false;
		}
		else{
			buttonB = '0';
		}
		
		if(butSelect_flag){
			buttonSelect = '1';
			pisca_led(LEDSELECT_PIO, LEDSELECT_PIO_IDX_MASK);
			//butSelect_flag = false;
			
		}
		else{
			buttonSelect = '0';
		}
		
		if(butStart_flag){
			buttonStart = '1';
			pisca_led(LEDSTART_PIO, LEDSTART_PIO_IDX_MASK);
			//butStart_flag = false;
		}
		else {
			buttonStart = '0';
		}
		
		if(g_is_res_done==true){
			set_analog_result_x(g_res_value);
			printf("Res : %d \r\n", (g_res_value));
			//g_is_res_done = false;
			
		}
		
		if(g_is_res_done1 == true) {
			set_analog_result_y(g_res_value1);
			printf("Res1 : %d \r\n", (g_res_value1));
					//	g_is_res_done1 = false;

		}

/*
					while(!usart_is_tx_ready(UART_COMM));
					usart_write(UART_COMM, buttonStart);
					while(!usart_is_tx_ready(UART_COMM));
					usart_write(UART_COMM, buttonA);
					while(!usart_is_tx_ready(UART_COMM));
					usart_write(UART_COMM, buttonB);
					while(!usart_is_tx_ready(UART_COMM));
					usart_write(UART_COMM, analog_x);
					while(!usart_is_tx_ready(UART_COMM));
					usart_write(UART_COMM, analog_y);
					while(!usart_is_tx_ready(UART_COMM));
					usart_write(UART_COMM, eof);
					delay_ms(1); || g_is_res_done || g_is_res_done1*/
		
		if( butA_flag || butB_flag || butSelect_flag || butStart_flag || g_is_res_done || g_is_res_done1 ) {
		//esse while existe pois a velocidade do microprocessador � muito mais rapida do que a do bt. Ele existe para fazer o c�digo esperar o buffer do bt estar pronto.
			butA_flag = false;
			butB_flag = false;
			butSelect_flag = false;
			butStart_flag = false;
			g_is_res_done = false;
			g_is_res_done1 = false;
			send_command(buttonStart,buttonA,buttonB,analog_x,analog_y,eof );
			delay_ms(10);
			
			send_command(0,0,0,analog_x,analog_y,eof);
			

		}
	}
}
	