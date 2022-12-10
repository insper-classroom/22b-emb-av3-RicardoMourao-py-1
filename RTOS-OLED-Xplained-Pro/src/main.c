#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/
int genius_get_sequence(int level, int *sequence);
void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) ;
void io_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

/* BUZZER */
#define BUZZER_PIO           PIOC
#define BUZZER_PIO_ID        ID_PIOC
#define BUZZER_PIO_IDX       13
#define BUZZER_PIO_IDX_MASK  (1u << BUZZER_PIO_IDX)

/* LED 1 da placa OLED */
#define LED1_PIO          PIOA
#define LED1_PIO_ID       ID_PIOA
#define LED1_PIO_IDX      0
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)

/* LED 2 da placa OLED */
#define LED2_PIO          PIOC
#define LED2_PIO_ID       ID_PIOC
#define LED2_PIO_IDX      30
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)

/* LED 3 da placa OLED */
#define LED3_PIO          PIOB
#define LED3_PIO_ID       ID_PIOB
#define LED3_PIO_IDX      2
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)

/* Botao 1 da placa OLED */
#define BUT1_PIO          PIOD
#define BUT1_PIO_ID       ID_PIOD
#define BUT1_PIO_IDX      28
#define BUT1_PIO_IDX_MASK (1 << BUT1_PIO_IDX)

/* Botao 2 da placa OLED */
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX      31
#define BUT2_PIO_IDX_MASK (1 << BUT2_IDX)

/* Botao 3 da placa OLED */
#define BUT3_PIO      PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_IDX      19
#define BUT3_PIO_IDX_MASK (1 << BUT3_IDX)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

QueueHandle_t xQueueBtn;

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
	int x1 = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBtn, &x1, &xHigherPriorityTaskWoken);
}

void but2_callback(void) {
	int x1 = 1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBtn, &x1, &xHigherPriorityTaskWoken);
}

void but3_callback(void) {
	int x1 = 2;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBtn, &x1, &xHigherPriorityTaskWoken);
}

void TC0_Handler(void) {
	/* A leitura do periferico informa que a interrupcao foi satisfeita */
	volatile uint32_t status = tc_get_status(TC0, 0);
	int pulsos = 1000;//(double) 1000* (double) 100;
	for (int i = 0; i < pulsos; i++) {
		pin_toggle(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
	}
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
static void task_game(void *pvParameters) {
	io_init();
	
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Level: 0", 0, 0, &sysfont);
	
	int nSequence = 0;
	int sequence[512];
	int level = 0;
	int num;
	
	pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	
	for (;;)  {
		nSequence = genius_get_sequence(level, sequence);
		xQueueReset(xQueueBtn);
		for (int i=0; i<nSequence; i++) {
			
			if (sequence[i]==0) {
				TC_init(TC0, ID_TC0, 0, 1000);
				tc_start(TC0, 0);
				pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
				delay_ms(500);
				pin_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
				delay_ms(500);
				tc_stop(TC0, 0);
			}
			if (sequence[i]==1) {
				TC_init(TC0, ID_TC0, 0, 1500);
				tc_start(TC0, 0);
				pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
				delay_ms(500);
				pin_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
				delay_ms(500);
				tc_stop(TC0, 0);
			}
			if (sequence[i]==2) {
				TC_init(TC0, ID_TC0, 0, 2000);
				tc_start(TC0, 0);
				pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
				delay_ms(500);
				pin_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
				delay_ms(500);
				tc_stop(TC0, 0);
				
			}
			
			delay_ms(1000);
		}
		
		delay_ms(1000);
		
		for (int i=0; i<nSequence; i++) {
			if (xQueueReceive(xQueueBtn, &num, 1000)) {
				printf("Num %d \n", num);
				if (num != sequence[i]) {
					printf("Seq %d \n", sequence[i]);
					gfx_mono_draw_string("             ", 0,10, &sysfont);
					gfx_mono_draw_string("ERROU", 0,10, &sysfont);
					delay_ms(1000);
					gfx_mono_draw_string("             ", 0,10, &sysfont);
				}
			}
		}
		
		level++;
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void) {
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pmc_enable_periph_clk(BUZZER_PIO_ID);
	
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(BUZZER_PIO, PIO_OUTPUT_0, BUZZER_PIO_IDX_MASK, PIO_DEFAULT);
	
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	
	
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 40);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 40);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 40);
	
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback);


	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);

	pio_get_interrupt_status(BUT1_PIO);
	pio_get_interrupt_status(BUT2_PIO);
	pio_get_interrupt_status(BUT3_PIO);
	
	

	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
}

int genius_get_sequence(int level, int *sequence){
	int n = level + 3;

	for (int i=0; i< n ; i++) {
		*(sequence + i) = rand() % 3;
	}

	return n;
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void set_buzzer() {
	pio_set(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
}

void clear_buzzer() {
	pio_clear(BUZZER_PIO, BUZZER_PIO_IDX_MASK);
}

void buzzer_test(int freq) {
	double t = 1E6/freq;
	set_buzzer(); // 1
	delay_us(t/2);
	clear_buzzer(); // 0
	delay_us(t/2);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_game, "game", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}
	
	/* Create Queue to control oled */
	xQueueBtn = xQueueCreate(100, sizeof(int));
	if (xQueueBtn == NULL)
	printf("falha em criar a queue RGB \n");

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
