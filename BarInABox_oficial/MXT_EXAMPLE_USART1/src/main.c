/**
 * Alessandra Bl�cher
 *
**/

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "tfont.h"
#include "digital521.h"


/************************************************************************/
/* LCD + TOUCH                                                          */
/************************************************************************/
#define MAX_ENTRIES        3

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;
	
/************************************************************************/
/* RTOS                                                                  */
/************************************************************************/
#define TASK_MXT_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_MXT_STACK_PRIORITY        (tskIDLE_PRIORITY)  

#define TASK_LCD_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOMB1_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_BOMB1_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOMB2_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_BOMB2_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOMB3_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_BOMB3_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOMB4_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_BOMB4_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOMB5_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_BOMB5_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOMB6_STACK_SIZE            (4*1024/sizeof(portSTACK_TYPE))
#define TASK_BOMB6_STACK_PRIORITY        (tskIDLE_PRIORITY)


QueueHandle_t xQueueTouch;
QueueHandle_t xQueueBomb;
SemaphoreHandle_t xSemaphoreB1,  xSemaphoreB2,  xSemaphoreB3,  xSemaphoreB4,  xSemaphoreB5,  xSemaphoreB6;


/* Canal do sensor de temperatura */

typedef struct {
  uint x;
  uint y;
} touchData;


/************************************************************************/
/*  Defines                                                             */
/************************************************************************/

// Bomba 1
#define VALVE_PIO      PIOC
#define VALVE_PIO_ID   ID_PIOC
#define VALVE_IDX      17
#define VALVE_IDX_MASK (1 << VALVE_IDX)

// Bomba 2
#define BOMBA_PIO      PIOD
#define BOMBA_PIO_ID   ID_PIOD
#define BOMBA_IDX      28
#define BOMBA_IDX_MASK (1 << BOMBA_IDX)

// Bomba 3
#define BOMBA3_PIO      PIOD
#define BOMBA3_PIO_ID   ID_PIOD
#define BOMBA3_IDX      21
#define BOMBA3_IDX_MASK (1 << BOMBA3_IDX)

// Bomba 4
#define BOMBA4_PIO      PIOD
#define BOMBA4_PIO_ID   ID_PIOD
#define BOMBA4_IDX      21
#define BOMBA4_IDX_MASK (1 << BOMBA4_IDX)

// Bomba 5
#define BOMBA5_PIO      PIOD
#define BOMBA5_PIO_ID   ID_PIOD
#define BOMBA5_IDX      21
#define BOMBA5_IDX_MASK (1 << BOMBA5_IDX)

// Bomba 6
#define BOMBA6_PIO      PIOD
#define BOMBA6_PIO_ID   ID_PIOD
#define BOMBA6_IDX      21
#define BOMBA6_IDX_MASK (1 << BOMBA6_IDX)

// Bot�o azul
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX      19
#define BUT_IDX_MASK (1 << BUT_IDX)

// Bot�o verde
#define BUT_VERDE_PIO		PIOC
#define BUT_VERDE_PIO_ID    ID_PIOC
#define BUT_VERDE_IDX	     31
#define BUT_VERDE_IDX_MASK (1 << BUT_VERDE_IDX)
///
// Bot�o vermelho
#define BUT_VERMELHO_PIO      PIOB
#define BUT_VERMELHO_PIO_ID   ID_PIOB
#define BUT_VERMELHO_IDX      2
#define BUT_VERMELHO_IDX_MASK (1 << BUT_VERMELHO_IDX)

// Bot�o branco
#define BUT_BRANCO_PIO      PIOB
#define BUT_BRANCO_PIO_ID   ID_PIOB
#define BUT_BRANCO_IDX      3
#define BUT_BRANCO_IDX_MASK (1 << BUT_BRANCO_IDX)

// Bot�o amarelo
#define BUT_AMARELO_PIO      PIOC
#define BUT_AMARELO_PIO_ID   ID_PIOC
#define BUT_AMARELO_IDX      30
#define BUT_AMARELO_IDX_MASK (1 << BUT_AMARELO_IDX)

// Bot�o preto
#define BUT_PRETO_PIO      PIOA
#define BUT_PRETO_PIO_ID   ID_PIOA
#define BUT_PRETO_IDX      0
#define BUT_PRETO_IDX_MASK (1 << BUT_PRETO_IDX)

// PWM
#define PIO_PWM_0 PIOA
#define ID_PIO_PWM_0 ID_PIOA
#define MASK_PIN_PWM_0 (1 << 0)

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;
volatile uint duty = 0;

/************************************************************************/
/* RTOS hooks                                                           */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* init                                                                 */
/************************************************************************/


static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

/**
 * \brief Set maXTouch configuration
 *
 * This function writes a set of predefined, optimal maXTouch configuration data
 * to the maXTouch Xplained Pro.
 *
 * \param device Pointer to mxt_device struct
 */
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}


void PWM0_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = duty;
	g_pwm_channel_led.channel = channel;
	pwm_channel_init(PWM0, &g_pwm_channel_led);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, channel);
}




/************************************************************************/
/* Callbacks: / Handler                                                 */
/************************************************************************/

uint8_t maqEst = 0;

void but_callback(void)
{
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreB1, &xHigherPriorityTaskWoken);
}

void but_verde_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreB2, &xHigherPriorityTaskWoken);

}

void but_vermelho_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreB3, &xHigherPriorityTaskWoken);	

}

void but_branco_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreB4, &xHigherPriorityTaskWoken);
}

void but_amarelo_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreB5, &xHigherPriorityTaskWoken);
}

void but_preto_callback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreB6, &xHigherPriorityTaskWoken);
}



void io_init(void)
{
	pmc_enable_periph_clk(VALVE_PIO_ID);
	pio_configure(VALVE_PIO, PIO_OUTPUT_0, VALVE_IDX_MASK, PIO_DEFAULT);
	pmc_enable_periph_clk(BOMBA_PIO_ID);
	pio_configure(BOMBA_PIO, PIO_OUTPUT_0, BOMBA_IDX_MASK, PIO_DEFAULT);


	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT_VERDE_PIO_ID);
	pmc_enable_periph_clk(BUT_VERMELHO_PIO_ID);
	pmc_enable_periph_clk(BUT_BRANCO_PIO_ID);
	pmc_enable_periph_clk(BUT_AMARELO_PIO_ID);
	pmc_enable_periph_clk(BUT_PRETO_PIO_ID);


	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	pio_configure(BUT_VERDE_PIO, PIO_INPUT, BUT_VERDE_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	pio_configure(BUT_VERMELHO_PIO, PIO_INPUT, BUT_VERMELHO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	pio_configure(BUT_BRANCO_PIO, PIO_INPUT, BUT_BRANCO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	pio_configure(BUT_AMARELO_PIO, PIO_INPUT, BUT_AMARELO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	pio_configure(BUT_PRETO_PIO, PIO_INPUT, BUT_PRETO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );


	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_callback);
	
	pio_handler_set(BUT_VERDE_PIO,
	BUT_VERDE_PIO_ID,
	BUT_VERDE_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_verde_callback);
	
	pio_handler_set(BUT_VERMELHO_PIO,
	BUT_VERMELHO_PIO_ID,
	BUT_VERMELHO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_vermelho_callback);
	
	
	pio_handler_set(BUT_BRANCO_PIO,
	BUT_BRANCO_PIO_ID,
	BUT_BRANCO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_branco_callback);
	
	pio_handler_set(BUT_AMARELO_PIO,
	BUT_AMARELO_PIO_ID,
	BUT_AMARELO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_amarelo_callback);
	
	pio_handler_set(BUT_PRETO_PIO,
	BUT_PRETO_PIO_ID,
	BUT_PRETO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but_preto_callback);
	

	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 5); 
	
	NVIC_EnableIRQ(BUT_VERDE_PIO_ID);
	NVIC_SetPriority(BUT_VERDE_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT_VERMELHO_PIO_ID);
	NVIC_SetPriority(BUT_VERMELHO_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT_BRANCO_PIO_ID);
	NVIC_SetPriority(BUT_BRANCO_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT_AMARELO_PIO_ID);
	NVIC_SetPriority(BUT_AMARELO_PIO_ID, 5);
	
	NVIC_EnableIRQ(BUT_PRETO_PIO_ID);
	NVIC_SetPriority(BUT_PRETO_PIO_ID, 5);
	
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_enable_interrupt(BUT_VERDE_PIO, BUT_VERDE_IDX_MASK);
	pio_enable_interrupt(BUT_VERMELHO_PIO, BUT_VERMELHO_IDX_MASK);
	pio_enable_interrupt(BUT_BRANCO_PIO, BUT_BRANCO_IDX_MASK);
	pio_enable_interrupt(BUT_AMARELO_PIO, BUT_AMARELO_IDX_MASK);
	pio_enable_interrupt(BUT_PRETO_PIO, BUT_PRETO_IDX_MASK);
		
}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}



void task_bomb1(void){
	  	xSemaphoreB1 = xSemaphoreCreateBinary();
		  xSemaphoreB2 = xSemaphoreCreateBinary();
		  xSemaphoreB3 = xSemaphoreCreateBinary();
		  xSemaphoreB4 = xSemaphoreCreateBinary();
		  xSemaphoreB5 = xSemaphoreCreateBinary();
		  xSemaphoreB5 = xSemaphoreCreateBinary();
		  
		  io_init();
		  
	while(true){
		
		if(xSemaphoreTake(xSemaphoreB1, ( TickType_t ) 100) == pdTRUE ){
			pio_set(VALVE_PIO, VALVE_IDX_MASK);
			// Tempo teste: descobrir o tempo para encher metade de um copo
			vTaskDelay(10000); 
			pio_clear(VALVE_PIO, VALVE_IDX_MASK);
			
		}
		vTaskDelay(100); 
	}
	

}

void task_bomb2(void){
	while(true){
		if(xSemaphoreTake(xSemaphoreB2, ( TickType_t ) 100) == pdTRUE ){
			pio_set(BOMBA_PIO, BOMBA_IDX_MASK);
			// Tempo teste: descobrir o tempo para encher metade de um copo
			vTaskDelay(10000);
			pio_clear(BOMBA_PIO, BOMBA_IDX_MASK);
			
		}
		vTaskDelay(100); 
	}
}

void task_bomb3(void){
}

void task_bomb4(void){
}

void task_bomb5(void){
}

void task_bomb6(void){
}



/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
	
  
   /* Create task to handler touch */
   if (xTaskCreate(task_bomb1, "Bomb 1", TASK_BOMB1_STACK_SIZE, NULL, TASK_BOMB1_STACK_PRIORITY, NULL) != pdPASS) {
	    printf("Failed to create test BOMB 1 task\r\n");
    }
	
if (xTaskCreate(task_bomb2, "Bomb 2", TASK_BOMB2_STACK_SIZE, NULL, TASK_BOMB2_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test BOMB 1 task\r\n");
	}
	
/*		if (xTaskCreate(task_bomb3, "Bomb 3", TASK_BOMB3_STACK_SIZE, NULL, TASK_BOMB3_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test BOMB 1 task\r\n");
	}
	
	if (xTaskCreate(task_bomb4, "Bomb 4", TASK_BOMB4_STACK_SIZE, NULL, TASK_BOMB4_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test BOMB 1 task\r\n");
	}
	
	if (xTaskCreate(task_bomb5, "Bomb 5", TASK_BOMB5_STACK_SIZE, NULL, TASK_BOMB5_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test BOMB 1 task\r\n");
	}
	
	if (xTaskCreate(task_bomb6, "Bomb 6", TASK_BOMB6_STACK_SIZE, NULL, TASK_BOMB6_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test BOMB 1 task\r\n");
	}*/
	
	

  /* Start the scheduler. */
  vTaskStartScheduler();

  while(1){

  }


	return 0;
}