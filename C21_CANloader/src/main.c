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
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "conf_bootloader.h"
#include "sam_ba_monitor.h"
#include "can_sam_ba.h"

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{
	uint32_t app_start_address;

	/* Load the Reset Handler address of the application */
	app_start_address = *(uint32_t *)(APP_START_ADDRESS + 4);

	/**
	 * Test reset vector of application @APP_START_ADDRESS+4
	 * Stay in SAM-BA if *(APP_START+0x4) == 0xFFFFFFFF
	 * Application erased condition
	 */
	if (app_start_address == 0xFFFFFFFF) {
		/* Stay in bootloader */
		return;
	}

	// NOTE: Pin-based bootloading was removed for this application

	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *) APP_START_ADDRESS);

	/* Rebase the vector table base address */
	SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

	/* Leaving bootloader memory section */
	//BOOT_EXIT_SIGNAL;

	/* Jump to application Reset Handler in the application */
	asm("bx %0"::"r"(app_start_address));
}

void general_io_pin_setup(void) {
	struct port_config pin_output;
	port_get_config_defaults(&pin_output);
	pin_output.direction = PORT_PIN_DIR_OUTPUT;

	port_pin_set_config(LED_WARNING, &pin_output);
	port_pin_set_config(LED_ERROR, &pin_output);
	port_pin_set_config(LED_ACTIVITY, &pin_output);

	port_pin_set_config(PIN_MOTOR_P1, &pin_output);
	port_pin_set_config(PIN_MOTOR_P2, &pin_output);
	port_pin_set_config(PIN_MOTOR_P3, &pin_output);
	port_pin_set_config(PIN_MOTOR_P4, &pin_output);
	port_pin_set_config(PIN_MOTOR_P5, &pin_output);
	port_pin_set_config(PIN_MOTOR_P6, &pin_output);

	port_pin_set_output_level(PIN_MOTOR_P1, false);
	port_pin_set_output_level(PIN_MOTOR_P2, false);
	port_pin_set_output_level(PIN_MOTOR_P3, false);
	port_pin_set_output_level(PIN_MOTOR_P4, false);
	port_pin_set_output_level(PIN_MOTOR_P5, false);
	port_pin_set_output_level(PIN_MOTOR_P6, false);

	port_pin_set_output_level(LED_ACTIVITY, true);
	port_pin_set_output_level(LED_WARNING, true);
}

int main (void)
{
	general_io_pin_setup();

	/* Jump in application if condition is satisfied */
	check_start_application();

	system_init();

	/* Insert application code here, after the board has been initialized. */
	can_open();

	// TODO: Set ERROR pin ON
	for (;;) {  }
}

void CAN0_Handler(void) {
	uint32_t status = can_read_interrupt_status(&can_sam_ba);

	if(status & CAN_RX_FIFO_1_NEW_MESSAGE) {
		can_clear_interrupt_status(&can_sam_ba, CAN_RX_FIFO_1_NEW_MESSAGE);


	}
}
