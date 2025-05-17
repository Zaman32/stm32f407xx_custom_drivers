/*
 * stm32f407xx_gpio.h
 *
 *  Created on: May 15, 2025
 *      Author: zaman
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "stm32f407xx.h"


/*
 * Configuration structure for a GPIO pin
 */
typedef struct {
	uint8_t pin_num;
	uint8_t mode; 			// defined in GPIO_PIN_MODES
	uint8_t speed;			// defined in GPIO_PIN_SPEED
	uint8_t pupd_control;	// defined in GPIO_PUPD
	uint8_t op_type;		// defined in GPIO_OUPUT_TYPE
	uint8_t alt_fun_mode;
}gpio_pin_config_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct {

	gpio_reg_def_t *p_gpio_x;
	gpio_pin_config_t pin_config;

}gpio_handle_t;


/*
 * @GPIO_PIN_MODES
 * GPIO modes
 */
#define GPIO_MODE_IN 	0
#define GPIO_MODE_OUT  	1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6
/*
 * @GPIO_PIN_SPEED
 * GPIO speed
 */
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_HIGH 	2
#define GPIO_SPEED_VHIGH 	3

/*
 * @GPIO_PUPD
 * GPIO pull up pull down
 */
#define GPIO_NO_PUPD 	0
#define GPIO_PU			1
#define GPIO_PD			2

/*
 * @GPIO_OUPUT_TYPE
 * GPIO output type
 */
#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD  	1

/******************************************************
 * @fn			- gpio_init
 *
 * @brief		- Initialize peripheral with specific configuration
 *
 * @param[in]	- pin configuration
 *
 * @return		- None
 *
 * @Note		-

 */
void gpio_init(gpio_handle_t pin_config);


/******************************************************
 * @fn			- gpio_deinit
 *
 * @brief		- Reset Peripheral through peripheral reset register in RCC
 *
 * @param[in]	- GPIOx base address
 *
 * @return		- None
 *
 * @Note		-

 */
void gpio_deinit(gpio_reg_def_t *p_gpio_x);


/******************************************************
 * @fn			- peri_clock_control
 *
 * @brief		- Enable or Disable peripheral clock
 *
 * @param[in]	- GPIOx base address
 * @param[in]	- ENABLE or DISABLE peripheral clock
 *
 * @return		- None
 *
 * @Note		-

 */
void peri_clock_control(gpio_reg_def_t *p_gpio_x, uint8_t en_or_dis);

/*
 * Read GPIO input pin
 */
/******************************************************
 * @fn			- read_pin
 *
 * @brief		- Read GPIOx input pin
 *
 * @param[in]	- GPIOx base address
 * @param[in]	- Input pin to read
 *
 * @return		- Pin value
 *
 * @Note		-

 */
uint8_t read_pin(gpio_reg_def_t *p_gpio_x, uint8_t pin_num);


/******************************************************
 * @fn			- read_port
 *
 * @brief		- Read GPIOx input port
 *
 * @param[in]	- GPIOx base address
 *
 * @return		- Input value of GPIOx port
 *
 * @Note		-

 */
uint16_t read_port(gpio_reg_def_t *p_gpio_x);


/******************************************************
 * @fn			-
 *
 * @brief		- Write to GPIOx output pin
 *
 * @param[in]	- GPIOx base address
 * @param[in]	- Output pin to write
 * @param[in]	- Value to write
 *
 * @return		- None
 *
 * @Note		-

 */
void write_pin(gpio_reg_def_t *p_gpio_x, uint8_t pin_num, uint8_t value);


/******************************************************
 * @fn			- write_port
 *
 * @brief		- Write to GPIOx output port
 *
 * @param[in]	- GPIOx base address
 * @param[in]	- Value to write
 *
 * @return		- None
 *
 * @Note		-

 */
void write_port(gpio_reg_def_t *p_gpio_x, uint16_t value);


/******************************************************
 * @fn			- toggle_pin
 *
 * @brief		- Toggle GPIOx pin
 *
 * @param[in]	- GPIOx base address
 * @param[in]	- Pin to toggle
 *
 * @return		- None
 *
 * @Note		-

 */
void toggle_pin(gpio_reg_def_t *p_gpio_x, uint8_t pin_num);


/******************************************************
 * @fn			- irq_control
 *
 * @brief		- Control IRQ
 *
 * @param[in]	- IRQ number
 * @param[in]	- IRQ priority
 * @param[in]	- ENABLE or DISABLE IRQ
 *
 * @return		- None
 *
 * @Note		-

 */
void irq_control(uint8_t irq_number, uint8_t irq_priority, uint8_t en_or_dis);


/******************************************************
 * @fn			- irq_handling
 *
 * @brief		- Handle IRQ
 *
 * @param[in]	- Pin number to handle IRQ
 *
 * @return		- None
 *
 * @Note		-

 */
void irq_handling(uint8_t pin_num);

#endif /* INC_STM32F407XX_GPIO_H_ */
