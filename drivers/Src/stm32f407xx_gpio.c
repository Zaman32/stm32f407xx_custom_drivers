/*
 * stm32f407xx_gpio.c
 *
 *  Created on: May 15, 2025
 *      Author: zaman
 */

#include "stm32f407xx_gpio.h"


void gpio_init(gpio_handle_t pin_init_cfg)
{
	// TODO add error checking for invalid config
	// TODO add error checking for invalid pin number

	uint32_t temp = 0;
	// 1. Configure mode
	if (pin_init_cfg.p_gpio_x->MODER <= GPIO_MODE_ANALOG)
	{
		// Non interrupt mode
		temp = (pin_init_cfg.pin_config.mode << ( 2*pin_init_cfg.pin_config.pin_num));
		pin_init_cfg.p_gpio_x->MODER &= ~((0x3) << (2 * pin_init_cfg.pin_config.pin_num)); // clear pin
		pin_init_cfg.p_gpio_x->MODER |= temp; // setting pin
	}
	else
	{
		// Int mode
	}

	// 2. Speed
	temp = (pin_init_cfg.pin_config.speed << ( 2*pin_init_cfg.pin_config.pin_num));
	pin_init_cfg.p_gpio_x->OSPEEDR &= ~((0x3) << (2 * pin_init_cfg.pin_config.pin_num)); // clear pin
	pin_init_cfg.p_gpio_x->OSPEEDR |= temp; // setting pin

	// 3. PUPD
	temp = (pin_init_cfg.pin_config.pupd_control << ( 2*pin_init_cfg.pin_config.pin_num));
	pin_init_cfg.p_gpio_x->PUPDR &= ~((0x3) << (2 * pin_init_cfg.pin_config.pin_num)); // clear pin
	pin_init_cfg.p_gpio_x->PUPDR |= temp; // setting pin

	// 4. OPTYPE
	temp = (pin_init_cfg.pin_config.op_type << ( 2*pin_init_cfg.pin_config.pin_num));
	pin_init_cfg.p_gpio_x->OTYPER &= ~((0x3) << (2 * pin_init_cfg.pin_config.pin_num)); // clear pin
	pin_init_cfg.p_gpio_x->OTYPER |= temp; // setting pin

	// 5. ALT functionality
	if(pin_init_cfg.pin_config.alt_fun_mode == GPIO_MODE_ALTFN)
	{
		uint8_t afr_position =  pin_init_cfg.pin_config.pin_num / 8; // determine AFR high or low register
		uint8_t afr_pin_num = pin_init_cfg.pin_config.pin_num % 8;	 // determine pin number in AFR high or low register

		temp = (pin_init_cfg.pin_config.alt_fun_mode << (4*afr_pin_num));
		pin_init_cfg.p_gpio_x->AFR[afr_position] &= ~(0xF << (4*afr_pin_num));
		pin_init_cfg.p_gpio_x->AFR[afr_position] |= temp;
	}


}


void gpio_deinit(gpio_reg_def_t *p_gpio_x)
{
	if (p_gpio_x == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if (p_gpio_x == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if (p_gpio_x == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if (p_gpio_x == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if (p_gpio_x == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if (p_gpio_x == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if (p_gpio_x == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if (p_gpio_x == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if (p_gpio_x == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}


void peri_clock_control(gpio_reg_def_t *p_gpio_x, uint8_t en_or_dis)
{
	if (en_or_dis == ENABLE)
	{
		if (p_gpio_x == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (p_gpio_x == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (p_gpio_x == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (p_gpio_x == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (p_gpio_x == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (p_gpio_x == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (p_gpio_x == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (p_gpio_x == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (p_gpio_x == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if (p_gpio_x == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
		else if (p_gpio_x == GPIOI)
		{
			GPIOI_PCLK_DIS();
		}
	}
}


uint8_t read_pin(gpio_reg_def_t *p_gpio_x, uint8_t pin_num)
{
	// TODO add error checking for invalid pin number

	return (uint8_t)((p_gpio_x->IDR >> pin_num) & 0x01);
}


uint16_t read_port(gpio_reg_def_t *p_gpio_x)
{
	return (uint16_t)(p_gpio_x->IDR);
}


void write_pin(gpio_reg_def_t *p_gpio_x, uint8_t pin_num, uint8_t value)
{
	// TODO add error checking for invalid value
	// TODO add error checking for invalid pin number

	if (value == GPIO_PIN_SET)
	{
		p_gpio_x->ODR |= (1<<pin_num);
	}
	else
	{
		p_gpio_x->ODR &= ~(1<<pin_num);
	}


}
void write_port(gpio_reg_def_t *p_gpio_x, uint16_t value)
{
	// TODO add error checking for invalid value
	p_gpio_x->ODR = value;
}
void toggle_pin(gpio_reg_def_t *p_gpio_x, uint8_t pin_num)
{
	p_gpio_x->ODR ^= (1<<pin_num);
}
void irq_control(uint8_t irq_number, uint8_t irq_priority, uint8_t en_or_dis);
void irq_handling(uint8_t pin_num);
