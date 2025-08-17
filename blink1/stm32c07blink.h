/*
 * stm32c07blink.h
 *
 * Purpose: Blink an LED without using HAL
 *  Created on: Aug 14, 2025
 *      Author: Edward Hu
 */

#pragma once
#ifndef INC_STM32C07BLINK_H_
#define INC_STM32C07BLINK_H_

/*
 * Registers (pg 50, 194)
 */

#define PERIPH_OFFSET	0x40000000

// Turn on the clock (GPIO is off by default to save power)
#define RCC_OFFSET 		0x00021000
#define RCC_START      	(PERIPH_OFFSET + RCC_OFFSET)
#define RCC_IOPENR      (*(volatile unsigned int *)(0x34 + RCC_START))
#define RCC_APBENR1 	(*(volatile unsigned int *)(0x3C + RCC_START))
#define RCC_TIM2_EN 	1U

// Timer shenanigans (Timer 2 is literally at the "bottom" of the Periph offset too lmaoo) pg 51
#define TIM2_CR1 		(*(volatile unsigned int*)(0x00 + PERIPH_OFFSET)) // Enable the channel
#define TIM2_DIER 		(*(volatile unsigned int*)(0x0C + PERIPH_OFFSET)) // Enable the interrupts
#define TIM2_SR 		(*(volatile unsigned int*)(0x10 + PERIPH_OFFSET)) // The interrupt status flag
#define TIM2_EGR		(*(volatile unsigned int*)(0x14 + PERIPH_OFFSET)) // Ability to create a reset event
#define TIM2_CNT	 	(*(volatile unsigned int*)(0x24 + PERIPH_OFFSET)) // The actual counter is at 0x24
#define TIM2_PSC        (*(volatile unsigned int*)(0x28 + PERIPH_OFFSET)) // PSC to split it lower
#define TIM2_ARR	 	(*(volatile unsigned int*)(0x2C + PERIPH_OFFSET)) // The reset value
#define TIM2_CR1_CEN	1U
#define TIM2_DIER_EN	1U
#define TIM2_SR_UIF 	1U
#define TIM2_EGR_UG     1U
#define TIM2_CNTRST		99U

// GPIO Port A
#define IOPORT_OFFSET   0x10000000
#define GPIOA_START		(IOPORT_OFFSET + PERIPH_OFFSET)

// Mostly just need Mode to set the pin to output
#define GPIOx_MODER		0x00
#define GPIOx_OTYPER	0x04
#define GPIOx_OSPEEDR 	0x08
#define GPIOx_PUPDR		0x0C
#define GPIOx_IDR 		0x10
#define GPIOx_ODR 		0x14
#define GPIOx_BSRR 		0x18
#define GPIOx_LCKR 		0x1C
#define GPIOx_AFRL 		0x20
#define GPIOx_AFRH		0x24
#define GPIOx_BRR		0x28

#define GPIOA_MODE 		(*(volatile unsigned int *)(GPIOA_START + GPIOx_MODER))
#define GPIOA_ODR		(*(volatile unsigned int *)(GPIOA_START + GPIOx_ODR))
#define GPIO_AEN 		1U

#define PIN5			(1U<<5)
#define PIN1			(1U<<1)
#define LED_PIN 		PIN1

#endif /* INC_STM32C07BLINK_H_ */
