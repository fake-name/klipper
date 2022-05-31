// Code to setup clocks and gpio on stm32f1
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "board/irq.h" // irq_disable
#include "board/usb_cdc.h" // usb_request_bootloader
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main


/****************************************************************
 * Clock setup
 ****************************************************************/

#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 2)

// Map a peripheral address to its enable bits
struct cline
lookup_clock_line(uint32_t periph_base)
{
    if (periph_base >= AHB1PERIPH_BASE) {
        uint32_t bit = 1 << ((periph_base - AHB1PERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->AHBENR, .bit=bit};
    } else if (periph_base >= APB2PERIPH_BASE) {
        uint32_t bit = 1 << ((periph_base - APB2PERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->APB2ENR, .rst=&RCC->APB2RSTR, .bit=bit};
    } else {
        uint32_t bit = 1 << ((periph_base - APB1PERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->APB1ENR, .rst=&RCC->APB1RSTR, .bit=bit};
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - APB2PERIPH_BASE) / 0x400;
    RCC->APB2ENR |= 1 << rcc_pos;
    RCC->APB2ENR;
}

// Main clock setup called at chip startup
static void
clock_setup(void)
{
    // Configure and enable PLL
    uint32_t cfgr;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL) {
        // Configure 72Mhz PLL from external crystal (HSE)
        RCC->CR |= RCC_CR_HSEON;
        uint32_t div = CONFIG_CLOCK_FREQ / (CONFIG_CLOCK_REF_FREQ / 2);
        cfgr = 1 << 16U; // Previously for STM32F1 RCC_CFGR_PLLSRC_Pos
        if ((div & 1) && div <= 16)
            cfgr |= RCC_CFGR_PLLXTPRE_PREDIV1_Div2; // Previously for STM32F1 RCC_CFGR_PLLXTPRE_HSE_DIV2
        else
            div /= 2;
        cfgr |= (div - 2) << 18U; // Previously for STM32F1 RCC_CFGR_PLLMULL_Pos
    } else {
        // Configure 72Mhz PLL from internal 8Mhz oscillator (HSI)
        uint32_t div2 = (CONFIG_CLOCK_FREQ / 8000000) * 2;
        cfgr = ((0 << 16U) // Previously for STM32F1 RCC_CFGR_PLLSRC_Pos
                | ((div2 - 2) << 18U)); // Previously for STM32F1 RCC_CFGR_PLLMULL_Pos
    }
    cfgr |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR2_ADCPRE12_DIV8; // TODO: Previously RCC_CFGR_ADCPRE_DIV8 NO IDEA
    RCC->CFGR = cfgr;
    RCC->CR |= RCC_CR_PLLON;

    // Set flash latency
    FLASH->ACR = FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE; // Previously (2 << FLASH_ACR_LATENCY_Pos)

    // Wait for PLL lock
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Switch system clock to PLL
    RCC->CFGR = cfgr | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) // Previously RCC_CFGR_SWS_Msk
        ;
}


/****************************************************************
 * GPIO setup
 ****************************************************************/

static void
stm32f3_alternative_remap(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
	uint32_t temp = 0x00;
	uint32_t temp_2 = 0x00;

	/* Check the parameters */
	// assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	// assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
	// assert_param(IS_GPIO_AF(GPIO_AF));

	temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
	GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4));
	temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
	GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
}

// static void
// stm32f1_alternative_remap(uint32_t mapr_mask, uint32_t mapr_value)
// {
//     // The MAPR register is a mix of write only and r/w bits
//     // We have to save the written values in a global variable
//     static uint32_t mapr = 0;
//
//     mapr &= ~mapr_mask;
//     mapr |= mapr_value;
//     SYSCFG->MAPR = mapr; // Previously AFIO
// }

#define STM_OSPEED 0x1 // ~10Mhz at 50pF

void GPIO_Init(GPIO_TypeDef* GPIOx, uint8_t pin, uint32_t mode, int pullup)
{
	uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;
	uint32_t tmpreg = 0x00;
	uint32_t pupd = (pullup < 0 ? 2 : (pullup == 0 ? 0 : 1));

	/*-------------------------- Configure the port pins -----------------------*/
	/*-- GPIO Mode Configuration --*/
	for (pinpos = 0x00; pinpos < 0x10; pinpos++)
	{
		pos = ((uint32_t)0x01) << pinpos;

		/* Get the port pins position */
		currentpin = (pin) & pos;

		if (currentpin == pos)
		{
			if ((mode == GPIO_OUTPUT) || (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) || (mode & GPIO_OPEN_DRAIN))
			{
				/* Check Speed mode parameters */
				// assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));

				/* Speed mode configuration */
				GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
				//TODO: constant speed?
				GPIOx->OSPEEDR |= ((uint32_t)(STM_OSPEED) << (pinpos * 2));

				/* Check Output mode parameters */
				// assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));

				/* Output mode configuration */
				GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos));

				//TODO: WTF? is it even correct
				GPIOx->OTYPER |= (uint16_t)(
					((uint16_t)(mode & GPIO_OPEN_DRAIN ? 1 : 0)) << ((uint16_t)pinpos)
				);
			}

			GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
			GPIOx->MODER |= (((uint32_t)mode) << (pinpos * 2));

			/* Use temporary variable to update PUPDR register configuration, to avoid
			   unexpected transition in the GPIO pin configuration. */
			tmpreg = GPIOx->PUPDR;
			tmpreg &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));

			// TODO: Pretty sure its correct
			tmpreg |= ((pupd) << (pinpos * 2));
			GPIOx->PUPDR = tmpreg;
		}
	}
}

// Set the mode and extended function of a pin
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
	GPIO_Init(regs, gpio % 16, mode, pullup);
    // uint32_t pos = gpio % 16, shift = (pos % 8) * 4, msk = 0xf << shift, cfg;
    // if (mode == GPIO_INPUT) {
    //     cfg = pullup ? 0x8 : 0x4;
    // } else if (mode == GPIO_OUTPUT) {
    //     cfg = STM_OSPEED;
    // } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
    //     cfg = 0x4 | STM_OSPEED;
    // } else if (mode == GPIO_ANALOG) {
    //     cfg = 0x0;
    // } else {
    //     if (mode & GPIO_OPEN_DRAIN)
    //         // Alternate function with open-drain mode
    //         cfg = 0xc | STM_OSPEED;
    //     else if (pullup > 0)
    //         // Alternate function input pins use GPIO_INPUT mode on the stm32f1
    //         cfg = 0x8;
    //     else
    //         cfg = 0x8 | STM_OSPEED;
    // }
    // if (pos & 0x8)
    //     regs->CRH = (regs->CRH & ~msk) | (cfg << shift);
    // else
    //     regs->CRL = (regs->CRL & ~msk) | (cfg << shift);
	//
    // if (pullup > 0)
    //     regs->BSRR = 1 << pos;
    // else if (pullup < 0)
    //     regs->BSRR = 1 << (pos + 16);

    if (gpio == GPIO('A', 13) || gpio == GPIO('A', 14)) {
		// Disable SWD to free PA13, PA14
		stm32f3_alternative_remap(GPIOA, 13, 2);
		stm32f3_alternative_remap(GPIOA, 14, 2);
		// TODO: make sure this is right
		// stm32f3_alternative_remap(AFIO_MAPR_SWJ_CFG_Msk,
		// 						  AFIO_MAPR_SWJ_CFG_DISABLE);
	}

    // STM32F1 remaps functions to pins in a very different
    // way from other STM32s.
    // Code below is emulating a few mappings to work like an STM32F4
    uint32_t func = (mode >> 4) & 0xf;
    if (func == 1) {
        // TIM2
        if (gpio == GPIO('A', 15) || gpio == GPIO('B', 3)) {
			stm32f3_alternative_remap(GPIOA, 15, func); // TIM2 on PA15
			stm32f3_alternative_remap(GPIOB, 3, func); // TIM2 on PB3
			// TODO: not sure about this?
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_TIM2_REMAP_Msk,
			// 	AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1
			// );
		} else if (gpio == GPIO('B', 10) || gpio == GPIO('B', 11)) {
			stm32f3_alternative_remap(GPIOB, 10, func); // TIM2 on PB10
			stm32f3_alternative_remap(GPIOB, 11, func); // TIM2 on PB11
			// TODO: not sure about this?
			// stm32f3_alternative_remap(AFIO_MAPR_TIM2_REMAP_Msk,
			// 						  AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2);
		}

    } else if (func == 2) {
        // TIM3 and TIM4
        if (gpio == GPIO('B', 4) || gpio == GPIO('B', 5)) {
			stm32f3_alternative_remap(GPIOB, 4, func); // TIM3 on PB4
			stm32f3_alternative_remap(GPIOB, 5, func); // TIM3 on PB5
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_TIM3_REMAP_Msk,
			// 	AFIO_MAPR_TIM3_REMAP_PARTIALREMAP
			// );
		} else if (gpio == GPIO('C', 6) || gpio == GPIO('C', 7)
                 || gpio == GPIO('C', 8) || gpio == GPIO('C', 9)) {
			stm32f3_alternative_remap(GPIOC, 6, func); // TIM3 on PC6
			stm32f3_alternative_remap(GPIOC, 7, func); // TIM3 on PC7
			stm32f3_alternative_remap(GPIOC, 8, func); // TIM3 on PC8
			stm32f3_alternative_remap(GPIOC, 9, func); // TIM3 on PC9
            // stm32f3_alternative_remap(AFIO_MAPR_TIM3_REMAP_Msk,
            //                           AFIO_MAPR_TIM3_REMAP_FULLREMAP);
		} else if (gpio == GPIO('D', 12) || gpio == GPIO('D', 13)
                 || gpio == GPIO('D', 14) || gpio == GPIO('D', 15)) {
			stm32f3_alternative_remap(GPIOD, 12, func); // TIM4 on PD12
			stm32f3_alternative_remap(GPIOD, 13, func); // TIM4 on PD13
			stm32f3_alternative_remap(GPIOD, 14, func); // TIM4 on PD14
			stm32f3_alternative_remap(GPIOD, 15, func); // TIM4 on PD15
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_TIM4_REMAP_Msk,
			// 	AFIO_MAPR_TIM4_REMAP
			// );
		}
    } else if (func == 4) {
        // I2C
        if (gpio == GPIO('B', 8) || gpio == GPIO('B', 9)) {
			stm32f3_alternative_remap(GPIOB, 8, func); // I2C on PB8
			stm32f3_alternative_remap(GPIOB, 9, func); // I2C on PB9
			// stm32f3_alternative_remap(AFIO_MAPR_I2C1_REMAP_Msk,
			// 						  AFIO_MAPR_I2C1_REMAP);
		}
    } else if (func == 5) {
        // SPI
        if (gpio == GPIO('B', 3) || gpio == GPIO('B', 4)
            || gpio == GPIO('B', 5)) {
			stm32f3_alternative_remap(GPIOB, 3, func); // SPI on PB3
			stm32f3_alternative_remap(GPIOB, 4, func); // SPI on PB4
			stm32f3_alternative_remap(GPIOB, 5, func); // SPI on PB5
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_SPI1_REMAP_Msk,
			// 	AFIO_MAPR_SPI1_REMAP
			// );
		}
    } else if (func == 7) {
        // USART
		// TODO: Verify alternative functions
        if (gpio == GPIO('B', 6) || gpio == GPIO('B', 7)) {
			stm32f3_alternative_remap(GPIOB, 6, func); // USART1_TX on PB6
			stm32f3_alternative_remap(GPIOB, 7, func); // USART1_RX on PB7
			// stm32f3_alternative_remap(AFIO_MAPR_USART1_REMAP_Msk,
			// 						  AFIO_MAPR_USART1_REMAP);
		} else if (gpio == GPIO('D', 5) || gpio == GPIO('D', 6)) {
			stm32f3_alternative_remap(GPIOD, 5, func); // USART2_TX on PD5
			stm32f3_alternative_remap(GPIOD, 6, func); // USART2_RX on PD6
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_USART2_REMAP_Msk,
			// 	AFIO_MAPR_USART2_REMAP
			// );
		} else if (gpio == GPIO('D', 8) || gpio == GPIO('D', 9)) {
			stm32f3_alternative_remap(GPIOD, 8, func); // USART3_TX on PD8
			stm32f3_alternative_remap(GPIOD, 9, func); // USART3_RX on PD9
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_USART3_REMAP_Msk,
			// 	AFIO_MAPR_USART3_REMAP_FULLREMAP
			// );
		}
    } else if (func == 9) {
        // CAN
        if (gpio == GPIO('B', 8) || gpio == GPIO('B', 9)) {
			stm32f3_alternative_remap(GPIOB, 8, func); // CAN_RX on PB8
			stm32f3_alternative_remap(GPIOB, 9, func); // CAN_TX on PB9
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_CAN_REMAP_Msk,
			// 	AFIO_MAPR_CAN_REMAP_REMAP2
			// );
		}
        if (gpio == GPIO('D', 0) || gpio == GPIO('D', 1)) {
			stm32f3_alternative_remap(GPIOD, 0, func); // CAN_RX on PD0
			stm32f3_alternative_remap(GPIOD, 1, func); // CAN_TX on PD1
			// stm32f3_alternative_remap(
			// 	AFIO_MAPR_CAN_REMAP_Msk,
			// 	AFIO_MAPR_CAN_REMAP_REMAP3
			// );
		}
    }
}


/****************************************************************
 * USB bootloader
 ****************************************************************/

// Reboot into USB "HID" bootloader
static void
usb_hid_bootloader(void)
{
    irq_disable();
	// TODO: BKPEN does not exist?
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Previously for STM32F1  | RCC_APB1ENR_BKPEN
    PWR->CR |= PWR_CR_DBP;
	// TODO: Taken from F4
    // BKP->DR4 = 0x424C; // HID Bootloader magic key
	RTC->BKP4R = 0x424C;
    PWR->CR &=~ PWR_CR_DBP;
    NVIC_SystemReset();
}

#define USB_BOOT_FLAG_ADDR (CONFIG_RAM_START + CONFIG_RAM_SIZE - 4096)
#define USB_BOOT_FLAG 0x55534220424f4f54 // "USB BOOT"

static void
usb_reboot_for_dfu_bootloader(void)
{
	irq_disable();
	*(uint64_t*)USB_BOOT_FLAG_ADDR = USB_BOOT_FLAG;
	NVIC_SystemReset();
}

// Reboot into USB "stm32duino" bootloader
// static void
// usb_stm32duino_bootloader(void)
// {
//     irq_disable();
// 	// TODO: BKPEN does not exist?
//     RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Previously for STM32F1  | RCC_APB1ENR_BKPEN
//     PWR->CR |= PWR_CR_DBP;
//     BKP->DR10 = 0x01; // stm32duino bootloader magic key
//     PWR->CR &=~ PWR_CR_DBP;
//     NVIC_SystemReset();
// }

// Handle USB reboot requests
void
usb_request_bootloader(void)
{
    if (CONFIG_STM32_FLASH_START_800)
        usb_hid_bootloader();
	// TODO: this is probably not gonna work?
    // else if (CONFIG_STM32_FLASH_START_2000)
    //     usb_stm32duino_bootloader();
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    // Run SystemInit() and then restore VTOR
    SystemInit();
    SCB->VTOR = (uint32_t)VectorTable;

    // Reset peripheral clocks (for some bootloaders that don't)
    RCC->AHBENR = 0x14;
    RCC->APB1ENR = 0;
    RCC->APB2ENR = 0;

    // Setup clocks
    clock_setup();

    // Disable JTAG to free PA15, PB3, PB4
    enable_pclock(SYSCFG_BASE); // previously for STM32F1 AFIO_BASE
    if (CONFIG_STM32F103GD_DISABLE_SWD) {
		// GigaDevice clone can't enable PA13/PA14 at runtime - enable here
		//TODO: absolutely no idea about this one
		stm32f3_alternative_remap(GPIOA, 13, 2);
		stm32f3_alternative_remap(GPIOA, 14, 2);
		// stm32f3_alternative_remap(
		// 	AFIO_MAPR_SWJ_CFG_Msk,
		// 	AFIO_MAPR_SWJ_CFG_DISABLE
		// );
	} else {
		//TODO: absolutely no idea about this neither here
		stm32f3_alternative_remap(GPIOA, 13, 2);
		stm32f3_alternative_remap(GPIOA, 14, 2);
		// stm32f3_alternative_remap(
		// 	AFIO_MAPR_SWJ_CFG_Msk,
		// 	AFIO_MAPR_SWJ_CFG_JTAGDISABLE
		// );
	}

    sched_main();
}
