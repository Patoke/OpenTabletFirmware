#include <algorithm>
#include <cstdint>

#include "gd32f3x0.h"

extern "C" {
extern void __libc_init_array();
extern void __libc_fini_array();

extern void SystemInit();
extern int main();

extern unsigned int _sidata;
extern unsigned int _sdata;
extern unsigned int _edata;

extern unsigned int _sbss;
extern unsigned int _ebss;

extern unsigned int _estack;
}

namespace {
	__attribute__((noinline)) auto setUp() -> void {
		std::copy(&_sidata, &_sidata + (&_edata - &_sdata), &_sdata);
		std::fill(&_sbss, &_ebss, 0);

		__libc_init_array();

		SystemInit();
	}

	[[noreturn]] __attribute__((noinline)) auto tearDown() -> void {
		__libc_fini_array();
		while (true) {}
	}
	[[noreturn]] __attribute__((section(".after_vectors"), naked)) auto startup() -> void {
		setUp();

		asm volatile("ldr r0, = main");
		asm volatile("blx r0");

		tearDown();
	}
}  // namespace

extern "C" {
void __attribute__((section(".after_vectors"), weak, naked)) Reset_Handler() {
	startup();
}

void __attribute__((weak)) Default_Handler() {
	while (true) {}
}

//void __attribute__((weak, alias("Default_Handler"), nothrow)) Reset_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) NMI_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) HardFault_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) MemManage_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) BusFault_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) UsageFault_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) SVC_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) DebugMon_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) PendSV_Handler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) SysTick_Handler();

void __attribute__((weak, alias("Default_Handler"), nothrow)) WWDGT_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) LVD_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) RTC_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) FMC_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) RCU_CTC_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) EXTI0_1_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) EXTI2_3_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) EXTI4_15_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TSI_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) DMA_Channel0_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) DMA_Channel1_2_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) DMA_Channel3_4_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) ADC_CMP_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER0_BRK_UP_TRG_COM_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER0_Channel_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER1_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER2_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER5_DAC_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER13_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER14_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER15_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) TIMER16_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) I2C0_EV_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) I2C1_EV_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) SPI0_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) SPI1_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) USART0_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) USART1_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) CEC_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) I2C0_ER_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) I2C1_ER_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) USBFS_WKUP_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) DMA_Channel5_6_IRQHandler();
void __attribute__((weak, alias("Default_Handler"), nothrow)) USBFS_IRQHandler();

// ----------------------------------------------------------------------------

using pHandler = void (*)();

// ----------------------------------------------------------------------------

// The vector table.
// This relies on the linker script to place at correct location in memory.
__attribute__((section(".isr_vector"), used)) pHandler g_pfnVectors[] = {
	reinterpret_cast<pHandler>(&_estack),
	Reset_Handler,
	NMI_Handler,
	HardFault_Handler,
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0,
	0,
	0,
	0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,
	WWDGT_IRQHandler,
	LVD_IRQHandler,
	RTC_IRQHandler,
	FMC_IRQHandler,
	RCU_CTC_IRQHandler,
	EXTI0_1_IRQHandler,
	EXTI2_3_IRQHandler,
	EXTI4_15_IRQHandler,
	TSI_IRQHandler,
	DMA_Channel0_IRQHandler,
	DMA_Channel1_2_IRQHandler,
	DMA_Channel3_4_IRQHandler,
	ADC_CMP_IRQHandler,
	TIMER0_BRK_UP_TRG_COM_IRQHandler,
	TIMER0_Channel_IRQHandler,
	TIMER1_IRQHandler,
	TIMER2_IRQHandler,
	TIMER5_DAC_IRQHandler,
	0,
	TIMER13_IRQHandler,
	TIMER14_IRQHandler,
	TIMER15_IRQHandler,
	TIMER16_IRQHandler,
	I2C0_EV_IRQHandler,
	I2C1_EV_IRQHandler,
	SPI0_IRQHandler,
	SPI1_IRQHandler,
	USART0_IRQHandler,
	USART1_IRQHandler,
	0,
	CEC_IRQHandler,
	0,
	I2C0_ER_IRQHandler,
	0,
	I2C1_ER_IRQHandler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	USBFS_WKUP_IRQHandler,
	0,
	0,
	0,
	0,
	0,
	DMA_Channel5_6_IRQHandler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	USBFS_IRQHandler
};
}