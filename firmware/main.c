#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/flash.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

uint16_t buff[2048];

volatile int cnt = 0;

void rcc_clock_setup_in_hsi_out_56mhz(void)
{
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(HSI);
	rcc_wait_for_osc_ready(HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);	/* Set. 56MHz Max. 72MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV4);	/* Set. 14MHz Max. 14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);	/* Set. 28MHz Max. 36MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);	/* Set. 56MHz Max. 72MHz */

	/*
	 * Sysclk is running with 56MHz -> 2 waitstates.
	 * 0WS from 0-24MHz
	 * 1WS from 24-48MHz
	 * 2WS from 48-72MHz
	 */
	flash_set_ws(FLASH_LATENCY_2WS);

	/*
	 * Set the PLL multiplication factor to 16.
	 * 8MHz (internal) * 7 (multiplier) / 2 (PLLSRC_HSI_CLK_DIV2) = 56MHz
	 */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL14);

	/* Select HSI/2 as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(PLL);
	rcc_wait_for_osc_ready(PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ppre1_frequency = 28000000;
	rcc_ppre2_frequency = 56000000;
}

static void setup_adc(u32 adc)
{
	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(adc);
	adc_set_continous_conversion_mode(adc);
	// adc_disable_external_trigger_regular() is defective in libopencm3
	ADC_CR2(adc) |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_SWSTART;
	adc_set_right_aligned(adc);
	adc_set_conversion_time_on_all_channels(adc, ADC_SMPR_SMP_1DOT5CYC);

	adc_on(adc);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(adc);
	adc_calibration(adc);

	uint8_t channel_array[16];
	/* Select the channel we want to convert. */
	channel_array[0] = 0;
	adc_set_regular_sequence(adc, 1, channel_array);
}

static void setup_adc_dma(void)
{
	rcc_peripheral_enable_clock(&RCC_APB2ENR, 
			RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN);

	/* ADC pin for AD8307 output */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_ANALOG, GPIO0);

	/* setup DMA */
	rcc_peripheral_enable_clock(&RCC_AHBENR,
			RCC_AHBENR_DMA1EN);

	/* Make sure the ADC doesn't run during config. */
	adc_off(ADC1);
	adc_off(ADC2);

	dma_channel_reset(DMA1, DMA_CHANNEL1);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (u32) &ADC1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL1, (u32) &buff);
	dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_32BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_32BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

	adc_enable_dma(ADC1);

	ADC1_CR1 |= ADC_CR1_DUALMOD_FIM;

	setup_adc(ADC1);
	setup_adc(ADC2);
}


/* Set up all the peripherals */
static void setup(void)
{
	rcc_clock_setup_in_hsi_out_56mhz();

	rcc_peripheral_enable_clock(&RCC_APB2ENR, 
			RCC_APB2ENR_IOPAEN |
			RCC_APB2ENR_IOPBEN |
			RCC_APB2ENR_AFIOEN |
			RCC_APB2ENR_USART1EN);

	rcc_peripheral_enable_clock(&RCC_APB1ENR, 
			RCC_APB1ENR_TIM4EN);

	nvic_enable_irq(NVIC_TIM4_IRQ);

	/* GPIO pin for the LED */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);

	/* GPIO pin for USART TX */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);

	/* Setup USART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);

	/* Finally enable the USART. */
	usart_enable(USART1);

	setup_adc_dma();
}

static void get_samples(void)
{
	const int nsamples = sizeof(buff)/sizeof(*buff);

	// nsamples/2 because we transfer two samples per request in dual mode
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, nsamples/2);
	dma_enable_channel(DMA1, DMA_CHANNEL1);

	while(!(DMA_ISR(DMA1) & DMA_ISR_TCIF(DMA_CHANNEL1))) {}
	DMA_IFCR(DMA1) = DMA_IFCR_CTCIF(DMA_CHANNEL1);

	dma_disable_channel(DMA1, DMA_CHANNEL1);

	printf("DS");

	int n;
	for(n = 0; n < nsamples; n++) {
		printf(" %d", buff[n]);
	}
	printf(" DE\n");
}

/* Provide _write syscall used by libc */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++) {
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	} else {
		errno = EIO;
		return -1;
	}
}

/* Delay execution for some arbitrary amount of time */
void delay(void)
{
	int i;

	for (i = 0; i < 8000000; i++) {
		__asm__("nop");
	}
}

int main(void)
{
	setup();
	printf("boot\n");

	// trigger start of continuous conversion
	ADC_CR2(ADC1) |= ADC_CR2_SWSTART;

	while (1) {
		get_samples();
		delay();
	}

	return 0;
}
