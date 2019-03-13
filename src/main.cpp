/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "stm32f746xx.h"
#include "stm32f7xx.h"

void RCC_init(){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
}

void GPIO_init(){

	// Инициализирует GPIO как AF12 от from, до to вклюительно
	auto G_ini = [](GPIO_TypeDef *GPIO, uint8_t from, uint8_t to){
		uint8_t a = 0;
		uint8_t b = 0;
		for (int i = from; i < to+1; ++i) {
			// AF mode
			GPIO->MODER &= ~(0b11 << (i << 1));
			GPIO->MODER |= (0b10 << (i << 1));
			// AF12
			a = i/8;
			b = i%8;
			GPIO->AFR[a] &= ~(0b1111 << (b << 2));  // reset to zero
			GPIO->AFR[a] |= (0b1100 << (b << 2));
			// MAX SPEED
			GPIO->OSPEEDR |= (0b11 << (i << 1));
		}
	};

	G_ini(GPIOF, 0, 5);
	G_ini(GPIOF, 11, 15);

	G_ini(GPIOH, 5, 7);

	G_ini(GPIOG, 0, 1);
	G_ini(GPIOG, 4, 5);
	G_ini(GPIOG, 8, 8);
	G_ini(GPIOG, 15, 15);

	G_ini(GPIOD, 0, 1);
	G_ini(GPIOD, 8, 10);
	G_ini(GPIOD, 14, 15);

	G_ini(GPIOE, 0, 1);
	G_ini(GPIOE, 7, 15);
	// GPIOF

}

void FMC_init(){
	  /** FMC GPIO Configuration
	  PF0   ------> FMC_A0
	  PF1   ------> FMC_A1
	  PF2   ------> FMC_A2
	  PF3   ------> FMC_A3
	  PF4   ------> FMC_A4
	  PF5   ------> FMC_A5

	  PH5   ------> FMC_SDNWE

	  PF11   ------> FMC_SDNRAS
	  PF12   ------> FMC_A6
	  PF13   ------> FMC_A7
	  PF14   ------> FMC_A8
	  PF15   ------> FMC_A9

	  PG0   ------> FMC_A10
	  PG1   ------> FMC_A11

	  PE7   ------> FMC_D4
	  PE8   ------> FMC_D5
	  PE9   ------> FMC_D6
	  PE10   ------> FMC_D7
	  PE11   ------> FMC_D8
	  PE12   ------> FMC_D9
	  PE13   ------> FMC_D10
	  PE14   ------> FMC_D11
	  PE15   ------> FMC_D12

	  PH6   ------> FMC_SDNE1
	  PH7   ------> FMC_SDCKE1

	  PD8   ------> FMC_D13
	  PD9   ------> FMC_D14
	  PD10   ------> FMC_D15
	  PD14   ------> FMC_D0
	  PD15   ------> FMC_D1

	  PG4   ------> FMC_BA0
	  PG5   ------> FMC_BA1
	  PG8   ------> FMC_SDCLK

	  PD0   ------> FMC_D2
	  PD1   ------> FMC_D3

	  PG15   ------> FMC_SDNCAS

	  PE0   ------> FMC_NBL0
	  PE1   ------> FMC_NBL1 */

	// Настройка на 2 банк
	// SDRAM Control registers 1,2 (FMC_SDCR1,2)

	/*
	 * 1. Program the memory device features into the FMC_SDCRx register.The SDRAM clock
	 * 	hhhhfrequency, RBURST and RPIPE must be programmed in the FMC_SDCR1
	 *	register.
	 * */
	SET_BIT(FMC_Bank5_6->SDCR[0], FMC_SDCR2_RPIPE_0);
	CLEAR_BIT(FMC_Bank5_6->SDCR[0], FMC_SDCR2_RPIPE_1);  	//  01: One HCLK clock cycle delay

	CLEAR_BIT(FMC_Bank5_6->SDCR[0], FMC_SDCR2_RBURST);  	//  0: single read requests are not managed as bursts

	SET_BIT(FMC_Bank5_6->SDCR[0], FMC_SDCR2_SDCLK_1);
	CLEAR_BIT(FMC_Bank5_6->SDCR[0], FMC_SDCR2_SDCLK_0); 	//  10: SDCLK period = 2 x HCLK periods



	SET_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_RPIPE_0);
	CLEAR_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_RPIPE_1);  	//  01: One HCLK clock cycle delay

	CLEAR_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_RBURST);  	//  0: single read requests are not managed as bursts

	SET_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_SDCLK_1);
	CLEAR_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_SDCLK_0); 	//  10: SDCLK period = 2 x HCLK periods



	CLEAR_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_WP);  			//  0: Write accesses allowed

	SET_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_CAS);			//  11: 3 cycles

	SET_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_NB);			//  1: Four internal Banks

	SET_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_MWID_0);
	CLEAR_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_MWID_1);		//  01: 16 bits

	SET_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_NR_0);
	CLEAR_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_NR_1);		//  01: 12 bits

	CLEAR_BIT(FMC_Bank5_6->SDCR[1], FMC_SDCR2_NC);			//  00: 8 bits

	/*
	 * 2. Program the memory device timing into the FMC_SDTRx register. The TRP and TRC
	 * timings must be programmed in the FMC_SDTR1 register.
	 */
	// SDRAM Timing registers 1,2 (FMC_SDTR1,2)
	CLEAR_BIT(FMC_Bank5_6->SDTR[1], 0xFFFFFFFF);		//  Reset all
	SET_BIT(FMC_Bank5_6->SDTR[1], (2 << 24));			//  TRCD[3:0]: Row to column delay
	SET_BIT(FMC_Bank5_6->SDTR[1], (2 << 20));			//  TRP [3:0]: Row precharge delay
	SET_BIT(FMC_Bank5_6->SDTR[1], (2 << 16));			//  TWR [3:0]: Recovery delay
	SET_BIT(FMC_Bank5_6->SDTR[1], (6 << 12));			//  TRC [3:0]: Row cycle delay
	SET_BIT(FMC_Bank5_6->SDTR[1], (4 << 8));			//  TRAS[3:0]: Self refresh time
	SET_BIT(FMC_Bank5_6->SDTR[1], (7 << 4));			//  TXSR[3:0]: Exit Self-refresh delay
	SET_BIT(FMC_Bank5_6->SDTR[1], (2 << 0));			//  TMRD[3:0]: Load Mode Register to Active

	/*
	 * 3. Set MODE bits to ‘001’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
	 * FMC_SDCMR register to start delivering the clock to the memory (SDCKE is driven
	 * high).
	 * */

	// Обязательно за один раз
	FMC_Bank5_6->SDCMR = 1 | (1 << 3) | (1 << 5);

	/*
	 * 4. Wait during the prescribed delay period. Typical delay is around 100 μs (refer to the
	 * SDRAM datasheet for the required delay after power-up).
	 * */
	// Delay(>100 us)
	for (int var = 0; var < 21277; ++var) {}

	/*
	 * 5. Set MODE bits to ‘010’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
	 * FMC_SDCMR register to issue a “Precharge All” command.
	 * */

	// Обязательно за один раз
	FMC_Bank5_6->SDCMR = 2 | (1 << 3) | (1 << 5);

	/*
	 * 6. Set MODE bits to ‘011’, and configure the Target Bank bits (CTB1 and/or CTB2) as well
	 * as the number of consecutive Auto-refresh commands (NRFS) in the FMC_SDCMR
	 * register. Refer to the SDRAM datasheet for the number of Auto-refresh commands that
	 * should be issued. Typical number is 8 (ref to datasheet).
	 * */

	// Обязательно за один раз
	FMC_Bank5_6->SDCMR = 3 | (1 << 3) | (8 << 5);

	/*
	 * 7. Configure the MRD field according to the SDRAM device, set the MODE bits to '100',
	 * and configure the Target Bank bits (CTB1 and/or CTB2) in the FMC_SDCMR register
	 * to issue a "Load Mode Register" command in order to program the SDRAM device.
	 * In particular:
	 * a) the CAS latency must be selected following configured value in FMC_SDCR1/2
	 * 		registers
	 * b) the Burst Length (BL) of 1 must be selected by configuring the M[2:0] bits to 000 in
	 *	 	the mode register. Please refer to SDRAM device datasheet.
	 * 	 	If the Mode Register is not the same for both SDRAM banks, this step has to be
	 *	 	repeated twice, once for each bank, and the Target Bank bits set accordingly.
	 * */
	// Обязательно за один раз
	FMC_Bank5_6->SDCMR = 4 | (1 << 3) | (8 << 5) | (0b00001000110000 << 9);

	/*
	 * 8. Program the refresh rate in the FMC_SDRTR register
	 * The refresh rate corresponds to the delay between refresh cycles. Its value must be
	 * adapted to SDRAM devices.
	 * */
	int SDRAM_refresh_period = 64 /*ms*/;
	int Number_of_rows = 4096;  // Или в два раза больше?
	float COUNT = ((float)(SDRAM_refresh_period*1000)/*us*/ / Number_of_rows);
	int Refresh_rate = (COUNT + 1) * 133 - 20;
	FMC_Bank5_6->SDRTR = Refresh_rate << 1;
}

void FMC_write_32b(uint32_t pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize){
	__IO uint32_t *pSdramAddress = (uint32_t *)pAddress;

	for(; BufferSize != 0; BufferSize--)
	{
	  *(__IO uint32_t *)pSdramAddress = *pSrcBuffer;
	  pSrcBuffer++;
	  pSdramAddress++;
	}
}

void test_RW(){
#define WRITE_READ_ADDR     ((uint32_t)0x0800)
#define SDRAM_DEVICE_ADDR  ((uint32_t)0xD0000000)
#define BUFFER_SIZE         ((uint32_t)0x1000)

	/* Read/Write Buffers */
	uint32_t aTxBuffer[BUFFER_SIZE];
	uint32_t aRxBuffer[BUFFER_SIZE];

	for(int i=0; i<BUFFER_SIZE; i++)
	{
		aTxBuffer[i]=0x12345678+i;	 /* TxBuffer init */
	}
	FMC_write_32b(SDRAM_DEVICE_ADDR+WRITE_READ_ADDR, aTxBuffer, BUFFER_SIZE);
}

int main(void)
{

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

	RCC_init();
	GPIO_init();
	FMC_init();
	test_RW();
	for(;;);
}
