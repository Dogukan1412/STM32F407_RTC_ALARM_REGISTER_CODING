#include "stm32f4xx.h"

#define OCAK				((uint8_t)0x01)
#define SUBAT				((uint8_t)0x02)
#define MART				((uint8_t)0x03)
#define NISAN				((uint8_t)0x04)
#define MAYIS				((uint8_t)0x05)
#define HAZIRAN				((uint8_t)0x06)
#define TEMMUZ				((uint8_t)0x07)
#define AGUSTOS				((uint8_t)0x08)
#define EYLUL				((uint8_t)0x09)
#define EKIM				((uint8_t)0x10)
#define KASIM				((uint8_t)0x11)
#define ARALIK				((uint8_t)0x12)

#define PAZARTESI			((uint8_t)0x01)
#define SALI				((uint8_t)0x02)
#define CARSAMBA			((uint8_t)0x03)
#define PERSEMBE			((uint8_t)0x04)
#define CUMA				((uint8_t)0x05)
#define CUMARTESI			((uint8_t)0x06)
#define PAZAR				((uint8_t)0x07)



void CLK_Config(void);
void GPIO_Config(void);
void RTC_Config(void);
void RTC_Set_Time_BCD(uint8_t saat, uint8_t dakika, uint8_t saniye);
void RTC_Set_Date_BCD(uint8_t yil, uint8_t ay, uint8_t gun, uint8_t hafta);
void RTC_Set_Alarm_A(uint8_t gun, uint8_t saat, uint8_t dakika, uint8_t saniye);

int main(void)
{
	CLK_Config();
	GPIO_Config();
	RTC_Config();

	RTC_Set_Date_BCD(0x23,MAYIS,0x16,SALI);
	RTC_Set_Time_BCD(0x17, 0x12,0x00);
	RTC_Set_Alarm_A(0x16,0x17,0x13,0x00);

  while (1)
  {

  }
}

void CLK_Config() // Clock speed for 168MHz
{
	RCC->CR |= 0x00010000;                 // HSEON ENABLE
	while(!(RCC->CR & 0x00020000));        // HSEON Ready Flag wait
	RCC->CR |= 0x00080000;              // CSS ENABLE
	RCC->CR |= 0x01000000;				// PLL ON
	RCC->PLLCFGR |= 0x00400000;        // PLL SRC HSE is selected
	RCC->PLLCFGR |= 0x00000004;       // PLL M 4
	RCC->PLLCFGR |= 0x00005A00;        // PLL N 168
	RCC->PLLCFGR |= 0x00000000;       // PLL P 2
	RCC->CFGR |= 0x00000000;          // AHB PRESCALER 1
	RCC->CFGR |= 0x00080000;          // APB2 PRESCALER 2
	RCC->CFGR |= 0x00001400;          // APB1 PRESCALER 4
	RCC->CIR |= 0x00080000;             // HSE READY FLAG CLEAR
	RCC->CIR |= 0x00800000;             // CSS FLAG CLEAR
}


void GPIO_Config(void)  // User led configuration
{
	RCC->AHB1ENR |= 0x1U << 3U; // D port clock enable

	GPIOD->MODER |= 0x55000000; // pins D12, D13, D14, D15 is selected output mode
	GPIOD->OSPEEDR |= 0xFF000000; // very high speed is selected
	GPIOD->PUPDR |= 0x00000000; // no pull up, pull down
}


void RTC_Config(void)
{
	PWR->CR |= 1 << 8;				// Disable backup domain write protection
	RCC->BDCR |= 1 << 15;   		// RTC clock enable
	RCC->CSR |= 1 << 0;				// Internal low-speed oscillator enable

	//while((RCC->CSR & (1 << 1)) != (RCC->CSR |= (1 << 1)));		// wait to ready LSI

	RCC->BDCR |= 1 << 16;			// Backup domain software reset
	RCC->BDCR &= ~(1 << 16);	    // Backup domain software reset is removing
	RCC->BDCR |= 2 << 8;			// LSI oscillator clock used as the RTC clock

	RTC->WPR = 0xCA;				// write protection key
	RTC->WPR = 0x53;				// write protection key

	RTC->ISR |= 1 << 7;				// Initialization mode

	RTC->CR |= 0 << 8;					// Alarm A disable
	RTC->CR |= 0 << 12;					// Alarm A interrupt disable
	RTC->PRER = (127 << 16) | (249 << 0);		// 32KHz / ((128-1)*(250-1)) = 1Hz = 1s

	RTC->ISR &= ~(1 << 7);				// Removing Initialization mode (Normal Mode)
	RTC->WPR = 0x03;				// write protection key (random number) disable (locked)
}


void RTC_Set_Time_BCD(uint8_t saat, uint8_t dakika, uint8_t saniye)
{
	RTC->WPR = 0xCA;				// write protection key
	RTC->WPR = 0x53;				// write protection key
	RTC->ISR |= 1 << 7;				// Initialization mode
	RTC->TR |= (saat << 16) | (dakika << 8) | (saniye << 0);	  /*(Bits 19:16 HU[3:0]: Hour units in BCD format)
																	(Bit 11:8 MNU[3:0]: Minute units in BCD format)
																	(Bits 3:0 SU[3:0]: Second units in BCD format)*/

	RTC->ISR &= ~(1 << 7);				// Removing Initialization mode (Normal Mode)
	RTC->WPR = 0x03;				// write protection key (random number) disable (locked)
}


void RTC_Set_Date_BCD(uint8_t yil, uint8_t ay, uint8_t gun, uint8_t hafta)
{
	RTC->WPR = 0xCA;				// write protection key
	RTC->WPR = 0x53;				// write protection key
	RTC->ISR |= 1 << 7;				// Initialization mode
	RTC->DR |= (yil << 16) | (ay << 8) | (gun << 0) | (hafta << 13);  /*(Bits 19:16 YU[3:0]: Year units in BCD format)
																		(Bits 11:8 MU: Month units in BCD format)
																		(Bits 3:0 DU[3:0]: Date units in BCD format)
																		(Bits 15:13 WDU[2:0]: Week day units)*/

	RTC->ISR &= ~(1 << 7);				// Removing Initialization mode (Normal Mode)
	RTC->WPR = 0x03;				// write protection key (random number) disable (locked)
}


void RTC_Set_Alarm_A(uint8_t gun, uint8_t saat, uint8_t dakika, uint8_t saniye)
{
	RTC->WPR = 0xCA;				// write protection key
	RTC->WPR = 0x53;				// write protection key
	RTC->ISR |= 1 << 7;				// Initialization mode

	RTC->ALRMAR |= (gun << 24) | (saat << 16) | (dakika << 8) | (saniye << 0);    /*(Bits 27:24 DU[3:0]: Date units or day in BCD format.)
																					(Bits 19:16 HU[3:0]: Hour units in BCD format.)
																					(Bits 11:8 MNU[3:0]: Minute units in BCD format.)
																					(Bits 3:0 SU[3:0]: Second units in BCD format.)*/

	RTC->CR |= 1 << 8;					// Alarm A enable
	RTC->CR |= 1 << 12;					// Alarm A interrupt Enable
	EXTI->IMR |= 1 << 17;				// Interrupt request from line 17 is not masked
	EXTI->RTSR |= 1 << 17;				// Rising trigger enabled (for Event and Interrupt) for input line 17
	NVIC_EnableIRQ(RTC_Alarm_IRQn);		// RTC alarm A is activated in NVIC
	NVIC_SetPriority(RTC_Alarm_IRQn, 0);

	RTC->ISR &= ~(1 << 7);				// Removing Initialization mode (Normal Mode)
	RTC->WPR = 0x03;				// write protection key (random number) disable (locked)
}


void RTC_Alarm_IRQHandler()
{
	RTC->ISR &= ~(0 << 8);		 // Alarm A flag (This flag is cleared by software by writing 0).
	GPIOD->ODR = 0x0000F000;   // D12, D13, D14, D15 pins set
}




/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
