#include<stm32h7xx.h>
// analog pin pa3->ahb1->adc_12_inp15;
//PA0->ADC1_INP16,
#define eoc (1ul<<2);
uint32_t sensor;
uint32_t x;
void clock_init(void){
	//RCC->CR|=RCC_CR_HSI48ON;
		//hse on and wait for it to satabilize;
		//hse 24mhz;
		RCC->CR|=RCC_CR_HSEON; //
		while (!(RCC->CR & (1<<17)));

		//pll mux  hse clock is selected.
		RCC->PLLCKSELR|=(2ul<<0);
		//pll 1 is  getting 8mhz
		RCC->PLLCKSELR|=(3ul<<4);
		//pll 2,3 is getting 12mHz
		RCC->PLLCKSELR|=(2ul<<12); // pll 2
		//RCC->PLLCKSELR|=(2ul<<20); //pll 3
		//pll1_p_ck for sys_ck, pll1_r_ck for traceportc
		RCC->PLL1DIVR|=(69ul<<0); // mul by 70; pll1= 420Mhz;
		RCC->PLL1DIVR|=(1ul<<24); // pll1_r_clk =240mhz;
		RCC->PLL1DIVR|=1ul<<9; //pll1_p_clk =240mhz;
		RCC->PLL1DIVR|=1ul<<16; // pll1_q_clk=240mhz;

		RCC->PLL2DIVR|=(10ul<<0); //mul by 11; pll2=132Mhz;
		RCC->PLL2DIVR|=(1ul<<9); //pll2_q_clk=66mhz;

		//sws select pll_1_r;
		RCC->CFGR|=(3ul<<3);

		//RCC->SRDCCIPR&=~(3ul<<16);
		//RCC->SRDCCIPR|=(2ul<<16);

		RCC->CR|=RCC_CR_PLL1ON;
		while (!(RCC->CR & (1<<25)));

		RCC->CR|=RCC_CR_PLL2ON;
		while(!(RCC->CR&(1<<27)));
		while(!(RCC->CR&(1<<14)));

		//while(!(RCC->CR&(1<<15))){
			//x=5;
		//}
		//RCC->CR|=RCC_CR_PLL3ON;
		//pll1_p_ck

}
void pa0_adc_init(void){
	/** configure the adc gpio pin **/

	/* enable  clock access to gpioA*/
	RCC->AHB4ENR|=RCC_AHB4ENR_GPIOAEN;

	/*set PA0 to analog mode*/
	GPIOA->MODER|=(3ul<<0);
	/** configure the adc module **/

	/* enable  clock access to adc*/
	RCC->AHB1ENR|=RCC_AHB1ENR_ADC12EN;
	/*conversion sequence start*/
	ADC1->SQR1|=16ul<<6;
	/*conversion sequence length*/
	ADC1->SQR1|=0ul<<0;

	/*adc module enable*/
	ADC1->CR&=~ADC_CR_ADCAL;
	ADC1->CR&=~ADC_CR_JADSTART;
	ADC1->CR&=~ADC_CR_ADSTART;
	ADC1->CR&=~ADC_CR_ADSTP;
	ADC1->CR&=~ADC_CR_ADDIS;
	ADC1->CR&=~ADC_CR_ADEN;
	ADC1->CR&=~ADC_CR_DEEPPWD;// enabling adc
	ADC1->CR|=ADC_CR_ADVREGEN;
	ADC12_COMMON->CCR|=11ul<<18;
	ADC1->PCSEL|=ADC_PCSEL_PCSEL_16;
	ADC1->CFGR&=~(5UL<<2);// 16 BIT MODE
	//ADC1->CFGR|=(5ul<<2);
	//ADC1->SMPR2|=(5UL<<18);//SAMPLING 810.5 ADC CYCLES

	//ADC12_COMMON->CCR|=(ADC_CCR_CKMODE_0|ADC_CCR_CKMODE_1); // adc_sclk/4 (Synchronous clock mode)
	ADC1->DIFSEL&=~ADC_DIFSEL_DIFSEL_16;// single channel adc

	ADC1->CR|=ADC_CR_ADEN; //enable adc

	//ovsr=1024 ovss[3:0]=1010
}

void start_conversion(void){
	/*start adc conversion*/
	ADC1->CR|=ADC_CR_ADSTART;
}

uint32_t adc_read(void){
	/* wait for conversion to complete*/
	//ADC1->CFGR|=ADC_CFGR_CONT;
	//while(!(ADC1->ISR&(1ul<<1))){}
	//while(!(ADC1->ISR&(1ul<<0))){};
	while(!(ADC1->ISR &(1ul<<2))) {
		sensor=ADC1->ISR;
	}
	//while(!(ADC1->ISR&(1ul<<3))){
		//;
	//}
	/*read converted result*/
	return (ADC1->DR);
}


uint32_t sensor_value;

int main(void){
	x=0;
	clock_init();
	x=10;
	pa0_adc_init();
	x=15;
	while(1){
		start_conversion();
		sensor_value=adc_read();
		for(int i=0;i<1000000;i++){
		}
		//sensor_value=sensor_value>>12;
	}

}




































/*
void adc_init(){

	//select pin when adcen is off
	//ADCAL=0, JADSTART=0, ADSTART=0, ADSTP=0, ADDIS=0 and ADEN=0)  except for bit ADVREGEN
	//which must be 1
	ADC1->CR&=~ADC_CR_ADCAL;
	ADC1->CR&=~ADC_CR_JADSTART;
	ADC1->CR&=~ADC_CR_ADSTART;
	ADC1->CR&=~ADC_CR_ADSTP;
	ADC1->CR&=~ADC_CR_ADDIS;
	ADC1->CR&=~ADC_CR_ADEN;
	ADC1->CR|=ADC_CR_ADVREGEN;

	//adc_pcsel
	//The software is allowed to write these bits only when ADSTART=0 and JADSTART=0 (which ensures that no conversion is ongoing).
	ADC1->PCSEL|=ADC_PCSEL_PCSEL_15;
	ADC1->CFGR&=~(5UL<<2);// 16 BIT MODE
	ADC1->SMPR2|=(5UL<<15);//SAMPLING 810.5 ADC CYCLES
	ADC12_COMMON->CCR|=(ADC_CCR_CKMODE_0|ADC_CCR_CKMODE_1); // adc_sclk/4 (Synchronous clock mode)
	ADC1->DIFSEL&=~ADC_DIFSEL_DIFSEL_15;// single channel adc
	ADC1->CR|=ADC_CR_DEEPPWD;// enabling adc
	ADC1->CR|=ADC_CR_ADEN; //enable adc
	//ADC1->CFGR|=
}

void calibrate_adc(){

  Write ADCALDIF=0 before launching a calibration which will be applied for singleended input conversions.
• Write ADCALDIF=1 before launching a calibration which will be applied for differential
input conversions
Write ADCALLIN=1 before launching a calibration which will run the linearity calibration
same time as the offset calibration.
• Write ADCALLIN=0 before launching a calibration which will not run the linearity
calibration but only the offset calibration.

DIFSEL in the ADC_DIFSEL register, ADCx_CCR register
and the control bits ADCAL and ADEN in the ADC_CR register, only if the ADC is disabled
(ADEN must be equal to 0).
 *

}

int main(void){
	clock_int();
	gpio_init();
	adc_init();
	uint32_t x=0;
	ADC1->CFGR|=ADC_CFGR_CONT;
	ADC1->CR|=ADC_CR_ADSTART;
	while(1){
	GPIOB->ODR^=GPIO_ODR_OD0;
	while(!(ADC_ISR_EOC&(1UL<<0))); // wait for eoc
	GPIOB->ODR^=GPIO_ODR_OD0;
	x|=ADC_DR_RDATA;
	ADC1->IER&=~(ADC_ISR_EOC);
	}
}
*/
