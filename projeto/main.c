#include "STM32F10x.h"
#include <cmsis_os.h>

//Pin definition
//LEDs
#define LED1 0  //PA0
#define LED2 1  //PA1
#define LED3 2  //PA2
#define LED4 15 //PA15
#define LED5 8  //PA8
#define LED6 6  //PA6
#define LED7 5  //PA5
#define LED8 11 //PA11

//LCD display
#define LCD_RS 15
#define LCD_EN 12

//Switches
#define SW1 12  //PB12
#define SW2 13  //PB13
#define SW3 14  //PB14
#define SW4 15  //PB15
#define SW5 5   //PB5
#define SW6 4   //PB4
#define SW7 3   //PB3
#define SW8 3   //PA3
#define SW9 4   //PA4
#define SW10 8  //PB8
#define SW11 9  //PB9
#define SW12 11 //PB11
#define SW13 10 //PB10
#define SW14 7  //PA7
#define SW15 15 //PC15
#define SW16 14 //PC14
#define SW17 13 //PC13

//Potentiometer
#define POT 1 //PB1

//Buzzer
#define BUZ 0 //PB0

#define LEDS 8

const unsigned int pinosLeds[LEDS] = {LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8}; //fila com os defines dos leds em ordem
unsigned int t = 0x1000;

void setup_RedPill();
void buzzDesliga();
void lcd_init(void);
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_print(char * str);

osThreadId leds_ID, gray_ID, pot_ID, buz_ID, ctl_ID, frq_ID, dis_ID;

void ledON (void const *argument){
int i;

	
while(1){
	for ( i = 0;i < LEDS; i++){		
		if ( i % 2 != 0) GPIOA->ODR |= (1 << pinosLeds[i]);	
		else GPIOA->ODR &= ~(1 << pinosLeds[i]);			
	}	
	osDelay(t);
	for( i = 0; i < LEDS; i++){
		if(i % 2 == 0)GPIOA->ODR |= (1 << pinosLeds[i]);
		else GPIOA->ODR &= ~(1 << pinosLeds[i]);				
	}	
	osDelay(t);
	}	
}
void ledGRAY(void const *argument){
 while (1) {
	 int count;
	 int grayCode = 0;
	 int i;		 
        for ( count = 0; count < (1 << LEDS); count++) {
            grayCode = count ^ (count>>1);  // Calcula o código Gray para o contador atual					
            // Configura os LEDs com base no código Gray
            for (i = 0; i < LEDS; i++) {							
                if (grayCode & (1 << i)) {
                    GPIOA->ODR |= (1 << pinosLeds[i]);  // Acende o LED se o bit estiver definido no código Gray
                } else {
                    GPIOA->ODR &= ~(1 << pinosLeds[i]); // Apaga o LED se o bit não estiver definido
                }
								osDelay(t>>5);
            }
        }
    }
}
void ledPOT(void const *argument){
unsigned int ADC_CTRL;
uint16_t ADC_VALUE;
ADC_CTRL = 0x00000000;
ADC_VALUE=0x0000;

	while(ADC_CTRL !=0x00000007){
		ADC1->SQR3 = 9;	/* choose channel 9 as the input */
		ADC1->CR2 = 1;	/* ADON = 1 (start conversion) */
		ADC1->CR2 |= 2; //autocalibration
		osDelay(2);
		while((ADC1->SR&(1<<1)) == 0); /* wait until the EOC flag is set */		
		ADC_VALUE = ADC1->DR;
		switch(ADC_VALUE >> 3){	
			case 0 ... 64:
			{	GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR &= ~(1<<LED2);
				GPIOA->ODR &= ~(1<<LED3);
				GPIOA->ODR &= ~(1<<LED4);
				GPIOA->ODR &= ~(1<<LED5);
				GPIOA->ODR &= ~(1<<LED6);
				GPIOA->ODR &= ~(1<<LED7);
				GPIOA->ODR &= ~(1<<LED8);
				osDelay(100);
				break;}			
			case 65 ... 128:
			{	GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR |= (1<<LED2);
				GPIOA->ODR &= ~(1<<LED3);
				GPIOA->ODR &= ~(1<<LED4);
				GPIOA->ODR &= ~(1<<LED5);
				GPIOA->ODR &= ~(1<<LED6);
				GPIOA->ODR &= ~(1<<LED7);
				GPIOA->ODR &= ~(1<<LED8);
				osDelay(100);
				break;}			
			case 129 ... 192:
			{	GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR |= (1<<LED2);
				GPIOA->ODR |= (1<<LED3);
				GPIOA->ODR &= ~(1<<LED4);
				GPIOA->ODR &= ~(1<<LED5);
				GPIOA->ODR &= ~(1<<LED6);
				GPIOA->ODR &= ~(1<<LED7);
				GPIOA->ODR &= ~(1<<LED8);
				osDelay(100);
				break;}				
			case 193 ... 256:
			{	GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR |= (1<<LED2);
				GPIOA->ODR |= (1<<LED3);
				GPIOA->ODR |= (1<<LED4);
				GPIOA->ODR &= ~(1<<LED5);
				GPIOA->ODR &= ~(1<<LED6);
				GPIOA->ODR &= ~(1<<LED7);
				GPIOA->ODR &= ~(1<<LED8);
				osDelay(100);
				break;}		
			case 257 ... 320:
			{	GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR |= (1<<LED2);
				GPIOA->ODR |= (1<<LED3);
				GPIOA->ODR |= (1<<LED4);
				GPIOA->ODR |= (1<<LED5);
				GPIOA->ODR &= ~(1<<LED6);
				GPIOA->ODR &= ~(1<<LED7);
				GPIOA->ODR &= ~(1<<LED8);
				osDelay(100);
				break;}			
			case 321 ... 384:
			{ GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR |= (1<<LED2);
				GPIOA->ODR |= (1<<LED3);
				GPIOA->ODR |= (1<<LED4);
				GPIOA->ODR |= (1<<LED5);
				GPIOA->ODR |= (1<<LED6);
				GPIOA->ODR &= ~(1<<LED7);
				GPIOA->ODR &= ~(1<<LED8);
				osDelay(100);
		    break;}			
			case 385 ... 448:
			{ GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR |= (1<<LED2);
				GPIOA->ODR |= (1<<LED3);
				GPIOA->ODR |= (1<<LED4);
				GPIOA->ODR |= (1<<LED5);
				GPIOA->ODR |= (1<<LED6);
				GPIOA->ODR |= (1<<LED7);
				GPIOA->ODR &= ~(1<<LED8);
				osDelay(100);
				break;}			
			case 449 ... 512:
			{ GPIOA->ODR |=(1<<LED1);
				GPIOA->ODR |= (1<<LED2);
				GPIOA->ODR |= (1<<LED3);
				GPIOA->ODR |= (1<<LED4);
				GPIOA->ODR |= (1<<LED5);
				GPIOA->ODR |= (1<<LED6);
				GPIOA->ODR |= (1<<LED7);
				GPIOA->ODR |= (1<<LED8);
				osDelay(100);
				break;}					
			default:
			{	GPIOA->ODR = 0x00000000;
				break;}
		}
	}
}

void BUZZ(void const *argument){
unsigned int ADC_CTRL;
int ADC_VALUE;
ADC_CTRL = 0x00000000;
ADC_VALUE=0x0000;
while(ADC_CTRL !=0x00000007){
		ADC1->SQR3 = 9;	/* choose channel 9 as the input */
		ADC1->CR2 = 1;	/* ADON = 1 (start conversion) */
		ADC1->CR2 |= 2; //autocalibration
		osDelay(2);
		while((ADC1->SR&(1<<1)) == 0); /* wait until the EOC flag is set */		
		ADC_VALUE = ADC1->DR;
		RCC->APB1ENR |= (1<<1);
    TIM3->CCR2 = 5000;
		TIM3->CCER = 0x1 << 8; /*CC2P = 0, CC2E = 1 */
		TIM3->CCMR2 = 0x0030;  /* toggle channel 3 */
		TIM3->ARR = -1*((ADC_VALUE << 4) + 0xf);				
		TIM3->CR1 = 1;
	}
}

void FREQ (void const *argument){
	int flag = (int)argument;
	unsigned int aux = 0;
	if (flag == 1){ 
		aux = t<<1;
		while(t < aux){
			t++;
			osDelay(5);
		}
	}
	if (flag == 2){
		aux = t>>1;
		while(t > aux){
			t--;
			osDelay(5);
		}
	}
	if (flag == 3){ 
		t = 0x1000;
	}
	if (flag == 4){
		unsigned int ADC_CTRL;
		int ADC_VALUE;
		ADC_CTRL = 0x00000000;
		ADC_VALUE=0x0000;
		ADC1->SQR3 = 9;	/* choose channel 9 as the input */
		ADC1->CR2 = 1;	/* ADON = 1 (start conversion) */
		ADC1->CR2 |= 2; //autocalibration
		osDelay(2);
		while((ADC1->SR&(1<<1)) == 0); /* wait until the EOC flag is set */		
		ADC_VALUE = ADC1->DR;
		t = ADC_VALUE;
	}
	if (flag == 5) t = 500;
	if (flag == 6) t = 2000;
	if (flag == 7) t = 10000;
}

void DISP (void const *argument){
while(1){
	osDelay(500);
	lcd_command(0x01); //limpa lcd
	lcd_command(0x80); //primeira posição da primeira linha
	lcd_print("Teste da Redpill");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_command(0xC0);
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_print("Aperte as teclas");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	
	}
}
osThreadDef(ledON, osPriorityNormal, 1, 0);
osThreadDef(ledGRAY, osPriorityNormal, 1, 0);
osThreadDef(ledPOT, osPriorityNormal, 1, 0);
osThreadDef(BUZZ, osPriorityNormal, 1, 0);
osThreadDef(FREQ, osPriorityAboveNormal, 1, 0);
osThreadDef (DISP, osPriorityNormal, 1, 0);

void controle(void const *argument){
    long int SW_CTRL;
    SW_CTRL = 0x00000000;
	
		dis_ID = osThreadCreate(osThread(DISP), NULL);
	
	
    while (SW_CTRL != 0x0001FFFF){				
				if (buz_ID == NULL) buzzDesliga();
        if ((GPIOB->IDR & (1 << SW5)) == 0){
                if (leds_ID == NULL) {
                leds_ID = osThreadCreate(osThread(ledON), NULL);               
                osThreadTerminate(gray_ID);
                gray_ID = NULL;                
                osThreadTerminate(pot_ID);
                pot_ID = NULL;                
                osThreadTerminate(buz_ID);
                buz_ID = NULL;
								}
        }
        if ((GPIOB->IDR & (1 << SW6)) == 0){
               if (gray_ID == NULL) {
                gray_ID = osThreadCreate(osThread(ledGRAY), NULL);                
                osThreadTerminate(leds_ID);
                leds_ID = NULL;                
                osThreadTerminate(pot_ID);
                pot_ID = NULL;                
                osThreadTerminate(buz_ID);
                buz_ID = NULL;
							 }
        }
        if ((GPIOB->IDR & (1 << SW7)) == 0){
        		osThreadTerminate(buz_ID);
            buz_ID = NULL;
            osThreadTerminate(pot_ID);
            pot_ID = NULL;
            if (pot_ID == NULL) {
                pot_ID = osThreadCreate(osThread(ledPOT), NULL);
                osThreadTerminate(leds_ID);
                leds_ID = NULL;
                osThreadTerminate(gray_ID);
                gray_ID = NULL;
                osThreadTerminate(buz_ID);
                buz_ID = NULL;
            }
        }
        if ((GPIOA->IDR & (1 << SW8)) == 0){ 
            if (buz_ID == NULL) {
							  osThreadTerminate(pot_ID); 
                pot_ID = NULL;
								pot_ID = osThreadCreate(osThread(ledPOT), NULL);
                buz_ID = osThreadCreate(osThread(BUZZ), NULL);                
                osThreadTerminate(leds_ID);
                leds_ID = NULL;            
                osThreadTerminate(gray_ID);
                gray_ID = NULL;
            }
        }
								
				if ((GPIOB->IDR & (1 << SW1)) == 0){ 
					osDelay(300);
					frq_ID = osThreadCreate(osThread(FREQ), (void *)1);
				}
				if ((GPIOB->IDR & (1 << SW2)) == 0){ 	
					osDelay(300);
					frq_ID = osThreadCreate(osThread(FREQ), (void *)2);
				}
				if ((GPIOB->IDR & (1 << SW3)) == 0){
					osDelay(300);
					frq_ID = osThreadCreate(osThread(FREQ), (void *)3);
					}      
				if ((GPIOB->IDR & (1 << SW4)) == 0){
					osDelay(300);
					frq_ID = osThreadCreate(osThread(FREQ), (void *)4);
					} 
				if ((GPIOC->IDR & (1 << SW15)) == 0){
					osDelay(300);
					frq_ID = osThreadCreate(osThread(FREQ), (void *)5);
					}
				if ((GPIOC->IDR & (1 << SW16)) == 0){
					osDelay(300);
					frq_ID = osThreadCreate(osThread(FREQ), (void *)6);
					}
				if ((GPIOC->IDR & (1 << SW17)) == 0){
					osDelay(300);
					frq_ID = osThreadCreate(osThread(FREQ), (void *)7);
					}
        osDelay(100);
    }
}

osThreadDef(controle, osPriorityRealtime, 1, 0);

int main (void){
	osKernelInitialize ();                    // initialize CMSIS-RTOS
	setup_RedPill();
	lcd_init();
	ctl_ID = osThreadCreate(osThread(controle), NULL);
	osKernelStart ();                         // start thread execution
	while(1){
        ;
	}
}

//--------------------------------Setups e Inits--------------------------------------------------//

void setup_RedPill(){
	//int16_t swa, swb, swc;  //Variables to read the switches according to the port it is connected
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AF clock
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	osDelay(100);
	RCC->APB2ENR |= 0xFC |(1<<9);		//ENABLE clocks for GPIOs and ADC1
	//Setting up outputs for leds
	ADC1->CR2 = 1;	/* ADON = 1 (power-up) */
	ADC1->CR2 |=(1<<2); //enable calibration
	ADC1->SMPR2 = 1<<3; /* SMP1 = 001 */
	osDelay(1);	/* wait 1us to make sure the adc module is stable */
	GPIOA->CRL = 0x43344333;	//PA3, PA4 and PA7: inputs (switches)
	GPIOA->CRH = 0x33333333;  //PA8 - PA15: outputs (leds)
	//Settig up inputs for switches
	GPIOB->CRL = 0x4444440B; //PB0 set for output+alternate wave form, since it is connected to buzzer.
	GPIOB->CRH = 0x44444444;
	GPIOB->ODR = 0xF000; //set pull-up in PB12 - PB15
	GPIOC->CRH = 0x44444444;
	GPIOC->ODR = 0xFFFFFFFF; //set pull-up in GPIOC
	osDelay(1); //wait for I/O setup
	GPIOA->ODR &=~(1<<LCD_RS); //Turn off LED4
	osDelay(1); //wait for LED4 to turn off
}
void buzzDesliga(void){
    // Desativa o canal 2 do timer TIM3
    TIM3->CCER &= ~(1 << 8);  // Desabilita o CC2E (Output Compare) para o canal 2
    // Reseta o valor do registrador CCR2 para parar a geração do sinal
    TIM3->CCR2 = 0;
    // Desliga o timer TIM3
    TIM3->CR1 &= ~1;  // Define o bit CEN (Counter Enable) como 0 para parar o timer
}
void lcd_putValue(unsigned char value){
	uint16_t aux;
	aux = 0x0000; //clear aux
	GPIOA->BRR = (1<<5)|(1<<6)|(1<<8)|(1<<11); /* clear PA5, PA6, PA8, PA11 */
	aux = value & 0xF0;
	aux = aux>>4;
	GPIOA->BSRR = ((aux&0x0008)<<8) | ((aux&0x0004)<<3) | ((aux&0x0002)<<5) | ((aux&0x0001)<<8);
	GPIOA->ODR |= (1<<LCD_EN); /* EN = 1 for H-to-L pulse */
	osDelay(3);			/* make EN pulse wider */
	GPIOA->ODR &= ~ (1<<LCD_EN);	/* EN = 0 for H-to-L pulse */
	osDelay(1);			/* wait	*/
	GPIOA->BRR = (1<<5)|(1<<6)|(1<<8)|(1<<11); /* clear PA5, PA6, PA8, PA11 */
	aux = 0x0000; //clear aux
	aux = value & 0x0F;
	GPIOA->BSRR = ((aux&0x0008)<<8) | ((aux&0x0004)<<3) | ((aux&0x0002)<<5) | ((aux&0x0001)<<8);
	GPIOA->ODR |= (1<<LCD_EN); /* EN = 1 for H-to-L pulse */
	osDelay(3);			/* make EN pulse wider */
  GPIOA->ODR &= ~(1<<LCD_EN);	/* EN = 0 for H-to-L pulse */
  osDelay(1);			/* wait	*/
}
void lcd_command(unsigned char cmd){
	GPIOA->ODR &= ~ (1<<LCD_RS);	/* RS = 0 for command */
	lcd_putValue(cmd);
}
void lcd_data(unsigned char data){
	GPIOA->ODR |= (1<<LCD_RS);	/* RS = 1 for data */
	lcd_putValue(data); 
}
void lcd_print(char * str){
  unsigned char i = 0;
	while(str[i] != 0){
		lcd_data(str[i]); 
		i++;
	}
}
void lcd_init(){
	osDelay(15);
	GPIOA->ODR &= ~(1<<LCD_EN);	/* LCD_EN = 0 */
	osDelay(3); 			/* wait 3ms */
	lcd_command(0x33); //lcd init.
	osDelay(5);
	lcd_command(0x32); //lcd init.
	osDelay(3);
	lcd_command(0x28); // 4-bit mode, 1 line and 5x8 charactere set
	osDelay(3);
	lcd_command(0x0e); // display on, cursor on
	osDelay(3);
	lcd_command(0x01); // display clear
	osDelay(3);
	lcd_command(0x06); // move right
	osDelay(3);
}