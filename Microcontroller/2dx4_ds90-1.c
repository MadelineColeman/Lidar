// Madeline Coleman, colemm6, 400251330
//Bus Speed: 80MHz
//LED: PN1
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

uint16_t	dev=0x52;

int status=0;
uint16_t angleSteps = 5*512/360;
uint16_t count = 0;
uint8_t sensorState=0;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 //activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R=0b00000011;
	GPIO_PORTN_DEN_R=0b00000011;
	return;
}
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	
	GPIO_PORTH_DIR_R=0b00001111;													 								
	GPIO_PORTH_DEN_R=0b00001111;											
	return;
}

void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 //activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        //allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;       								    // make PM0 an input, PM0 is reading if the button is pressed or not 
  GPIO_PORTM_DEN_R = 0b00000001;
	return;
}

void DutyCycleCW(uint16_t delay) {
	GPIO_PORTH_DATA_R = 0b00001100;
	SysTick_Wait1ms(5);
	GPIO_PORTH_DATA_R = 0b00000110;
	SysTick_Wait1ms(5);
	GPIO_PORTH_DATA_R = 0b00000011;
	SysTick_Wait1ms(5);
	GPIO_PORTH_DATA_R = 0b00001001;
	SysTick_Wait1ms(5);
}

void DutyCycleCCW(uint16_t delay) {
	GPIO_PORTH_DATA_R = 0b00001001;
	SysTick_Wait1ms(2);
	GPIO_PORTH_DATA_R = 0b00000011;
	SysTick_Wait1ms(2);
	GPIO_PORTH_DATA_R = 0b00000110;
	SysTick_Wait1ms(2);
	GPIO_PORTH_DATA_R = 0b00001100;
	SysTick_Wait1ms(2);
}

void ToF_init(void){
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	status = VL53L1X_ClearInterrupt(dev);
  GPIO_PORTN_DATA_R = 0b00000010;
	status = VL53L1X_SensorInit(dev);
	GPIO_PORTN_DATA_R = 0b00000000;
	FlashAllLEDs();
	Status_Check("SensorInit", status);
  status = VL53L1X_StartRanging(dev);
	status = VL53L1X_SetDistanceMode(dev, 2);
	FlashAllLEDs();
	Status_Check("StartRanging", status);
}

int main(void) {
  UART_Init();
  uint16_t Distance;
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	PortM_Init();
	PortH_Init();
	SysTick_Init();
	ToF_init();
	
	int input  = UART_InChar(); //wait for input from python
	
	while(1){
		if ((GPIO_PORTM_DATA_R & 0b00000001) == 1){ //check for button press
			count = 0;
			sprintf(printf_buffer,"%u\r\n", 9999); //send 9999 to python program
			UART_printf(printf_buffer);
			for(int i = 1; i <= 512; i++){
				DutyCycleCW(1); //rotate motor
				count++;
				if(count%angleSteps == 0){ //check if 10 degree increment has been reached 
					GPIO_PORTN_DATA_R = 0b000000010; //light the appropriate LED
					SysTick_Wait1ms(10);
					status = VL53L1X_GetDistance(dev, &Distance); //take distance measurement
					sprintf(printf_buffer,"%u\r\n", count);
					UART_printf(printf_buffer); //send the angle steps to the python program
					sprintf(printf_buffer,"%u\r\n", Distance);
					UART_printf(printf_buffer); //send the distance to the python program
				}
				else{
					GPIO_PORTN_DATA_R = 0b000000000;
				}
			}
			SysTick_Wait1ms(5);
			for (int j = 0; j <512; j++){ //rotate motor counter clockwise so wires dont tangle
				DutyCycleCCW(1);
			}
		}
	}
}



#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
       
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
		GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}
