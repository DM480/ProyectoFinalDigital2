// Librerias necesarias

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

//PROTOTIPOS FUNCIONES

void UART1config(void);
void display(uint8_t valor);

/**
 * main.c
 */

//VARIABLES
uint8_t par1;
uint8_t par2;
uint8_t par3;
uint8_t par4;
uint8_t disp = 0;
uint8_t plus = 0;

void main(void)
//CONFIGURACION DE PUERTOS
{   //Configuracion a 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    //Activacion de puertos A,C y E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //Configuracion de los LEDs como salidas pa los parqueos
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

    //Cofiguración como entradas a los PUSH
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_3);
    //IntMasterEnable();
    UART1config();

//LOOP PRINCIPAL
    while(1){        // Mientras sea verdadero

        //Se leen los botones correspondientes a cada parqueo
        par1 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2);
        par2 = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3);
        par3 = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6);
        par4 = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7);

        //Se verifica si la variable correspondiente a cada PUSH
        //está en 0 o 1 cuando esta en 0 la led amarilla se enciende
        // y si esta en 1 se enciende la roja

        //Pa parqueo 1
        if (par1 == 0){ //Parqueo 1 ocupado
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5, 16); //Se enciende led roja
            disp &= ~(1); //Clear bit 0
        }
        else{               //Parqueo 1 disponible
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5, 32); //Se enciende led verde
            disp |= 1;   //Set bit 0
        }

        //Pa parqueo 2
        if (par2 == 0){ //Parqueo 2 ocupado
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, 4); //Se enciende led roja
            disp &= ~(2); //Clear bit 1
        }
        else{               //Parqueo 2 disponible
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, 8); //Se enciende led verde
            disp |= 2;   //Set bit 1
        }

        //Pa parqueo 3
        if (par3 == 0){ //Parqueo 3 ocupado
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5, 16); //Se enciende led roja
            disp &= ~(4); //Clear bit 2
        }
        else{               //Parqueo 3 disponible
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5, 32); //Se enciende led verde
            disp |= 4;   //Set bit 2
        }

        //Pa parqueo 4
        if (par4 == 0){ //Parqueo 4 ocupado
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, 64); //Se enciende led roja
            disp &= ~(8); //Clear del bit 3
        }
        else{               //Parqueo 4 esta disponible
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, 128); //Se enciende led verde
            disp |= 8;   //Set bit 3
        }

        //Se suma en variable plus, cantidad total de parqueos disponibles, evaluando cada bit de la variable disp
        if (1 & disp){
                    plus+=1;
                }
        if (2 & disp){
                    plus+=1;
                }
        if (4 & disp){
                    plus+=1;
                }
        if (8 & disp){
                    plus+=1;
                }

        display(plus); //display muestra los parqueos disponibles
        UARTCharPut(UART1_BASE, disp); //envio del valor de Disp al ESP32
        plus = 0;  //Se resetea el valor de plus

    }

}

//FUNCIONES



void UART1config(void){
    //Configuracion del UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1); //Se activa clock para UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//Se activa el clock para puerto C tiva
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5); //Se activan los pines 6 y 7 del puerto D
    UARTConfigSetExpClk(UART1_BASE,SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTEnable(UART1_BASE);
}


void display(uint8_t valor){
    //Segun el valor de la variable se enciende cierto numero en el display
    switch(valor){
        case 0:
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, 95); //Se encienden los pines para mostrar un 0
            break;
        case 1:
                    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, 80); //Se encienden los pines para mostrar un 1
                    break;
        case 2:
                    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, 62); //Se encienden los pines para mostrar un 2
                    break;
        case 3:
                    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, 122); //Se encienden los pines para mostrar un 3
                    break;
        case 4:
                    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, 113); //Se encienden los pines para mostrar un 4
                    break;
    }

}
