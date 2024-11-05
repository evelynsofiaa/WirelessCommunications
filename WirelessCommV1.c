/* 
 * File:   TXRXDisplay.c
 * Author: Sergio Fierro
 *
 * Created on 31 de octubre de 2024, 09:40 AM
 */

// Configuración del microcontrolador
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2 // System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = XT_XT     // XT_XT para 4MHz
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#include <xc.h>

#define _XTAL_FREQ 4000000  // Cristal de 4 MHz

// Constants
#define Delay_LCD       50           // Delay used for LCD ENABLE pin
#define Delay_Shift     50          // Delay used to shift characters 

#define StartByteRFID 0x0A           // El bit que identifica el inicio
#define StopByteRFID  0x0D           // El bit que identifica el final

void config_Usart() {
    BRGH = 0;                     // Low speed or high speed serial
    SYNC = 0;                     // Asynchronous data
    TX9 = 0;                      // 8-bit data transmission
    TXEN = 1;                     // Enable asynchronous serial TX
    SPBRG = 25;                   // Baud rate-decide from table 20-3
    SPEN = 1;                     // Serial port enabled
    RX9 = 0;                      // 8-bit reception mode
    CREN = 1;                     // Enable continuous reception
}

void Tx_byte(char x) {
    while (TXIF == 0) {           // Wait for TX buffer
    }                             // To be empty
    TXREG = x;                    // Put char(byte) to TX
}

int RFID_Card_Byte() {
    int RxByte;                   // Declare RxByte variable
    while (RCIF == 0) {           // Wait for RX buffer to be empty
    }
    if (OERR) {                   // OERR: overrun
        CREN = 0;                 // Occurs when errors in RX
        RxByte = RCREG;           // Abort RX and perform
        RxByte = RCREG;           // Two reads in RCREG
        __delay_ms(100);          // Wait 100ms
        CREN = 1;                 // Re-enable serial reception
    }
    return RCREG;                 // Receive byte
}

void RFID_read() {
    char RFID_data[20]; // Buffer para almacenar el mensaje
    int k = 0;
    int RxByte = RFID_Card_Byte();

    // Verificar si el primer byte coincide con el inicio del mensaje
    if (RxByte == StartByteRFID) {
        while (1) {
            RxByte = RFID_Card_Byte();
            if (RxByte == StopByteRFID) {
                // Llegó el fin del mensaje
                break;
            }
            RFID_data[k++] = RxByte; // Almacenar el byte en el buffer
            if (k >= 20) {
                // Evitar desbordamiento del buffer
                break;
            }
        }

        // Añadir un carácter nulo al final para marcar el fin del string
        RFID_data[k] = '\0';

        // Transmitir el mensaje recibido por el puerto serial
        for (int i = 0; i < k; i++) {
            Tx_byte(RFID_data[i]);
        }
    }
}


void main() {
    config_Usart();               // Configura USART antes de enviar datos
    char string[] = "Hola";      // Intento de mandar 'hola' por TX
    int i;
    for (i = 0; i < 4; i++) {
        Tx_byte(string[i]);
    }
    while (1);
}
