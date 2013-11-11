

#include "Hardware.h"
#include <plib.h>


// <editor-fold defaultstate="collapsed" desc="Config Bits">
/*CP: Code-Protect bit
 * Prevents boot and program Flash memory from being read or modified by an external programming device.
 *      1 = Protection is disabled
 *      0 = Protection is enabled*/
#pragma config CP       = OFF
/*BWP: Boot Flash Write-Protect bit
 * Prevents boot Flash memory from being modified during code execution.
 *      1 = Boot Flash is writable
 *      0 = Boot Flash is not writable*/
#pragma config BWP      = OFF
/*PWP<7:0>: Program Flash Write-Protect bits
 * Prevents selected program Flash memory pages from being modified during code execution. The PWP bits
 *  represent the 1?s complement of the number of write-protected program Flash memory pages.
 *      11111111 = Disabled
 *      11111110 = 0xBD00_0FFF
 *      11111101 = 0xBD00_1FFF
 *      ...*/
#pragma config PWP      = OFF
/*ICESEL: In-Circuit Emulator/Debugger Communication Channel Select bit
 *      1 = PGEC2/PGED2 pair is used
 *      0 = PGEC1/PGED1 pair is used*/
#pragma config ICESEL   = ICS_PGx2
/*DEBUG<1:0>: Background Debugger Enable bits (forced to ?11? if code-protect is enabled)
 *      11 = Debugger is disabled
 *      10 = Debugger is enabled
 *      01 = Reserved (same as ?11? setting)
 *      00 = Reserved (same as ?11? setting)*/
#pragma config DEBUG    = OFF
/*FWDTEN: Watchdog Timer Enable bit
 *      1 = The WDT is enabled and cannot be disabled by software
 *      0 = The WDT is not enabled; it can be enabled in software*/
#pragma config FWDTEN   = OFF           // Watchdog Timer
/*WDTPS<4:0>: Watchdog Timer Postscale Select bits
 *      10100 = 1:1048576
 *      10011 = 1:524288
 *      10010 = 1:262144
 *      ...
 *      00001 = 1:2
 *      00000 = 1:1*/
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
/*FCKSM<1:0>: Clock Switching and Monitor Selection Configuration bits
 *      1x = Clock switching is disabled, Fail-Safe Clock Monitor is disabled
 *      01 = Clock switching is enabled, Fail-Safe Clock Monitor is disabled
 *      00 = Clock switching is enabled, Fail-Safe Clock Monitor is enabled*/
#pragma config FCKSM    = CSDCMD
/*OSCIOFNC: CLKO Enable Configuration bit
 *      1 = CLKO output is disabled
 *      0 = CLKO output signal is active on the OSCO pin; the Primary Oscillator must be disabled or configured
 *          for External Clock mode (EC) for the CLKO to be active (POSCMOD<1:0> = 11 or 00)*/
#pragma config OSCIOFNC = OFF
/*POSCMOD<1:0>: Primary Oscillator Configuration bits
 *      11 = Primary Oscillator disabled
 *      10 = HS Oscillator mode selected
 *      01 = XT Oscillator mode selected
 *      00 = External Clock mode selected*/
#pragma config POSCMOD  = HS
/*IESO: Internal External Switchover bit
 *      1 = Internal External Switchover mode is enabled (Two-Speed Start-up is enabled)
 *      0 = Internal External Switchover mode is disabled (Two-Speed Start-up is disabled)*/
#pragma config IESO     = OFF
/*FSOSCEN: Secondary Oscillator Enable bit
 *      1 = Enable the Secondary Oscillator
 *      0 = Disable the Secondary Oscillator*/
#pragma config FSOSCEN  = OFF
/*FNOSC<2:0>: Oscillator Selection bits
 *      111 = Fast RC Oscillator with divide-by-N (FRCDIV)
 *      110 = FRCDIV16 Fast RC Oscillator with fixed divide-by-16 postscaler
 *      101 = Low-Power RC Oscillator (LPRC)
 *      100 = Secondary Oscillator (SOSC)
 *      011 = Primary Oscillator (POSC) with PLL module (XT+PLL, HS+PLL, EC+PLL)
 *      010 = Primary Oscillator (XT, HS, EC)(1)
 *      001 = Fast RC Oscillator with divide-by-N with PLL module (FRCDIV+PLL)
 *      000 = Fast RC Oscillator (FRC)*/
#pragma config FNOSC    = PRIPLL
/*FPLLMUL<2:0>: PLL Multiplier bits
 *      111 = 24x multiplier
 *      110 = 21x multiplier
 *      101 = 20x multiplier
 *      100 = 19x multiplier
 *      011 = 18x multiplier
 *      010 = 17x multiplier
 *      001 = 16x multiplier
 *      000 = 15x multiplier*/
#pragma config FPLLMUL  = MUL_20
/*FPLLIDIV<2:0>: PLL Input Divider bits
 *      111 = 12x divider
 *      110 = 10x divider
 *      101 = 6x divider
 *      100 = 5x divider
 *      011 = 4x divider
 *      010 = 3x divider
 *      001 = 2x divider
 *      000 = 1x divider*/
#pragma config FPLLIDIV = DIV_2
/*FPLLODIV<2:0>: PLL Output Divider bits
 *      111 = PLL output divided by 256
 *      110 = PLL output divided by 64
 *      101 = PLL output divided by 32
 *      100 = PLL output divided by 16
 *      011 = PLL output divided by 8
 *      010 = PLL output divided by 4
 *      001 = PLL output divided by 2
 *      000 = PLL output divided by 1*/
#pragma config FPLLODIV = DIV_1
/*FPBDIV<1:0>: Peripheral Bus Clock Divisor Default Value bits
 *      11 = PBCLK is SYSCLK divided by 8
 *      10 = PBCLK is SYSCLK divided by 4
 *      01 = PBCLK is SYSCLK divided by 2
 *      00 = PBCLK is SYSCLK divided by 1*/
#pragma config FPBDIV   = DIV_8
// </editor-fold>




// EEPROM Constants
#define EEPROM_I2C_BUS              I2C2
#define EEPROM_ADDRESS              0x50        // 0b1010000 Serial EEPROM address


#define MY_SLAVE_ADDRESS            0x15
#define DEVICE_SELECTION            I2C2


#include "FIFOI2C.h"

void DelayTime(int ms)
{
    int i = 0;
    while (i < (ms*6000))
    {
        i++;
    }
}

#define SLAVE_ADDRESS_7_BIT 0x1E


//Pulse Gobbler Delay
int main(void)
{
    int i = 0;
    FIFOI2C_RX_Byte a, b, c;
    uint8 bty[10];
    
    //Configures system for optimum preformance without changing PB divider
    SYSTEMConfig(GetSystemClock(), SYS_CFG_PCACHE | SYS_CFG_WAIT_STATES);

    FIFOI2C_initialize();
//    FIFOI2C_addQueue_readDeviceRegisters(0, 0x00, 2);
//    FIFOI2C_addQueue_readDeviceRegisters(1, 0x00, 1);



    // Enable multi-vector interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    bty[0] = 0x70;
    bty[1] = 0xA0;
    bty[2] = 0x00;

    while(1)
    {
        FIFOI2C_addQueue_writeDeviceRegisters(0, 0x00, bty, 3);
        FIFOI2C_addQueue_readDeviceRegisters(0, 0x00, 3);
        DelayTime(100);
        a = FIFOI2C_readQueue(0);
        b = FIFOI2C_readQueue(0);
        c = FIFOI2C_readQueue(0);
    }




    //I2C2CONbits.SEN = 1;

    //Start Condition


//    I2C_7_BIT_ADDRESS slave7BitAddress;
//    I2C_FORMAT_7_BIT_ADDRESS(slave7BitAddress, SLAVE_ADDRESS_7_BIT, I2C_READ);
//
//
//    //Device 7-bit Addre: 0x1E
//    //write: 0x3c
//    //Read: 0x3D
//
//    // <bit 0> : 0 -> write
//    //           1 -> read
//            int temp_a = 0;
//    unsigned char data;
//    data = 0;
////    I2C2BRG = (1.0/(2.0*FIFOI2C_BAUD_RATE) - 104e-9) * GetPeripheralClock() - 1.5; //-1.5 takes into account rounding for -2.
////    //Enable I2C2 Module
////    I2C2CONbits.ON = 1;
//    while(1)
//    {
//        //I2C2TRN = I2C_GET_7_BIT_ADDRESS_BYTE(slave7BitAddress);
//
//        //Wait for bus idle
//        while (I2C2STATbits.TRSTAT);
//
//        //Start sequence
//        I2C2CONbits.SEN = 1;
//        //Wait for start sequence to finish
//        while(I2C2CONbits.SEN == 1); //~~flicks
//
//        //send device address
//        I2C2TRN = 0X3C; //Write address
//       //TRSTAT: Transmit Status bit
//        while(I2C2STATbits.TRSTAT); //1 = Master transmit is in progress (8 bits + ACK) //~~flicks
//                                    //0 = Master transmit is not in progress
//
//        //Send the addr to read (config reg B) (defualt 0x10)
//        I2C2TRN = 0X01; //Read for demo purposes
//        //Transmit Buffer Full Status bit
//        while(I2C2STATbits.TRSTAT); //1 = Master transmit is in progress (8 bits + ACK) //~~flicks
//                                    //0 = Master transmit is not in progress
//
//        //Repeated Start Condition Enable bit
//        I2C2CONbits.RSEN = 1;
//        while(I2C2CONbits.RSEN); //0 = Repeated Start condition is not in progress //~~flicks
//        //send device address
//        I2C2TRN = 0X3D; //Write address
//        //TRSTAT: Transmit Status bit
//        while(I2C2STATbits.TRSTAT); //1 = Master transmit is in progress (8 bits + ACK)  //~~flicks
//                                    //0 = Master transmit is not in progress
//
//        //Receive Enable bit
//        I2C2CONbits.RCEN = 1;
//        //Receive Buffer Full Status bit
//        while(!I2C2STATbits.RBF); //1 = Receive complete; I2CxRCV register is full  //~~flicks
//        //read the received data
//        data = I2C2RCV;
//
//
//        //Acknowledge the reception of data (NACK stops)
//        //ACKDT: Acknowledge Data bit
//        I2C2CONbits.ACKDT = 1; //1 = NACK
//                               //0 = ACK is sent
//        //Acknowledge Sequence Enable bit
//        I2C2CONbits.ACKEN = 1; //1 = Initiate Acknowledge sequence on SDAx and SCLx pins, and transmit ACKDT data bit; cleared by module
//        while(I2C2CONbits.ACKEN); //Wait for the completion //~~flicks
//        I2C2CONbits.ACKDT = 0;
//
//        //14.Generate a Stop condition on SDAx and SCLx.
//        I2C2CONbits.PEN = 1; //1 = Initiate Stop condition on SDAx and SCLx pins; cleared by module
//        while(I2C2CONbits.PEN); //Wait for it to finish  //~~flicks
//
//        //1001000b (0x48)
//        DelayTime(1000);
//    }

}