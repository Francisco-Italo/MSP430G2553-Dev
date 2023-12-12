/*
 * Code: Read raw accelerometer X, Y and Z data continuously
 *
 *  P1.6          UCB0SCL
 *  P1.7          UCB0SDA
 *
 * MPU-6050 Accelerometer & Gyro
 *
 * Slave address:   0x68 (AD0=0) or 0x69 (AD0=1)
 *
 * Z-data buffer addresses:
 *          0x3B ACCEL_XOUT_H R ACCEL_XOUT[15:8]
 *          0x3C ACCEL_XOUT_L R ACCEL_XOUT[ 7:0]
 *          0x3D ACCEL_YOUT_H R ACCEL_YOUT[15:8]
 *          0x3E ACCEL_YOUT_L R ACCEL_YOUT[ 7:0]
 *          0x3F ACCEL_ZOUT_H R ACCEL_ZOUT[15:8]
 *          0x40 ACCEL_ZOUT_L R ACCEL_ZOUT[ 7:0]
 *
 * pins not used:   INT (interrupt for data ready in the 1024-byte FIFO bufer)
 *            XCL, XDA (external clock and data lines for MPU-6050 I2C bus)
 *
 * Reading the raw values: disable sleep mode
 *          0x6B PWR_MGMT_1 --> set to 0
 *
 */
#include <msp430.h>

/*
 * main.c
 */

unsigned char RX_Data[6];
unsigned char TX_Data[2];
unsigned char RX_ByteCtr;
unsigned char TX_ByteCtr;

int xAccel;
int yAccel;
int zAccel;

unsigned char slaveAddress = 0x68;  // Set slave address for MPU-6050
          // 0x68 for ADD pin=0
          // 0x69 for ADD pin=1

const unsigned char PWR_MGMT_1   = 0x6B;  // MPU-6050 register address
const unsigned char ACCEL_XOUT_H = 0x3B;  // MPU-6050 register address
const unsigned char ACCEL_XOUT_L = 0x3C;  // MPU-6050 register address
const unsigned char ACCEL_YOUT_H = 0x3D;  // MPU-6050 register address
const unsigned char ACCEL_YOUT_L = 0x3E;  // MPU-6050 register address
const unsigned char ACCEL_ZOUT_H = 0x3F;  // MPU-6050 register address
const unsigned char ACCEL_ZOUT_L = 0x40;  // MPU-6050 register address

void i2cInit(void);
void i2cWrite(unsigned char);
void i2cRead(unsigned char);
void initUart(void)
{
    UCA0CTL1 |= UCSSEL_2; // Use SMCLK
    UCA0BR0 = 104; // 1MHz 9600
    UCA0BR1 = 0; // 1MHz 9600
    UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1
    P1SEL = BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
    UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
    IE2 |= UCA0TXIE;
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;         // Stop WDT

  // Set clock speed (default = 1 MHz)
  BCSCTL1 = CALBC1_1MHZ;          // Basic Clock System CTL (1,8,12 16_MHZ available)
  DCOCTL  = CALDCO_1MHZ;          // Digitally-Controlled Oscillator CTL

  // set up I2C pins
  P1SEL |= BIT6 + BIT7;         // Assign I2C pins to USCI_B0
  P1SEL2|= BIT6 + BIT7;         // Assign I2C pins to USCI_B0

  // Initialize the I2C state machine
  i2cInit();

  // Wake up the MPU-6050
  slaveAddress = 0x68;          // MPU-6050 address
  TX_Data[1] = 0x6B;            // address of PWR_MGMT_1 register
  TX_Data[0] = 0x00;            // set register to zero (wakes up the MPU-6050)
  TX_ByteCtr = 2;
  i2cWrite(slaveAddress);

  while (1)
  {
    // Point to the ACCEL_ZOUT_H register in the MPU-6050
    slaveAddress = 0x68;          // MPU-6050 address
    TX_Data[0] = 0x3B;          // register address
    TX_ByteCtr = 1;
    i2cWrite(slaveAddress);

    // Read the two bytes of data and store them in zAccel
    slaveAddress = 0x68;          // MPU-6050 address
    RX_ByteCtr = 6;
    i2cRead(slaveAddress);
    xAccel  = RX_Data[5] << 8;        // MSB
    xAccel |= RX_Data[4];         // LSB
    yAccel  = RX_Data[3] << 8;        // MSB
    yAccel |= RX_Data[2];         // LSB
    zAccel  = RX_Data[1] << 8;        // MSB
    zAccel |= RX_Data[0];         // LSB

    // do something with the data

    __no_operation();                       // Set breakpoint >>here<< and read

  }
}

//*********************************************************************************************
void i2cInit(void)
{
  // set up I2C module
  UCB0CTL1 |= UCSWRST;        // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;         // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;      // Use SMCLK, keep SW reset
  UCB0BR0 = 10;         // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0CTL1 &= ~UCSWRST;       // Clear SW reset, resume operation
  IE2 |= UCB0RXIE + UCB0TXIE;
}

//*********************************************************************************************
void i2cWrite(unsigned char address)
{
  __disable_interrupt();
  UCB0I2CSA = address;        // Load slave address
  IE2 |= UCB0TXIE;        // Enable TX interrupt
  while(UCB0CTL1 & UCTXSTP);      // Ensure stop condition sent
  UCB0CTL1 |= UCTR + UCTXSTT;     // TX mode and START condition
  __bis_SR_register(CPUOFF + GIE);    // sleep until UCB0TXIFG is set ...
}

//*********************************************************************************************
void i2cRead(unsigned char address)
{
  __disable_interrupt();
  UCB0I2CSA = address;        // Load slave address
  IE2 |= UCB0RXIE;        // Enable RX interrupt
  while(UCB0CTL1 & UCTXSTP);      // Ensure stop condition sent
  UCB0CTL1 &= ~UCTR;        // RX mode
  UCB0CTL1 |= UCTXSTT;        // Start Condition
  __bis_SR_register(CPUOFF + GIE);    // sleep until UCB0RXIFG is set ...
}

/**********************************************************************************************/
// USCIAB0TX_ISR
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  if(UCB0CTL1 & UCTR)         // TX mode (UCTR == 1)
  {
    if (TX_ByteCtr)               // TRUE if more bytes remain
    {
      TX_ByteCtr--;       // Decrement TX byte counter
      UCB0TXBUF = TX_Data[TX_ByteCtr];  // Load TX buffer
    }
    else            // no more bytes to send
    {
      UCB0CTL1 |= UCTXSTP;      // I2C stop condition
      IFG2 &= ~UCB0TXIFG;     // Clear USCI_B0 TX int flag
      __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
    }
  }
  else // (UCTR == 0)         // RX mode
  {
    RX_ByteCtr--;               // Decrement RX byte counter
    if (RX_ByteCtr)               // RxByteCtr != 0
    {
      RX_Data[RX_ByteCtr] = UCB0RXBUF;  // Get received byte
      if (RX_ByteCtr == 1)      // Only one byte left?
      UCB0CTL1 |= UCTXSTP;      // Generate I2C stop condition
    }
    else            // RxByteCtr == 0
    {
      RX_Data[RX_ByteCtr] = UCB0RXBUF;  // Get final received byte
      __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
    }
  }
}
