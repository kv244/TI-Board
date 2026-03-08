//###########################################################################
// FILE:   lidar_servo_scan.c
// TITLE:  LiDAR Servo Scanner — TMS320F28069M LaunchPad
//
// DESCRIPTION:
//   Sweeps an RC servo 0°-180° and back continuously.
//   At each angle step, reads distance from Garmin LIDAR-Lite v3 via I2C.
//   Streams [angle, distance_high, distance_low] packets over SCI-A UART
//   to Arduino UNO R4 WiFi for 2D polar map display on 0.96" OLED.
//
// WIRING:
//   LIDAR-Lite v3:
//     Red    (5V)    → External 5V supply (with 680uF cap to GND)
//     Black  (GND)   → GND
//     Green  (SDA)   → GPIO32 (I2C SDA)
//     Blue   (SCL)   → GPIO33 (I2C SCL)
//     Yellow (MODE)  → GPIO1  (trigger, pulled high via 10K to 3.3V)
//     Orange         → Not connected
//
//   RC Servo:
//     Signal         → GPIO0 (ePWM1A)
//     VCC            → External 5V
//     GND            → GND
//
//   Arduino UNO R4 WiFi:
//     GPIO28 (SCI-A TX) → Arduino Pin 0 (RX)  [direct, no level shifter]
//     GND               → Arduino GND
//
// UART PACKET FORMAT (5 bytes per measurement):
//   [0xFF] [angle_deg] [dist_high] [dist_low] [checksum]
//   angle_deg : 0-180 degrees (1 byte)
//   dist_high : distance MSB in cm (1 byte)
//   dist_low  : distance LSB in cm (1 byte)
//   checksum  : angle_deg XOR dist_high XOR dist_low
//
// LIDAR-Lite v3 I2C:
//   Default address : 0x62
//   Trigger register: 0x00, write 0x04 to start measurement
//   Status register : 0x01, bit 0 = busy (wait until 0)
//   Distance regs   : 0x8F (2 bytes, high then low, auto-increment)
//###########################################################################

#include "DSP28x_Project.h"

//--- Servo Configuration -----------------------------------------------------
#define PWM_PERIOD_TICKS  12500         // 20ms at 625kHz TBCLK = 50Hz
#define SERVO_MIN_TICKS   625           // 1.0ms = 0°
#define SERVO_MID_TICKS   937           // 1.5ms = 90°
#define SERVO_MAX_TICKS   1250          // 2.0ms = 180°
#define SWEEP_STEP_TICKS  1             // Step size per move
#define STEP_DELAY_MS     30            // 30ms per step — time for LiDAR read

//--- LIDAR-Lite v3 I2C -------------------------------------------------------
#define LIDAR_I2C_ADDR    0x62          // Default 7-bit address
#define LIDAR_REG_ACQUIRE 0x00          // Write 0x04 here to trigger
#define LIDAR_REG_STATUS  0x01          // Bit 0 = busy
#define LIDAR_REG_DIST    0x8F          // Read 2 bytes: high, low (cm)
#define LIDAR_ACQUIRE_CMD 0x04          // Trigger measurement, no DC correct
#define LIDAR_TIMEOUT     5000          // Max busy-wait cycles

//--- UART Configuration ------------------------------------------------------
#define LSPCLK_HZ         20000000UL
#define BAUD_RATE         115200UL
#define SYNC_BYTE         0xFF

//--- Function Prototypes -----------------------------------------------------
void    InitEPwm1ForServo(void);
void    InitSciA(void);
void    InitI2C(void);
void    SetServoAngle(Uint16 degrees);
Uint16  LidarReadDistance(void);
void    I2CWriteReg(Uint16 addr, Uint16 reg, Uint16 data);
Uint16  I2CReadReg(Uint16 addr, Uint16 reg, Uint16 numBytes, Uint16 *buf);
void    UartSendPacket(Uint16 angle, Uint16 distance);
void    SciATxChar(Uint16 ch);
void    DelayMs(Uint32 ms);

//=============================================================================
// main()
//=============================================================================
void main(void)
{
    Uint16 angle     = 0;
    int16  direction = 1;       // 1 = forward sweep, -1 = reverse
    Uint16 distance  = 0;

    // Step 1: System init
    InitSysCtrl();

    // Step 2: GPIO setup
    InitEPwm1Gpio();            // GPIO0 = ePWM1A (servo)

    EALLOW;
    // GPIO1 = trigger output for LIDAR (MODE pin, hold HIGH for I2C mode)
    GpioCtrlRegs.GPAPUD.bit.GPIO1  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;    // GPIO mode
    GpioCtrlRegs.GPADIR.bit.GPIO1  = 1;    // Output
    GpioDataRegs.GPASET.bit.GPIO1  = 1;    // Hold HIGH = I2C mode

    // GPIO32 = SDA, GPIO33 = SCL (I2C-A)
    GpioCtrlRegs.GPBPUD.bit.GPIO32  = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;   // I2C SDA
    GpioCtrlRegs.GPBPUD.bit.GPIO33  = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;   // I2C SCL

    // GPIO28 = SCI-A TX, GPIO29 = SCI-A RX
    GpioCtrlRegs.GPAPUD.bit.GPIO28   = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28  = 1;  // SCITXDA
    GpioCtrlRegs.GPAPUD.bit.GPIO29   = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29  = 1;  // SCIRXDA
    EDIS;

    // Step 3: Disable interrupts — polling only for simplicity
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    // Step 4: Init peripherals
    InitEPwm1ForServo();
    InitSciA();
    InitI2C();

    // Step 5: Move servo to start position
    SetServoAngle(0);
    DelayMs(500);

    // Step 6: Main sweep loop
    for(;;)
    {
        // Move servo to current angle
        SetServoAngle(angle);

        // Wait for servo to settle + LiDAR measurement time
        DelayMs(STEP_DELAY_MS);

        // Read distance from LIDAR-Lite v3
        distance = LidarReadDistance();

        // Send [angle, distance] packet to Arduino
        UartSendPacket(angle, distance);

        // Advance angle
        angle += direction;

        // Reverse at limits
        if(angle >= 180)
        {
            angle     = 180;
            direction = -1;
        }
        else if(angle == 0 && direction == -1)
        {
            direction = 1;
        }

        ServiceDog();
    }
}

//=============================================================================
// SetServoAngle()
// Converts 0-180 degrees to CMPA ticks and updates ePWM1
//=============================================================================
void SetServoAngle(Uint16 degrees)
{
    if(degrees > 180) degrees = 180;
    Uint16 ticks = SERVO_MIN_TICKS +
                   ((Uint32)(degrees) * (SERVO_MAX_TICKS - SERVO_MIN_TICKS)) / 180;
    EPwm1Regs.CMPA.half.CMPA = ticks;
}

//=============================================================================
// LidarReadDistance()
// Triggers a measurement on LIDAR-Lite v3 and returns distance in cm
//=============================================================================
Uint16 LidarReadDistance(void)
{
    Uint16 buf[2] = {0, 0};
    Uint16 status = 0;
    Uint32 timeout = 0;

    // Trigger acquisition
    I2CWriteReg(LIDAR_I2C_ADDR, LIDAR_REG_ACQUIRE, LIDAR_ACQUIRE_CMD);

    // Wait until LIDAR is no longer busy (status bit 0 = 0)
    do {
        I2CReadReg(LIDAR_I2C_ADDR, LIDAR_REG_STATUS, 1, &status);
        timeout++;
    } while((status & 0x01) && (timeout < LIDAR_TIMEOUT));

    if(timeout >= LIDAR_TIMEOUT) return 0xFFFF;  // Error value

    // Read 2 bytes of distance (high byte, low byte) in cm
    I2CReadReg(LIDAR_I2C_ADDR, LIDAR_REG_DIST, 2, buf);

    return ((buf[0] << 8) | buf[1]);
}

//=============================================================================
// I2CWriteReg()
// Writes one byte to a register on the I2C device
//=============================================================================
void I2CWriteReg(Uint16 addr, Uint16 reg, Uint16 data)
{
    // Send START + address + write
    I2caRegs.I2CSAR  = addr;
    I2caRegs.I2CCNT  = 2;               // Register byte + data byte
    I2caRegs.I2CDXR  = reg;             // Register address
    I2caRegs.I2CMDR.all = 0x6E20;       // START, STOP, MST, TRX, IRS

    // Wait for TX ready, send register address
    while(I2caRegs.I2CSTR.bit.XRDY == 0) {}
    I2caRegs.I2CDXR = data;

    // Wait for STOP
    while(I2caRegs.I2CSTR.bit.SCD == 0) {}
}

//=============================================================================
// I2CReadReg()
// Reads numBytes from register reg on device at addr into buf[]
//=============================================================================
Uint16 I2CReadReg(Uint16 addr, Uint16 reg, Uint16 numBytes, Uint16 *buf)
{
    Uint16 i;

    // Write register pointer
    I2caRegs.I2CSAR = addr;
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR = reg;
    I2caRegs.I2CMDR.all = 0x6620;       // START, MST, TRX (no STOP)

    while(I2caRegs.I2CSTR.bit.XRDY == 0) {}
    while(I2caRegs.I2CSTR.bit.ARDY == 0) {}

    // Read numBytes
    I2caRegs.I2CCNT = numBytes;
    I2caRegs.I2CMDR.all = 0x6C20;       // START, STOP, MST, receive

    for(i = 0; i < numBytes; i++)
    {
        while(I2caRegs.I2CSTR.bit.RRDY == 0) {}
        buf[i] = I2caRegs.I2CDRR;
    }

    while(I2caRegs.I2CSTR.bit.SCD == 0) {}
    return 0;
}

//=============================================================================
// UartSendPacket()
// Sends 5-byte packet: [0xFF][angle][dist_hi][dist_lo][checksum]
//=============================================================================
void UartSendPacket(Uint16 angle, Uint16 distance)
{
    Uint16 dist_hi  = (distance >> 8) & 0xFF;
    Uint16 dist_lo  = distance & 0xFF;
    Uint16 checksum = angle ^ dist_hi ^ dist_lo;

    SciATxChar(SYNC_BYTE);
    SciATxChar(angle & 0xFF);
    SciATxChar(dist_hi);
    SciATxChar(dist_lo);
    SciATxChar(checksum);
}

//=============================================================================
// SciATxChar() — blocking UART transmit
//=============================================================================
void SciATxChar(Uint16 ch)
{
    while(SciaRegs.SCICTL2.bit.TXRDY == 0) {}
    SciaRegs.SCITXBUF = ch;
}

//=============================================================================
// InitEPwm1ForServo()
//=============================================================================
void InitEPwm1ForServo(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    EPwm1Regs.TBPRD               = PWM_PERIOD_TICKS;
    EPwm1Regs.TBPHS.half.TBPHS    = 0;
    EPwm1Regs.TBCTR               = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UP;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV4;
    EPwm1Regs.TBCTL.bit.PHSEN     = TB_DISABLE;
    EPwm1Regs.CMPA.half.CMPA      = SERVO_MID_TICKS;
    EPwm1Regs.AQCTLA.bit.ZRO      = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAU      = AQ_CLEAR;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

//=============================================================================
// InitSciA() — 115200 baud, 8N1
//=============================================================================
void InitSciA(void)
{
    SciaRegs.SCICCR.all  = 0x0007;
    SciaRegs.SCICTL1.all = 0x0003;
    SciaRegs.SCICTL2.all = 0x0000;
    SciaRegs.SCIHBAUD    = 0x0000;
    SciaRegs.SCILBAUD    = (Uint16)((LSPCLK_HZ / (BAUD_RATE * 8)) - 1);
    SciaRegs.SCICTL1.all = 0x0023;
}

//=============================================================================
// InitI2C() — 400kHz fast mode for LIDAR-Lite v3
//=============================================================================
void InitI2C(void)
{
    // I2C module clock = SYSCLK / (IPSC+1)
    // Target I2C module clock = 10MHz: IPSC = 80MHz/10MHz - 1 = 7
    // For 400kHz: ICCL + ICCH = 10MHz/400kHz = 25 ticks
    // ICCL = 10, ICCH = 15 (asymmetric for better signal integrity)

    I2caRegs.I2CMDR.bit.IRS = 0;       // Reset I2C
    I2caRegs.I2CPSC.all     = 7;       // Prescale: 80MHz/8 = 10MHz
    I2caRegs.I2CCLKL        = 10;      // Low period
    I2caRegs.I2CCLKH        = 15;      // High period → ~400kHz
    I2caRegs.I2CMDR.bit.IRS = 1;       // Release from reset
}

//=============================================================================
// DelayMs()
//=============================================================================
void DelayMs(Uint32 ms)
{
    Uint32 i;
    for(i = 0; i < ms; i++)
        DELAY_US(1000);
}
