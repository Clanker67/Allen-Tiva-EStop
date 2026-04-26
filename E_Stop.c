/////////////////////////////
//ALLEN ZAINA ESTOP TIVA CODE
/////////////////////////////
// System functions prototype include section:
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include <string.h>
// UART & SSI Prototype functions
#include "driverlib/uart.h"
#include "driverlib/ssi.h"

// Interrupts prototype functions
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

// prototype of the uart send function, this function is used to echo back the sent character
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
#define SendDataOnUART(a) UARTSend((uint8_t *)a, strlen(a));

#define FALSE 0
#define TRUE 1
#define LED_OFF FALSE

// Communication Signals
int32_t UART_Received_Char;
uint32_t SSI_ARD_Received_Char;

//---------------ADDED BY ALLEN------------------------//

//Aliases Made by Allen
#define ESTOP_OUTPUT_PORT GPIO_PORTF_BASE
#define ESTOP_LED_PIN GPIO_PIN_1
#define ESTOP_RELAY_PIN GPIO_PIN_2
#define HEARTBEAT GPIO_PIN_3
//#define ESTOP_INPUT_PORT GPIO_PORTA_BASE
//#define ESTOP_INPUT_PIN1 GPIO_PIN_2
//#define ESTOP_INPUT_PIN2 GPIO_PIN_3
#define MECHANICAL_ESTOP_SIMULATED GPIO_PIN_4
#define WIRELESS_ESTOP_SIMULATED GPIO_PIN_0

//Testing using the onboard buttons:
#define ESTOP_INPUT_PORT GPIO_PORTA_BASE
#define ESTOP_INPUT_PIN1 GPIO_PIN_2
#define ESTOP_INPUT_PIN2 GPIO_PIN_3

//Communication Variables from Arduino Code AZ 1/27/2026
char checksum = 0;
double previousTime = 0; // Keep the last time correct data was recieved
unsigned int estopState = 0;
unsigned int estopStatePreivous = 0;
unsigned int estopHW = 0;
unsigned int correctData = 0;
unsigned int controlVehicle = 0; //variable to indicate if the system is controlled by software

//Variables used to keep track of the reception state across interrupts.
//Parser State
volatile uint8_t rx_idx = 0;
volatile uint8_t rx_len_byte0 = 0; // stores length LSB temporarily
volatile uint16_t rx_len = 0;
volatile uint8_t rx_state = 0; // 0=WAIT_START, 1=WAIT_LEN, 2=COLLECT
volatile uint8_t ROSDataIN_Valid = 0;

//Flag for sending the data
volatile uint8_t Data_Flag_Send = FALSE;
/*Andrew Rios Variables: This variable is going to make it so we know what mode we are in
 1 --> ON
 2 --> Self Drive
 */
uint8_t StatesLights = 2; // When the robot is on we want the light to be ON : TODO ADD MORE STATES
//state aliases
#define RX_WAIT_START 0
#define RX_WAIT_LEN 1
#define RX_COLLECT 2

#define START_BYTE 'e' // Change this later to match whatever my start bit is
#define IN_PKT_SIZE (sizeof(sensorData_IN_t))

//Data Structures for communication coming from arduino and microcontroller
typedef struct __attribute__((packed))
{
    char DataStart;
    uint16_t DataLength;
    char estop;
    char light;
    uint16_t seq;
    char checksum;
} sensorData_IN_t;

typedef union
{
    sensorData_IN_t sensor;
    uint8_t ROSPacket[sizeof(sensorData_IN_t)];
} ROS_Packet_IN_t; // This will set RosInputData as the new alias which will allow me to grab parts of sensorDataIn_t

ROS_Packet_IN_t ROSDataIN;

//Data Structures for communication leaving microcontroller
typedef struct __attribute__((packed))
{
    char DataStart; // data type changed A.Z
    int16_t DataLength;
    int8_t info; // data type changed A.Z
    uint16_t seq;
    char checksum; // data type changed A.Z
} sensorData_OUT_t;

typedef union
{
    sensorData_OUT_t sensor;
    uint8_t ROSPacket[sizeof(sensorData_OUT_t)];
} ROS_Packet_OUT_t;

ROS_Packet_OUT_t ROSDataOUT;

void Timer0A_Handler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    static int count = 0;
    const int div = 5;
    count++;
//Added by Andrew Rios March 2
    switch (StatesLights)
    {
    case 2:// E-Stop is pressed, light turns off;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
                break;
    case 3: // ON
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        break;
    case 4: //blinking
    {
        uint8_t v = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1);
        if (count >= div)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, v ^ GPIO_PIN_1);
            count = 0;
        }
    }
    default: // OFF or undefined state
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
        break;
    }

    //Sets Flag to send the data within the while(1) loop
    Data_Flag_Send = TRUE;

}
//Timer
void Timer0A_Init(uint32_t period_us)
{
// 1) Enable the peripheral clock for Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }

//2) Disable Timer A during configuration
    TimerDisable(TIMER0_BASE, TIMER_A);
//3) Configure as 32-bit periodic (down counter by default)
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    uint32_t clock_hz = SysCtlClockGet(); //automatically gets sysclock to calculate required ticks
    uint32_t ticks = (uint32_t) (((uint64_t) clock_hz * period_us) / 1000000ULL);

//5) Load timer
    TimerLoadSet(TIMER0_BASE, TIMER_A, ticks - 1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0A_Handler);
// 6) Clear any pending timeout interrupt and then arm it
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

// Enable interrupts in NVIC
    IntEnable(INT_TIMER0A);

//Global interrupt enable
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
}

static inline void UART0_SendByte(uint8_t b)
{
    UARTCharPut(UART0_BASE, b);
}

void RosUpdateSend(void)
{
    uint8_t PA2_pin;
    uint8_t PA3_pin;
//fill the packet being sent for ros data, this is not actually being sent over ros2, but it is information to be used by ros
    ROSDataOUT.sensor.DataStart = 'f';
    ROSDataOUT.sensor.DataLength = sizeof(ROSDataOUT.ROSPacket);

//read the digital input pins but sets the variable equal to the !NOT of the input (logic high input == 0, etc)
    PA2_pin = (GPIOPinRead(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN1) ? 0 : 1);
    PA3_pin = (GPIOPinRead(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN2) ? 0 : 1);
    ROSDataOUT.sensor.info = (PA2_pin && PA3_pin);

    if (ROSDataOUT.sensor.seq == 65535)
    {
        ROSDataOUT.sensor.seq = 0;
    }
    else
        ROSDataOUT.sensor.seq++;

//For Debugging
    /*
     char msg[64];
     snprintf(msg, sizeof(msg), "PA2=%d PA3=%d INFO=%d SEQ=%u\r\n", PA2_pin,
     PA3_pin, ROSDataOUT.sensor.info, ROSDataOUT.sensor.seq);

     UARTSend((uint8_t*) msg, strlen(msg));
     */

//checksum of the bites to make sure the packet is properly filled
    uint32_t sum = 0;

//local variable only used for this forLoop:
    uint32_t i = 0;

    for (i = 3; i < sizeof(ROSDataOUT.ROSPacket) - 1; i++)
    {
        sum += ROSDataOUT.ROSPacket[i];
    }

    uint8_t chk = (uint8_t) (0xFF - (sum & 0xFF));
    ROSDataOUT.sensor.checksum = chk;

//send all the bytes
    for (i = 0; i < sizeof(ROSDataOUT.ROSPacket); i++)
    {
        UART0_SendByte(ROSDataOUT.ROSPacket[i]);
    }

}

//------------------------------------------------------//

int main(void)
{
// Set the clocking to run directly from the crystal.
    SysCtlClockSet(
    SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

// Used for buttons and LEDs
//Enable Port F, C, and A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

// UART starts here:
// Enable the peripheralS:
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

// Initialize Timer0A to interrupt at 10 Hz using period in microseconds (100000 us  = 0.1s = 10hz)
    Timer0A_Init(100000);

//setting up the output pins following the arduino code
    GPIOPinTypeGPIOOutput(ESTOP_OUTPUT_PORT,
    ESTOP_LED_PIN | ESTOP_RELAY_PIN | HEARTBEAT);

//Setting up the 2 input pins needed (Corresponding to pin 10 and 11 on the arduino)
    GPIOPinTypeGPIOInput(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN1 | ESTOP_INPUT_PIN2);

//Setup pullup resistors for the input ports
    GPIOPadConfigSet(ESTOP_INPUT_PORT,
    ESTOP_INPUT_PIN1 | ESTOP_INPUT_PIN2,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPD);

    ROSDataOUT.sensor.seq = 0;

// Enable processor interrupts.
    IntMasterEnable();

// Set GPIO A0 and A1 as UART pins.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

// Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));

// Enable the UART interrupt.
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    SendDataOnUART("\033cReady...:\n ");

// Loop forever echoing data through the UART.
    while (1)
    {
        if (Data_Flag_Send)
        {
            RosUpdateSend();
            Data_Flag_Send = FALSE;
        }

        if (ROSDataIN_Valid)
        {
            ROSDataIN_Valid = 0; // reset the checksum validator
//Logic translated from the arduino Allen Zaina

//Relay Check
            if (ROSDataIN.sensor.estop != 0)
            {
                GPIOPinWrite(ESTOP_OUTPUT_PORT, ESTOP_RELAY_PIN, 0);
            }
            else
            {
                GPIOPinWrite(ESTOP_OUTPUT_PORT, ESTOP_RELAY_PIN,
                ESTOP_RELAY_PIN);
            }

        }
    }
}

//*****************************************************************************
// The UART interrupt handler.
//*****************************************************************************
void ISR_UARTIntHandler(void)
{
    uint32_t ui32Status;

// Get the interrrupt status
    ui32Status = UARTIntStatus(UART0_BASE, true);

// Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ui32Status);

// Loop while there are characters in the receive FIFO.
    while (UARTCharsAvail(UART0_BASE))
    {
// Read the next character from the UART and write it back to the UART.
//UART_Received_Char = UARTCharGetNonBlocking(UART0_BASE);
//UARTCharPutNonBlocking(UART0_BASE, UART_Received_Char);

//code from john
        uint8_t b = (uint8_t) UARTCharGetNonBlocking(UART0_BASE);

// Optional echo for debugging:
// UARTCharPutNonBlocking(UART0_BASE, b);

// ---- RX state machine skeleton ----
        switch (rx_state)
        {
        case RX_WAIT_START:
            if (b == (uint8_t) START_BYTE)
            {
                rx_idx = 0;
                ROSDataIN.ROSPacket[rx_idx++] = b; // store start byte
                rx_state = RX_WAIT_LEN; //update the state to now be waiting which will put it into the next state
            }
            break;

        case RX_WAIT_LEN:
// We need 2 bytes for DataLength: LSB then MSB (little-endian)
            if (rx_idx == 1)
            {
// first length byte (LSB)
                rx_len_byte0 = b;
                ROSDataIN.ROSPacket[rx_idx++] = b; // store length LSB at index 1
            }
            else
            {
// second length byte (MSB)
                ROSDataIN.ROSPacket[rx_idx++] = b; // store length MSB at index 2

                rx_len = (uint16_t) ((uint16_t) b << 8) | rx_len_byte0; // assemble 16-bit length

// Sanity check: fixed-size packet
                if (rx_len != (uint16_t) IN_PKT_SIZE)
                {
                    rx_state = RX_WAIT_START;
                    rx_idx = 0;
                    break;
                }

                rx_state = RX_COLLECT;
            }
            break;

        case RX_COLLECT:
// prevent overflow
            if (rx_idx >= (uint8_t) IN_PKT_SIZE)
            {
                rx_state = RX_WAIT_START;
                rx_idx = 0;
                break;
            }

            ROSDataIN.ROSPacket[rx_idx++] = b; // store payload/checksum bytes

// done?
            if (rx_idx >= rx_len)
            {
// --- Inline checksum (no helper function) ---
                uint32_t sum = 0;

//local variable only used for this forLoop:
                uint32_t i = 0;

// Sum all bytes except the last one (checksum byte)
                for (i = 3; i < (rx_len - 1); i++)
                {
                    sum += ROSDataIN.ROSPacket[i];
                }

                uint8_t expected = (uint8_t) (0xFF - (sum & 0xFF));
                uint8_t got = ROSDataIN.ROSPacket[rx_len - 1];

                if (expected == got)
                {
                    ROSDataIN_Valid = 1; // packet is valid
                }
                else
                {
// checksum failed -> ignore packet
                }

// Reset parser for next packet
                rx_state = RX_WAIT_START;
                rx_idx = 0;
            }
            break;

        default:
            rx_state = RX_WAIT_START;
            rx_idx = 0;
            break;
        }

    }

}

//*****************************************************************************
// Send a string to the UART.
//*****************************************************************************
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
//
// Loop while there are more characters to send.
//
    while (ui32Count--)
    {
//
// Write the next character to the UART.
//
        UARTCharPut(UART0_BASE, *pui8Buffer++); //switched it to blocking
    }
}

