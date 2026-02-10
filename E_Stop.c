/////////////////////////////
//ALLEN ZAINA ESTOP TIVA CODE
/////////////////////////////
//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2
//! - SSI0Fss - PA3
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of SSI0.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1

//! Connections, for example, we can connect the TIVA C with an Arduino Micro, or any micro supporting SSI
//! Arduino
//! 10 CS
//! 12 MISO
//! 11 MOSI
//! 13 CLK

//! TIVA:
//! SSI0Clk - PA2
//! SSI0Fss - PA3
//! SSI0Rx  - PA4
//! SSI0Tx  - PA5

// System functions prototype include section:
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
// UART & SSI Prototype functions
#include "driverlib/uart.h"
#include "driverlib/ssi.h"

// Interrupts prototype functions
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

// prototype of the uart send function, this function is used to echo back the sent character
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
#define SendDataOnUART(a)  UARTSend((uint8_t *)a, strlen(a));

#define FALSE 0
#define TRUE  1
#define LED_OFF           FALSE

// Communication Signals
int32_t UART_Received_Char;
uint32_t SSI_ARD_Received_Char;

//---------------ADDED BY ALLEN------------------------//

//Globals variables for the timer
#define NUMT 3
volatile uint8_t TFlag[NUMT];
volatile uint32_t TPrev[NUMT];
volatile uint32_t TPeriod[NUMT];
volatile uint32_t g_ms = 0;

//Aliases Made by Allen
#define ESTOP_OUTPUT_PORT   GPIO_PORTF_BASE
#define ESTOP_LED_PIN      GPIO_PIN_1
#define ESTOP_RELAY_PIN    GPIO_PIN_2
#define HEARTBEAT          GPIO_PIN_3
//#define ESTOP_INPUT_PORT    GPIO_PORTC_BASE
//#define ESTOP_INPUT_PIN1    GPIO_PIN_4
//#define ESTOP_INPUT_PIN2    GPIO_PIN_5
#define MECHANICAL_ESTOP_SIMULATED   GPIO_PIN_4
#define WIRELESS_ESTOP_SIMULATED     GPIO_PIN_0

//Testing using the onboard buttons:
#define ESTOP_INPUT_PORT    GPIO_PORTA_BASE
#define ESTOP_INPUT_PIN1    GPIO_PIN_2
#define ESTOP_INPUT_PIN2    GPIO_PIN_3

//variable for for loops
//uint32_t i = 0;

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
volatile uint8_t rx_len = 0;
volatile uint8_t rx_state = 0;     // 0=WAIT_START, 1=WAIT_LEN, 2=COLLECT
volatile uint8_t ROSDataIN_Valid = 0;

//state aliases
#define RX_WAIT_START  0
#define RX_WAIT_LEN    1
#define RX_COLLECT     2

#define START_BYTE     'f'          // Change this later to match whatever my start bit is
#define IN_PKT_SIZE    (sizeof(sensorData_IN_t))

//Data Structures for communication coming from arduino and microcontroller
typedef struct __attribute__((packed))
{
    char DataStart;
    uint8_t DataLength;
    char estop;
    char light;
    uint16_t seq;
    uint8_t checksum;
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
    unsigned int DataLength;
    uint8_t info; // data type changed A.Z
    unsigned int seq;
    uint8_t checksum; // data type changed A.Z
} sensorData_OUT_t;

typedef union
{
    sensorData_OUT_t sensor;
    uint8_t ROSPacket[sizeof(sensorData_OUT_t)];
} ROS_Packet_OUT_t;

ROS_Packet_OUT_t ROSDataOUT;

/*
//Function to increment the timer variable g_ms
void SysTick_Handler(void)
{
    g_ms++;
}

//Functions to update the timers
//This function takes the ms that was being counted by the systick_handler and returns it anywhere that the function millis is called.
static inline uint32_t millis(void)
{
    return g_ms;
}

static inline void UART0_SendByte(uint8_t);

void TimersInit(void)
{
    uint32_t now = millis();
    //local variable only used for this forLoop:
     uint32_t i = 0;
    for (i = 0; i < NUMT; i++)
    {
        TPrev[i] = now;
        TFlag[i] = false;
    }

}


void TimersUpdate(void)
{
    uint32_t now = millis();
    //local variable only used for this forLoop:
     uint32_t i = 0;
    for (i = 0; i < NUMT; i++)
    {
        if ((now - TPrev[i]) >= TPeriod[i])
        {
            TPrev[i] = now;
            TFlag[i] = true;
        }
    }
}
*/

void Timer0A_Handler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //RosUpdateSend();
    uint8_t v = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, v ^ GPIO_PIN_1);

}

//Timer
void Timer0A_Init(uint32_t period_us)
{
    // 1) Enable the peripheral clock for Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    //2) Disable Timer A during configuration
    TimerDisable(TIMER0_BASE, TIMER_A);
    //3) Configure as 32-bit periodic (down counter by default)
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    uint32_t clock_hz = SysCtlClockGet(); //automatically gets sysclock to calculate required ticks
    uint32_t ticks = (uint32_t)(((uint64_t)clock_hz * period_us) / 1000000ULL);
    //5) Load timer
    TimerLoadSet(TIMER0_BASE, TIMER_A,ticks -1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0A_Handler);
     // 6) Clear any pending timeout interrupt and then arm it
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // 7) NVIC: set priority and enable the interrupt
    // "2" here matches your intent; exact numeric meaning depends on priority bits.
    //IntPrioritySet(INT_TIMER0A, 2 << 5);

    // Enable interrupts in NVIC
    IntEnable(INT_TIMER0A);

    //Global interrupt enable
    IntMasterEnable();

    TimerEnable(TIMER0_BASE,TIMER_A);
}


static inline void UART0_SendByte(uint8_t b)
{
    UARTCharPut(UART0_BASE, b);
}

void RosUpdateSend(void)
{
    uint8_t PC4_pin;
    uint8_t PC5_pin;
    //fill the packet being sent for ros data, this is not actually being sent over ros2, but it is information to be used by ros
    ROSDataOUT.sensor.DataStart = 'f';
    ROSDataOUT.sensor.DataLength = sizeof(ROSDataOUT.ROSPacket);

    //read the digital input pins
    PC4_pin = (GPIOPinRead(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN1) ? 1 : 0);
    PC5_pin = (GPIOPinRead(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN2) ? 1 : 0);
    ROSDataOUT.sensor.info = (PC4_pin && PC5_pin);

    ROSDataOUT.sensor.seq++;

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

    /*
    //Setting Up my SysTick
    SysTickPeriodSet(SysCtlClockGet() / 1000); //1ms
    SysTickIntEnable();
    SysTickEnable();
    */

    //Setting Periods
    TPeriod[0] = 20; //50hz
    TPeriod[1] = 100; // 10hz
    TPeriod[2] = 1000; // 1hz

    //initialize the timer to work at 10hz
    Timer0A_Init(100000);

    //setting up the output pins following the arduino code
    GPIOPinTypeGPIOOutput(ESTOP_OUTPUT_PORT,
    ESTOP_LED_PIN | ESTOP_RELAY_PIN | HEARTBEAT);

    //GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, MECHANICAL_ESTOP_SIMULATED | WIRELESS_ESTOP_SIMULATED);

    //writing the LED output for pin 1 which is the LED on top of the car
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

    //Writing the Estop Output for pin 2 which controls the relay
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    //Writing the heartbeat output
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

    //Setting up the 2 input pins needed (Corresponding to pin 10 and 11 on the arduino)
    GPIOPinTypeGPIOInput(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN1 | ESTOP_INPUT_PIN2);

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
        /*
        //setting up the timers to only send data out at 10 hz (every 100ms)
        TimersUpdate();

        if (TFlag[1]){
            TFlag[1] = 0; //sets the flag back to false
            RosUpdateSend();
        }
        */

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
            //LED Check?
            if (ROSDataIN.sensor.light != 0)
            {
                GPIOPinWrite(ESTOP_OUTPUT_PORT, ESTOP_LED_PIN, 0);
            }
            else
            {
                GPIOPinWrite(ESTOP_OUTPUT_PORT, ESTOP_LED_PIN, ESTOP_LED_PIN);
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
                ROSDataIN.ROSPacket[rx_idx++] = b;  // store start byte
                rx_state = RX_WAIT_LEN; //update the state to now be waiting which will put it into the next state
            }
            break;

        case RX_WAIT_LEN:
            rx_len = b;

            // Sanity check (simple version: fixed length)
            if (rx_len != (uint8_t) IN_PKT_SIZE)
            {
                // bad length -> reset and resync
                rx_state = RX_WAIT_START;
                rx_idx = 0;
                break;
            }

            ROSDataIN.ROSPacket[rx_idx++] = b;  // store length byte
            rx_state = RX_COLLECT;
            break;

        case RX_COLLECT:
            // prevent overflow
            if (rx_idx >= (uint8_t) IN_PKT_SIZE)
            {
                rx_state = RX_WAIT_START;
                rx_idx = 0;
                break;
            }

            ROSDataIN.ROSPacket[rx_idx++] = b;  // store payload/checksum bytes

            // done?
            if (rx_idx >= rx_len)
            {
                // --- Inline checksum (no helper function) ---
                    uint32_t sum = 0;

                    //local variable only used for this forLoop:
                    uint32_t i = 0;

                    // Sum all bytes except the last one (checksum byte)
                    for (i = 0; i < (rx_len - 1); i++)
                    {
                        sum += ROSDataIN.ROSPacket[i];
                    }

                    uint8_t expected = (uint8_t)(0xFF - (sum & 0xFF));
                    uint8_t got      = ROSDataIN.ROSPacket[rx_len - 1];

                    if (expected == got)
                    {
                        ROSDataIN_Valid = 1;   // packet is valid
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
        UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

