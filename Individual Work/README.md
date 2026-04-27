# Tiva C Emergency Stop Controller

This repository contains firmware for a **Texas Instruments TM4C123/Tiva C microcontroller** used as an emergency stop controller for an autonomous vehicle/robot system. The code monitors E-stop input signals, sends E-stop status information over UART, receives control packets from the higher-level system, and drives an output relay used to enable or disable vehicle motion.

The project is intended to act as an interface between the robot's safety hardware and the software/ROS-side control system.

---

## Main Features

- Reads two E-stop input signals using GPIO pins.
- Sends E-stop status packets over UART at a fixed timer rate.
- Receives UART packets from the higher-level controller.
- Uses a checksum to verify received packets.
- Controls an E-stop relay output based on received E-stop state.
- Uses Timer0A to create a periodic interrupt for sending data.
- Provides LED behavior for basic system status indication.

---

## Hardware Used

- **Microcontroller:** Texas Instruments TM4C123/Tiva C LaunchPad
- **UART:** UART0 through PA0/PA1
- **GPIO Inputs:** Two E-stop input signals
- **GPIO Outputs:** E-stop LED, relay control pin

---

## Pin Mapping

| Function | Port/Pin | Description |
|---|---:|---|
| UART0 RX | PA0 | Receives UART data |
| UART0 TX | PA1 | Sends UART data |
| E-stop Input 1 | PA2 | Reads one E-stop input signal |
| E-stop Input 2 | PA3 | Reads second E-stop input signal |
| E-stop LED | PF1 | Status LED output |
| E-stop Relay | PF2 | Relay control output |
| Heartbeat | PF3 | Heartbeat/status output |

> Note: The code currently defines PA2 and PA3 as the active E-stop input pins. PF1, PF2, and PF3 are configured as outputs.

---

## UART Configuration

UART0 is configured for:

- **Baud rate:** 115200
- **Data bits:** 8
- **Stop bits:** 1
- **Parity:** None

This is the common `115200 8-N-1` serial configuration.

---

## Timer Behavior

The function `Timer0A_Init(uint32_t period_us)` configures Timer0A using a period given in **microseconds**.

In `main()`, the timer is initialized with:

```c
Timer0A_Init(100000);
```

Since `100000 us = 0.1 s`, the timer interrupt runs at:

```text
1 / 0.1 s = 10 Hz
```

This means the microcontroller sends its output packet approximately **10 times per second**.

Inside the Timer0A interrupt handler:

- The timer interrupt flag is cleared.
- The LED behavior is updated based on `StatesLights`.
- `Data_Flag_Send` is set to `TRUE`.

The actual UART packet is sent from the main `while(1)` loop when `Data_Flag_Send` is detected.

---

## Output Packet Format

The microcontroller sends data using the `sensorData_OUT_t` structure:

```c
typedef struct __attribute__((packed))
{
    char DataStart;
    int16_t DataLength;
    int8_t info;
    uint16_t seq;
    char checksum;
} sensorData_OUT_t;
```

| Field | Description |
|---|---|
| `DataStart` | Start byte for the outgoing packet. Currently set to `'f'`. |
| `DataLength` | Total size of the outgoing packet. |
| `info` | E-stop input status based on PA2 and PA3. |
| `seq` | Sequence counter that increments each packet. |
| `checksum` | Checksum used to verify packet integrity. |

The `info` byte is created from PA2 and PA3:

```c
PA2_pin = (GPIOPinRead(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN1) ? 0 : 1);
PA3_pin = (GPIOPinRead(ESTOP_INPUT_PORT, ESTOP_INPUT_PIN2) ? 0 : 1);
ROSDataOUT.sensor.info = (PA2_pin && PA3_pin);
```

This means the outgoing `info` value is only `1` when both E-stop input conditions are active according to the inverted input logic.

---

## Input Packet Format

The microcontroller receives data using the `sensorData_IN_t` structure:

```c
typedef struct __attribute__((packed))
{
    char DataStart;
    uint16_t DataLength;
    char estop;
    char light;
    uint16_t seq;
    char checksum;
} sensorData_IN_t;
```

| Field | Description |
|---|---|
| `DataStart` | Start byte for incoming packets. Currently expected to be `'e'`. |
| `DataLength` | Total size of the incoming packet. |
| `estop` | E-stop command/state received from the higher-level system. |
| `light` | Light/status command. |
| `seq` | Sequence counter from the sender. |
| `checksum` | Checksum used to verify packet integrity. |

The UART interrupt handler uses a simple state machine to receive packets:

1. Wait for the start byte `'e'`.
2. Read the 2-byte packet length.
3. Collect the rest of the packet.
4. Calculate and verify the checksum.
5. Set `ROSDataIN_Valid = 1` if the packet is valid.

---

## Relay Control Logic

When a valid incoming packet is received, the main loop checks the incoming E-stop field:

```c
if (ROSDataIN.sensor.estop != 0)
{
    GPIOPinWrite(ESTOP_OUTPUT_PORT, ESTOP_RELAY_PIN, 0);
}
else
{
    GPIOPinWrite(ESTOP_OUTPUT_PORT, ESTOP_RELAY_PIN, ESTOP_RELAY_PIN);
}
```

Behavior:

| Incoming `estop` Value | Relay Output |
|---:|---|
| Nonzero | Relay pin set LOW |
| Zero | Relay pin set HIGH |

This allows the higher-level controller to command whether the vehicle relay should be enabled or disabled.

---

## Checksum Method

Both incoming and outgoing packets use the same checksum style.

The code sums packet bytes starting after the start byte and length bytes, then calculates:

```c
checksum = 0xFF - (sum & 0xFF);
```

For incoming packets, the calculated checksum is compared against the received checksum byte. If the values match, the packet is considered valid.

---

## LED State Behavior

The LED behavior is controlled using the `StatesLights` variable.

| `StatesLights` Value | LED Behavior |
|---:|---|
| 2 | LED off, used for E-stop pressed state |
| 3 | LED on |
| 4 | LED blinking |
| Default | LED off |

The blink rate is reduced using a divider inside the timer interrupt so the LED does not blink too quickly to see.

---

## How the Code Runs

At startup, the firmware:

1. Sets the system clock to use the 16 MHz crystal.
2. Enables GPIO ports A, C, and F.
3. Enables UART0.
4. Initializes Timer0A for a 10 Hz interrupt rate.
5. Configures PF1, PF2, and PF3 as outputs.
6. Configures PA2 and PA3 as E-stop input pins.
7. Sets up UART0 on PA0 and PA1.
8. Enables UART receive interrupts.
9. Enters the main loop.

During normal operation:

- Timer0A periodically requests a UART send.
- The main loop sends status packets when the timer flag is set.
- UART receive interrupts parse incoming packets.
- Valid incoming packets update the relay output.

---

## Build Notes

This code is written for the TM4C123/Tiva C using TI DriverLib-style includes such as:

```c
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
```

To build the project, make sure the TivaWare/DriverLib files are correctly linked in your IDE.

Common development environments include:

- Code Composer Studio
- Keil uVision
- Other ARM embedded toolchains configured for the TM4C123

---

## Testing Notes

Useful tests include:

- Open a serial terminal at **115200 baud** and verify that packets are being transmitted.
- Toggle the PA2 and PA3 input signals and confirm that the outgoing `info` byte changes correctly.
- Send a valid incoming packet beginning with start byte `'e'` and confirm that the relay output on PF2 changes state.
- Use an oscilloscope or multimeter to verify the PF2 relay control output.
- Confirm that Timer0A sends data at approximately 10 Hz.

---

## Safety Note

This firmware is part of an emergency stop system, so it should be tested carefully before being used on a moving robot or vehicle. Always verify the relay behavior, input logic, and fail-safe state with the actual hardware before operation.

---

## Author

**Allen Zaina**

Emergency stop controller firmware for a Tiva C based robotic vehicle safety system.
