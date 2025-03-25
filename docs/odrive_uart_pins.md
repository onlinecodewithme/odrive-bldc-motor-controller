# ODrive UART Pin Locations

On the ODrive V3.6 controller, the UART pins are not explicitly labeled as "UART_TX" and "UART_RX" on the board. Instead, they are part of the GPIO header pins.

## UART Pin Locations

For ODrive V3.6 with firmware v0.5.1, the UART pins are:

1. **GPIO1** - This pin functions as **UART_TX** (transmit from ODrive)
2. **GPIO2** - This pin functions as **UART_RX** (receive to ODrive)

These pins are located on the GPIO header of your ODrive board. The GPIO header is typically a row of pins on the edge of the board.

## Visual Reference

```
ODrive V3.6 GPIO Header
┌───────────────────────┐
│                       │
│  ● GND                │
│  ● 3.3V               │
│  ● GPIO1 (UART_TX) ←  │ Connect to Arduino RX (pin 10)
│  ● GPIO2 (UART_RX) ←  │ Connect to Arduino TX (pin 11)
│  ● GPIO3               │
│  ● GPIO4               │
│  ● GPIO5               │
│  ● GPIO6               │
│  ● GPIO7               │
│  ● GPIO8               │
│                       │
└───────────────────────┘
```

## Connection Instructions

1. Connect Arduino pin 10 (RX) to ODrive GPIO1 (UART_TX)
2. Connect Arduino pin 11 (TX) to ODrive GPIO2 (UART_RX)
3. Connect Arduino GND to ODrive GND

## Important Notes

- The UART pins on ODrive V3.6 with firmware v0.5.1 are pre-configured for UART communication at 115200 baud rate
- Make sure to use the correct voltage levels: ODrive GPIO pins operate at 3.3V logic level, while Arduino typically uses 5V logic level. If your Arduino operates at 5V, you might need a logic level converter
- Some Arduino boards (like Arduino Due) operate at 3.3V logic level and can be directly connected to the ODrive

## Alternative Method

If you're having trouble identifying the GPIO pins, you can also use the USB connection for initial testing:

1. Connect the ODrive to your computer via USB
2. Use the `odrivetool` to communicate with the ODrive
3. Once your configuration is working, you can switch to the UART connection for your final setup
