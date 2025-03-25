# RC Receiver Connection Guide

This guide explains how to connect your RadioLink R12F v1.0 receiver to the Arduino for the tracked robot project.

## RadioLink R12F v1.0 Pinout

Looking at the image of your RadioLink R12F receiver, I can see the pins are labeled from bottom to top on the left side of the receiver:

```
┌─────────────────────────┐
│                         │
│  12: S.BUS              │
│  11: TX                 │
│  10: RX                 │
│  9: 3.12V/DC            │
│  8: CH8                 │
│  7: CH7                 │
│  6: CH6                 │
│  5: CH5                 │
│  4: CH4                 │
│  3: CH3                 │
│  2: CH2                 │
│  1: CH1                 │
│                         │
└─────────────────────────┘
```

### How to Identify Power and Ground Pins

For power and ground connections:
- **Pin 9 (3.12V/DC)**: This is the power pin, located 4th from the top.
- **Ground**: The RadioLink R12F receiver doesn't have a specific pin labeled as "GND" in the image. The ground connection is typically made through the negative (-) terminal of the servo connectors. 

When connecting to standard RC servos or an Arduino, you would use the negative wire of the servo connector (usually black or brown) as the ground connection. Each channel on the receiver has three connection points in the form of a standard servo connector:
1. Signal (usually white or yellow wire)
2. Power (usually red wire)
3. Ground (usually black or brown wire)

For Arduino connection, you would connect the ground wire from one of the channel's servo connectors to the Arduino GND pin.

## Connection to Arduino

For our tracked robot with differential steering, we need two channels:
1. One channel for throttle (forward/backward movement)
2. One channel for steering (left/right turning)

### Standard Connection

Connect the receiver to the Arduino as follows:

1. **Throttle Channel**: Connect pin 2 (CH2) to Arduino pin 2
2. **Steering Channel**: Connect pin 1 (CH1) to Arduino pin 3
3. **Power**: Connect pin 9 (3.12V/DC) to Arduino 5V (the receiver can operate on 3.3-5V)
4. **Ground**: Connect signal ground to Arduino GND

### Wiring Diagram

```
RadioLink R12F                Arduino
┌─────────────┐              ┌─────────────┐
│             │              │             │
│  Pin 2      │──────────────│ Pin 2       │
│  (CH2)      │              │ (Throttle)  │
│             │              │             │
│  Pin 1      │──────────────│ Pin 3       │
│  (CH1)      │              │ (Steering)  │
│             │              │             │
│  Pin 9      │──────────────│ 5V          │
│  (3.12V/DC) │              │             │
│             │              │             │
│  GND        │──────────────│ GND         │
│             │              │             │
│             │              │             │
└─────────────┘              └─────────────┘
```

## Alternative Connection Using SBUS

If you prefer to use SBUS (which allows all channels to be transmitted over a single wire):

1. Connect pin 12 (S.BUS) to Arduino pin 2
2. Connect pin 9 (3.12V/DC) to Arduino 5V
3. Connect signal ground to Arduino GND

Note: Using SBUS requires a different Arduino sketch that can decode the SBUS protocol. The current sketch is set up for standard PWM channel inputs.

## Transmitter Setup

On your transmitter (Jumper T12D):

1. Make sure CH1 is assigned to the steering control (typically right stick left/right)
2. Make sure CH2 is assigned to the throttle control (typically left stick up/down)
3. Set the channel ranges appropriately (usually 1000-2000μs)
4. You may want to adjust the expo and rates to get smoother control

## Testing the Connection

After connecting the receiver to the Arduino:

1. Power on your transmitter first
2. Then power on the receiver and Arduino
3. The receiver should show a solid light indicating it's bound to the transmitter
4. Upload the test sketch to verify the RC signals are being received correctly

```arduino
// Simple RC receiver test sketch
void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT); // Throttle channel
  pinMode(3, INPUT); // Steering channel
}

void loop() {
  int throttle = pulseIn(2, HIGH); // Read throttle pulse width
  int steering = pulseIn(3, HIGH); // Read steering pulse width
  
  Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print(" | Steering: ");
  Serial.println(steering);
  
  delay(100);
}
```

## Troubleshooting

- **No signal received**: Check if the receiver is properly bound to the transmitter
- **Erratic readings**: Make sure there's no interference and the receiver has good power
- **Reversed controls**: Reverse the channel direction in your transmitter settings
- **Signal too weak**: Position the receiver's antenna away from metal parts and power sources
