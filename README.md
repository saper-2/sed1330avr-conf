# sed1330avr-conf
SED1330 configurer - code for AVR

It was tested with **SED1330** , should work too with: S1D13700 / SED1335 / S1D13705 .

## Hardware config :)
SED interface must be set into 8080 to work with my driver.
For all DATA & control signals I used 10k pullups. Vcc is 5V
```
SED       --- AVR
DATA[7:0] --- PORTC[7:0]
#RST      --- PORTA.0
#CS       --- PORTA.1
#WR       --- PORTA.2
#RD       --- PORTA.3
A0        --- PORTA.4
```
On PORTB.0 is a test led (Hi=ON, Lo=OFF).

MCU: ATMega32 @ 16MHz
UART: 38400,8N1

The PC part (Windows C# Net4.6) is at https://github.com/saper-2/sed1330-configurer

