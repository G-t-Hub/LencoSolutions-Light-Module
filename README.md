# LencoSolutions light module info
I started with the original ballance buddy project and added stuff like a brake function and knight rider animation.
The code was also revised to make it non blocking and fast.
Have fun, maybe contribute and check out my site for the hardware: LencoSolutions.nl



# Light module
Arduino based accessory board for the balance app in BV's BLDC firmware.

## Features
1. Directional LEDs for headlight/taillight with dimming, brake lights and knight rider animation
1. Buzzer for dutycycle and low voltage warnings (maybe in the feature also temp. warnings)
1. It uses CAN bus, and can be used in paralel with VESC express and smart BMS devices
1. Displays battery percentage (linear) based on voltage (can be activated with the footsensors also)
1. Build in stepdown converter that is rated up 100 volt batteries

## Parts List
LencoSolutions PCB
1. [Light Module](https://lencosolutions.nl/webshop/light-module/)
1. [Programmer](https://www.amazon.com/Mixse-USBASP-Programming-Device-ATMEL/dp/B075JN58HT)

# Wiring
## Can Module: 
The new version of this hardware has all the pin outs written on the top of the PCB
All the different part are now integrated and also include a port to connect a voltage meter and pull down resisters for the VESC ads's for your footsensors.

# Configuration
## Options and pins
Features are designed to be configured VIA the constants
1. esc.cpp: Configure CAN bus IDs, you must match the ID set in the VESC Tool
1. balance_beeper.cpp: Configure wiring, alerts and expected battery voltages
1. lennart-ballanceleds-0.10.0.ino: Main loop there you set nr of leds and stuff like color


## Compiling/Installing
All the required libraries are included, just hit the upload button in Arduino IDE

# Future plans
1. VESC control over the settings like the color of the lights through can bus
1. A battery indication over a dedicated LED bar