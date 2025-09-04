# Bluetooth HID Media Controller with PSoC 6

Dit project demonstreert hoe een **PSoC 6 Prototyping Kit (CY8CPROTO-062-4343W)** kan worden gebruikt als een **Bluetooth Human Interface Device (HID)**.  
De ingebouwde **CapSense®-knoppen en slider** fungeren hierbij als invoerapparaat om mediabediening op een laptop of smartphone draadloos aan te sturen.  

De oorspronkelijke CapSense GATT-service werd aangepast naar een **HID-service**, waarbij de CapSense-inputs gekoppeld zijn aan HID-commando’s zoals play/pause, mute en volumeregeling.

## Functionaliteit
- **Button 0** → Mute toggle  
- **Button 1** → Play/Pause  
- **Slider** → Volume omhoog/omlaag  

## Hardware
- **Board**: [PSoC 6 Prototyping Kit CY8CPROTO-062-4343W](https://www.infineon.com/CY8CPROTO-062-4343W)  
- **Programmer/Debugger**: KitProg3 (via USB)  
- **Doelapparaat**: Laptop of smartphone met Bluetooth Low Energy (BLE)  

## Software
- [ModusToolbox™ 3.1](https://www.infineon.com/modustoolbox)  
- ModusToolbox configurators:  
  - **Bluetooth Configurator** voor de HID-service  
  - **CapSense Configurator** voor de knoppen en slider  
- **Middleware**: CapSense, Bluetooth LE stack, HID libraries  

## Installatie
1. Clone deze repository:  
   ```bash
   git clone https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6

2. De belangrijkste bestanden van dit project zijn de volgende:

   main.c:
   Entry point van de applicatie. Hier worden de FreeRTOS-taken aangemaakt en de initialisatie van Bluetooth, CapSense en boardcomponenten uitgevoerd.

   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/source/main.c

   capsense.c:
   Bevat de functies voor het initialiseren, uitlezen en verwerken van CapSense-inputs (knoppen en slider).

   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/source/capsense.c

   capsense.h:
   Headerbestand met de declaraties van CapSense-functies en structuren.

   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/source/capsense.h

   board.c:
   Bevat de functies voor board-specifieke logica, zoals LED-aansturing of andere hardware die door

   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/source/board/board.c

   board.h:
   Headerbestand met de definities en functies voor board-hardware.

   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/source/board/board.h

   bt_app.c:
   Implementeert de Bluetooth Low Energy stack events en HID-functionaliteit. Verzorgt de koppeling tussen CapSense-inputs en HID-rapporten.

   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/source/bt/bt_app.c

   bt_app.h:
   Headerbestand met definities en prototypes van de Bluetooth-functies.

   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/source/bt/bt_app.h

   Bluetooth configuratie:
   Configuratiebestand van de ModusToolbox Bluetooth Configurator. Hierin wordt de HID-service en de report map gedefinieerd.
   
   https://github.com/LoicSchillings/Bluetooth-Human-Interface-Device-with-PSOC6/blob/master/design.cybt
