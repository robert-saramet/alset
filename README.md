## Main architecture
### Outdated, reference only - [current doccumentation here](https://robert-saramet.github.io/alset-v2/)
### 1. Boards
<h4>
     <ul>
         <li> <span class="esp32"> Sparkfun Thing ESP32 </span> (main MCU) </li>
         <li> Pololu A-Star Micro 32u4 (ultrasonic, motors, encoder, slider </li>
         <li> Pololu A-Star Micro 328PB (ultrasonic, GPS, LCD, leds) </li>
         <li> Raspberry Pi 3 or 4 (openCV, Blynk server, dashboard, flashing) </li>
    </ul>
</h4>

### 2. Sensors
- 4x ultrasonic sensors
    - front center
    - front left
    - front right
    - back
- 4x proximity sensors
    - front
    - left
    - right
    - back
- line sensor

### 3. Comms
- Raspberry Pi => HW UART => Thing
- A-Star <=> HW/SW UART <=> Thing
- Thing <=> BT <=> Phone (emulated serial)
- Thing <=> ESP-NOW <=> Joystick

### 4. Peripherals
- Thing:
    - ESC & servo (2x PWM)
    - IMU (I2C)
    - GPS (UART)
    - IR
- A-Star:
    - Ultrasonic
    - Proximity
    - Buzzer
    - LCD (I2C)
    - Encoder
    - Battery check

### 5. Power
- Motors: included NiMH pack
- 2p li-ion
- TP4056 charger
- 3.3v regulator
- TP4056 USB connected to powerbank
- if using Pi 4:
    - must upgrade powerbank
    - need to add solar panel to chassis

### 6. Board
- frequent usb connectors
    - on left side bottom
    - inwards, internally connected
- batteries connected via screw terminals
- 3.3v regulator output on power rail
- all gpio pins broken out (only for pcb)
- connectors for all modules

### 7. Safety
- Batteries
    - ESC has auto poweroff
    - 3v3 regulator has auto poweroff
    - must implement poweroff for Pi
    - perform shutdown functions on low voltage
- Obstacles
    - if any proximity sensor goes on stop immediately
    - if any distance sensor reads less than 10 cm stop immediately
    - add capacitive coating to chassis, connect to touch pin
    - if touch pin goes on stop immediately
    - wait for three seconds after any of the above triggers
    - if any distance sensor reads less than:
        - 20 cm: stop to read sensors
        - 30 cm: steer in clearest direction
    - if all distance sensors read less than 30 cm stop immediately
        - if clear after three seconds proceed normally
        - else try to rotate in 45* steps
- Connection
    - on connect do not start until brake is toggled
    - start with 10% speed
    - on connection lost for more than 100 ms stop
    - if connection lost multiple times in less than two minutes fully stop
    - if more than two consecutive packets are lost stop

### 8. Chassis
- manufacturing
    - 3D printing (most complicated, best long-term)
    - plexiglass/lexan (easiest, most durable, not quite industrial)
    - aluminium ????? (must check car torque first)
    - foam (difficut, tutorial available)
- design
    - small hole for ESC wires
    - holes for ultrasonic sensors
    - cable hooks???
    - panel for connectors
    - holes for cables
    - easy access to everything
    - simple way to detach from wheelbase

### Software
- useful libraries
    - dashboard: ESPUI, ESPDash, **Node-RED**
    - OTA updates: ElegantOTA
    - comms: ESP-NOW, **SerialTransfer**, SerialBT
- ESP32 Thing:
    - setup
        - show welcome screen
        - ask about changing default settings
        - look for button press
        - wait up to three seconds
        - if button pressed
            - ask config method
                - DIP switches
                - encoder
                - RFID
                - web (ESP32 AP)
                - BT (???)
            - save config
        - setup peripherals
    - core 0:
        - read (and send???) data from A-Star
        - read/send RasPi data
        - read data from wifi joystick
        - read/send bluetooth app data
        - check for OTA update connection
        - drive LCD (???)
    - core 1:
        - process data
        - drive ESC & servo
        - read GPS, IMU, IR
- A-Star:
    - read ultrasonic sensors
    - read battery voltage
    - drive status leds and buzzer
    - read touch sensor (or use Thing)
- RasPi:
    - run lane following
    - run sign recognition
    - run node-red
    - communicate with Thing (make this stand-alone?)
