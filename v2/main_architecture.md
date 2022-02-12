## Main architecture

---
#### *Please note that some parts of this page might be outdated or incomplete.*
---

### 1. Boards
<h4>
     <ul>
         <li> <span class="esp32"> Sparkfun ESP32 Thing </span> (main MCU) </li>
         <li> Pololu A-Star Micro 32u4 (motors, ultrasonic & speed sensors) </li>
         <li> Pololu A-Star Micro 328PB (GPS, LCD, ultrasonic sensors) </li>
         <li> Raspberry Pi 3B (openCV, Blynk server, GPS webapp) </li>
    </ul>
</h4>

### 2. Sensors
- 4x ultrasonic sensors
    - front center
    - front left
    - front right
    - left center
    - right center
    - back center

### 3. Comms
- Raspberry Pi <=> HW UART <=> ESP32
- 328PB <=> I2C <=> 32u4 <=> HW UART <=> ESP32
- ESP32 <=> BT/WiFi <=> Phone (emulated serial)
- ESP32 <=> BT <=> PS4 Controller
- ESP32 <=> ESP-NOW <=> Joystick (To-Do)

### 4. Peripherals
- ESP32:
    - IMU
    - All comms
- A-Star:
    - ESC & servo
    - Ultrasonic
    - Encoder
- 328PB:
    - GPS
    - LCD (I2C)
    - Buzzer & LEDs
    - Voltage sensor

### 5. Power
- Motors: included NiMH pack
- 2p li-ion
- TP4056 charger
- 5v regulator
- TP4056 USB connected to powerbank

### 6. Circuit board
- USB connectors on left side bottom
- Batteries connected via screw terminals
- 5V regulator output on power rail
- All gpio pins broken out (only for pcb)
- Connectors for all modules

### 7. Safety
- Batteries
    - ESC has auto poweroff
    - 5V regulator has auto poweroff
    - Must implement poweroff for Pi
    - Perform shutdown functions on low voltage
- Obstacles
    - if any distance changes by at least 2 cm since last read
        - if distF >= 60 and (distFL and distFR >= 60) and (distL & distR >= 40)
            - go straight forward
         - else if distF < 60 and (distFL or distFR < 60) or (distL or distR < 40)
             - if distFL > distFR and distL > distR
                 - if going forward steer left
                 - if going in reverse steer right
             - else if distFL < distFR and distL < distR
                 - if going forward steer right
                 - if going in reverse steer left
             - else
                 - ```distLFL = 5 * distL + 4 * distFL```
                 - ```distRFR = 5 * distR + 4 * distFR```
                 - if distLFL > distRFR
                     - if going forward steer left
                     - if going in reverse steer right
                 - else if distLFL < distRFR
                     - if going forward steer left
                     - if going in reverse steer right
         - else if (distFL or distFR <= 40) and (distL or distR <= 25)
             - same as above
         - else if (distFL or distFR <= 10) or (distL or distR <= 5)
             - brake and wait for 200 ms
             - read all sensors again
             - if any value is not the same
                 - wait 100 ms
                 - read all sensors again
                 - calculate average of each distance
                 - if mdoe is full auto go straight in reverse
    - else keep previous direction 

    - add capacitive coating to chassis, connect to touch pin (To-Do)
    - if touch pin goes on stop immediately
    - wait for three seconds after detecting touch
    
    - if all distance sensors read less than 30 cm stop immediately (To-Do)
        - if clear after three seconds proceed normally
        - else try to rotate in 60* steps
- Connection
    - on connection lost for more than 100 ms brake
    - on any powerloss fully stop
    - on connect do not start until brake is toggled (To-Do)
    - if connection lost multiple times in less than two minutes fully stop (To-Do)
    - if more than two consecutive packets are lost stop (To-Do)

### 8. Chassis
- design
    - simple way to detach from wheelbase
    - easy access to everything
    - modularity
    - cable hooks??? (To-Do)
    - holes for cables (To-Do)
