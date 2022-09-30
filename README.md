### Description

#### **Note: please check the [docs](https://github.com/robert-saramet/Alset/tree/alset-v1/docs) folder**.

Alset is a small-scale proof-of-concept autonomous car which can react to traffic signs and follow a track whilst avoiding obstacles. The purpose of this project is to reduce the danger of traffic accidents. Each year, 1.35 million people are killed on roadways around the world. With Alset, we hope to make the world safer for everybody.

![image](https://user-images.githubusercontent.com/44304738/117052703-b5778600-ad20-11eb-9ed1-ab6d96cddfa2.png)

### Features
- [x] Stop sign detection (stops for 3 seconds)
- [x] Line following
- [x] Returning to line
- [x] GPIO communications
- [x] Redundant obstacle detection
- [x] Wireless joystick control
- [ ] Camera pan & tilt
- [x] Server for picking destination
- [x] Pathfinding

### To-Do
- [x] Crossroad detection & steering
- [ ] React to other traffic signs
- [ ] Positioning via RFID scanner & tags
- [x] Better power management
- [x] Hardware revision
- [ ] Produce PCB
- [ ] Design custom chasis

---

### Traffic Sign Detection
The Haar cascade model is used with opencv, thus traffic signs can easily be implemented with the scripts in the "tools" folder.
The pipeline consists of creating a reasonable number of positive samples (500+ images that contain the sign) and at least half the number of negative images (images that do not contain the sign). The dataset we used can be found [here](https://www.mapillary.com/dataset/trafficsign) and consists of ~40000 images in total, all labeled in JSON files.

Next, a pos.txt file must be created containing all the positive image filenames (this can be achieved via the tools/parse_json.py script). It will be used for creating the .vec file. To do that, you will also need the opencv toolkit. On UNIX systems, the package manager can install everything for you, but on Windows you have to download the [3.4.x version](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/), not the latest one, since future versions no longer contain the cascade training toolkit. After downloading the opencv tools, you are ready to start. To generate the .vec file mentioned earlier, you need to use the ```opencv_createsamples``` program.
For example: ```opencv_createsamples -info pos.txt -w 24 -h 24 -num 1000 -vec pos.vec```

With the .vec file you've just created and a neg.txt file containing all the negative images filenames, you can use the ```opencv_traincascade``` program:

```opencv_traincascade -data YourCascadeFolder/ -vec pos.vec -bg neg.txt -w 24 -h 24 -numPos YourNumOfPosImg, -numNeg YourNumOfNegImg```
    
Complete documention on these commands can be found on the [opencv website]( https://docs.opencv.org/3.4/dc/d88/tutorial_traincascade.html).

The final cascade.xml file can be found in ```YourCascadeFolder```, as well as the stages (stage0.xml, stage1.xml, stage2.xml etc), which you won't need at the moment. They are mainly used for downgrading your cascade or for saving progress when the training stops unexpectedly.
Alternatively, you can use the unofficial [GUI version](https://amin-ahmadi.com/cascade-trainer-gui/).

The HAAR cascades are loaded at runtime by the raspberry pi, which uses opencv to recognize the signs captured by the camera. The generated output(position, distance etc) is then processed.

### Lane Following
To get the exact position of the car relative to the line, we decided to use a matrix of 5 sensors that emit IR light and measure it's reflection's intensity. 
It can easily detect crossroads, and can handle small line interruptions.
Independent of the line sensor, 2 ultrasonic sensors detect distance to objects and stop or steer the car, depending on the distance. 

### GPS
At the moment, due to the project's small scale, we use an RFID scanner to simulate a satelite GPS but with much higher accuracy. The intersection grid is mapped using a matrix of RFID stickers. Each sticker is assigned to an intersection, containing information about it's position and speed limit. The user can select a destination using the website hosted on the server. The server then sends the data to the raspberry pi; when it receives the data, it is parsed and a command queue made of broad directions is generated, which the microcontroller can follow.

### Server
The server is currently running on node.js express module.
The security protocol is currently based on a 8 character unique device-specific code that comes with the car from the factory. When the user logs in on the website, they just enter the code, which is checked against the server's database. If the code matches any car, the instructions are sent to it. However, before trying to log in, the car must be connected to the server, or else the code will not be found.
In the future, we will use MySQL for the database, hash the codes and also design a security protocol against bruteforce attacks (no more than 5 login requests can be sent from the same IP address in 10 minute).

---

### Third Party

- #### [Mapillary Traffic Sign Dataset](https://www.mapillary.com/dataset/trafficsign)
- #### [OpenCV](https://opencv.org/)
- #### [PD Controller](https://tutorial.cytron.io/2019/08/21/esp32-pd-line-following-robot-with-maker-line/)
- #### [Cascade Trainer GUI](https://amin-ahmadi.com/cascade-trainer-gui/)
- #### [PySerial Library](https://github.com/pyserial/pyserial)

### Parts

- #### Robot
  - [Raspberry Pi 3 B](https://cleste.ro/raspberry-pi-3-model-b.html)
  - [Arduino Mega 2560](https://cleste.ro/placa-de-dezvoltare-compatibila-cu-arduino-mega-2560.html)
  - [Wemos D1 Mini ESP8266](https://cleste.ro/placa-dezvoltare-esp12-mini-v2.html)
  - [Chasis & Motors](https://www.optimusdigital.ro/ro/robotica-kit-uri-de-roboti/11883-kit-robot-cu-4-motoare-negru.html)
  - [L298N Motor Driver](https://cleste.ro/modul-l298n-cu-punte-h-dubla.html)
  - [U-blox NEO-6M GPS](https://cleste.ro/modul-gps-ublox-neo-6m-cu-antena.html)
  - [RC522 RFID Reader](https://cleste.ro/modul-rfid-cu-card-si-tag.html)
  - [Cytron Maker Line Sensor](https://www.optimusdigital.ro/ro/altele/12072-senzori-de-linie-maker-line-pentru-incepatori.html?search_query=maker+line&results=2)
  - [2x HC-SR04 Ultrasound Sensors](https://cleste.ro/senzor-ultrasonic-hc-sr04.html)
  - [Collision Switch](https://cleste.ro/modul-impact.html)
  - [TP4056 Battery Charger](https://cleste.ro/modul-incarcare-baterii-litiu-1a-usb-c-tp4056.html)
  - [2x Panel Switch](https://cleste.ro/buton-panou.html)
  - [18650 Battery Holder](https://www.emag.ro/suport-acumulator-3-7v-18650-x-4-sloturi-s18650-4/pd/D8C49WBBM/)
- #### Joystick
  - [Lolin32 ESP32](https://cleste.ro/placa-dezvoltare-nodemcu-wifi-bluetooth-esp32.html)
  - [2x PS2 Joystick](https://cleste.ro/modul-joystick-ps2-compatibil-arduino.html)
  - [Passive Buzzer](https://cleste.ro/modul-buzzer-pasiv.html)
  - [SSD 1306 OLED](https://cleste.ro/ecran-oled-0-91.html)

---

### Roles
- #### Robert
  -  Line following
  - Joystick
  - Obstacle detection
  - Hardware & electronics
- #### Bogdan
  - Machine learning
  - Image recognition
  - Server
  - Webpage
