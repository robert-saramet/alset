### Description

Alset is a small-scale proof-of-concept autonomous car which can react to traffic signs and follow lines while also stopping for obstacles. The purpose of this project is to reduce the danger of traffic accidents. Each year, 1.35 million people are killed on roadways around the world. With Alset, we hope to make the world safer for everybody.

### Features
- [x] Stop sign detection (stops for 3 seconds)
- [x] Line following
- [x] Returning to line
- [x] GPIO communications
- [x] Redundant obstacle detection
- [x] Wireless joystick control
- [x] Camera pan & tilt
- [x] Server for picking destination
- [x] Pathfinding (WIP) 

### To-Do
- [ ] Crossroad detection & steering
- [ ] React to other traffic signs
- [ ] Positioning via RFID scanner & tags
- [ ] Better power management
- [ ] Hardware revision
- [ ] Produce PCB
- [ ] Design custom chasis

---

### Traffic Sign Detection
The Haar cascade model is used with opencv, thus traffic signs can easily be implemented with the scripts in the "tools" folder.
The pipeline consists of creating a reasonable number of positive samples (500+ images that contain the sign) and at least half the number of negative images (images that do not contain the sign). The dataset we used can be found [here](https://www.mapillary.com/dataset/trafficsign) and consists of ~40000 images in total, all labeled in JSON files.

Next, a pos.txt file must be created containing all the positive image filenames (this can be achieved via the tools/parse_json.py script). It will be used for creating the .vec file. To do that, you will also need the opencv toolkit. On UNIX systems, the package manager can install everything for you, but on Windows you have to download the [3.4.x version](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/), not the latest one, since future versions no longer contain the cascade training toolkit. After downloading the opencv tools, you are ready to start. To generate the .vec file mentioned earlier, you need to use the ```openv_createsamples``` program.
For example: ```openv_createsamples -info pos.txt -w 24 -h 24 -num 1000 -vec pos.vec```

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
