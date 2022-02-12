
![image](https://raw.githubusercontent.com/robert-saramet/alset-v2/db09ef69e4e6436f0b647ece5fb87bbe2354e2f0/docs/images/1.jpg)

### Rezumat
*Alset este o masina autonoma proof-of-concept la scara mica care poate reactiona la semne de circulatie, naviga prin GPS, urmari strazi/benzi si ocoli obstacole. In plus, Alset poate fi utilizat ca un kit complet universal pentru a face orice masina cu radiocomanda inteligenta. Este modular si ofera multa siguranta.*

### Functii
- [x] Identificarea semnelor de circulatie
- [x] Urmarirea strazii/benzii
- [x] Webapp pentru navigare GPS
- [x] Aplicatie pentru telefon
- [x] Operare prin controller de PS4
- [x] Live stream FPV de mare viteza
- [x] Functii de siguranta redundante
- [x] Evitarea obstacolelor
- [x] Modularitate
- [x] Debugging usor
- [x] Versatilitate

### To-Do
- [x] Implementarea mai multor semne
- [x] Gestionare mai buna a puterii
- [x] Revizie hardware
- [ ] Detectarea intersectiilor
- [ ] Slider de viteza (in lucru)
- [ ] Viteza constanta automata (in lucru)
- [ ] Directii prin TTS pentru GPS

---

### Controller
Alset poate fi condus cu ajutorul unui controller de PS4 conectat prin bluetooth. In acest scop, adresa MAC a consolei aferente controller-ului trebuie obtinuta cu ajutorul [acestei unelte](https://github.com/user-none/sixaxispairer) si atribuit pe ESP32. In urma conectarii controller-ului, robotul porneste in modul de siguranta, unde acceleratia si directia de la controller sunt ignorate. La intrarea in modul de siguranta, Alset opreste motoarele imediat. Pentru a incepe sa conduceti, apasati butonul patrat. Acesta comuta modul de siguranta, functionand drept kill switch. Pentru a inainta apasati trigger-ul drept. Pentru a frana, apasati atat trigger-ul drept cat si cel stang. Pentru a merge cu spatele, apasati trigger-ul stang. VIteza este proportionala cu forta aplicata pe trigger. Un mod turbo este disponibil prin apasarea butonului triunghi, crescand viteza minima si maxima. In plus, prin apasarea butonului cruce/x, utilizatorul poate schimba intre modul asistat si modul complet manual. In modul manual, utilizatorul controleaza atat acceleratia la motor cat si directia la servo. In modul asistat, utilizatorul ramane in controlul acceleratiei (pentru siguranta), dar directia este controlata de algoritmul de pathfinding cu senzori ultrasonici. Indiferent de mod, feedback cu privire la distanta pana la obstacole este oferit utilizatorului in doua moduri: in primul rand, led-urile RGB de pe controller isi schimba gradat culoarea de la verde la rosu in functie de distanta pana la cel mai apropiat obnstacol; in al doilea rand, motoarele de vibratie duale ale controller-ului vibreaza proportional cu distanta pana la cel mai apropiat obiect din fiecare directie (fata-stanga si fata-dreapta).

### Mecanica & modularitate
Dupa cum e posibil sa stiti, Alset v1 utiliza virare diferentiala si era la o scara mult mai mica. Insa, scopul acestui proiect fiind de proof-of-concept, am decis sa facem Alset v2 cat mai similar cu o masina reala. Folosind un singur motor si viraje tip Ackermann, in acelasi timp fiind mult mai mare (scara 1/10), Alset v2 atinge acest scop cu usurinta.

O alta imbunatatire fata de v1 este modularitatea inalta, permitand adaugarea, indepartarea si inlocuirea simpla a componentelor si oferind acces usor la tot. Baza robotului este atasata la corpul masiniii folosind sistemul standard de clip-uri prezent pe toate masinile RC, oferind montare si demontare rapida si *universala*. Totul restul este conectat cu suruburi, bateriile la terminale cu surub, iar distrbutia greutatii este optimizata pentru experienta de conducere perfecta.

In plus, *tot* ce este necesar pentru utilizarea lui Alset cu *orice* masina RC electrica este conexiunea a *doar doua cabluri* de la receptorul radio al masinii la soclul lor pe placa Alset: cel pentru ESC (controlor electronic de viteza) si cel pentru servomotor. Amandoua sunt apoi controlate de Alset prin PWM si PPM (pulse-position modulation).


### Detectarea semnelor de circulatie
Modelul de cascade Haar este utilizat impreuna cu opencv, astfel semnele de circulatie pot fi implementate cu usurinta, folosind [script-urile](https://github.com/robert-saramet/alset-v2/tree/main/tools) pe care le-am dezvoltat pentru extragerea informatiilor din fisierele JSON ale dataset-ului. Pipeline-ul consta in obtinerea unui numar rezonabil de imagini pozitive (500+ imagini care contin semnul in cauza) si imagini negative (care nu contin semnul) macar cat jumatate din numarul de imagini pozitive. Dataset-ul utilizat de noi poate fi gasit [aici](https://www.mapillary.com/dataset/trafficsign) si contine ~40000 de imagini in total, toate etichetate in fisere JSON.

Apoi, un fisier pos.txt trebuie creat, continand numele tuturor imaginilor pozitive (acest lucru se poate realiza cu script-ul `parse_jsons.py`). Acesta va fi utilizat pentru generarea fisierului `.vec`. Pentru asta, veti avea nevoie si de toolkit-ul opencv. Pe sisteme UNIX, manager-ul de pachete poate instala totul, dar pe Windows va trebui sa descarcati [versiunea 3.4.x](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/), nu ultima, intrucat versiunile ulterioare ale openCV nu mai contin toolkit-ul pentru generarea de cascade Haar. Dupa instalarea opencv, sunteti gata sa incepeti. Pentru a genera fisierul `.vec` mentionat anterior, va trebui sa utilizati programul `opencv_createsamples`. De exemplu:

```
openv_createsamples -info pos.txt -w 24 -h 24 -num 1000 -vec pos.vec
```

Cu fisierul `.vec` pe care tocmai l-ati creat si un fisier `neg.txt` continand numele tuturor fisierelor negative, puteti utiliza programul `opencv_traincascade`:

```
opencv_traincascade -data YourCascadeFolder/ -vec pos.vec -bg neg.txt -w 24 -h 24 -numPos YourNumOfPosImg, -numNeg YourNumOfNegImg
```
    
Documentatie completa asupra acestor comenzi puteti gasi pe [site-ul opencv]( https://docs.opencv.org/3.4/dc/d88/tutorial_traincascade.html).

Fisierul final `cascade.xml` poate fi gasit in `YourCascadeFolder`, impreuna cu stagiile (`stage0.xml`, `stage1.xml`, `stage2.xml` etc), utilizate principal pentru regresarea cascadei sau *salvarea progresului daca programul se opreste neasteptat*.
Alternativ, puteti utiliza [versiunea neoficiala cu GUI](https://amin-ahmadi.com/cascade-trainer-gui/).

Cascadele Haar sunt incarcate la pornire de catre raspberry pi, care utilizeaza opencv pentru a identifica semnele de circulatie vazute de camerea. Datele generate (pozitie, distanta) sunt apoi procesate.

### Urmarirea strazii
Functioneaza doar pe drumuri marcate, detectand linia de centru cu algoritmul Canny pentru detectarea marginilor. Dupa procesare (convertirea la grayscale, blurarea si aplicarea de edge detection pentru obtinerea contururilor), o lista de drepte geometrice este generata pentru determinarea pozitiei masinii relativ la drum. Acesti vectori sunt apoi adunati, iar vectorul final va decide directia masinii. Astfel, virajele medii vor functiona fara asistenta de la utilizator. Totusi, pe drumuri care nu sunt marcate corespunzator, se poate obtine comportament neasteptat.

### Siguranta
Trei intrerupatoare individuale pentru motoare, circuitul cu arduino si raspberry pi + router permit testarea usoara si fara risc, oferind si posibilitatea dezactivarii functiilor momentan nedorite (ex. dezactivarea raspberry pi-ului cand nu se foloseste opencv). Daca orice dispozitiv se opreste in timpul operarii sau daca semnalul este pierdut sau prea invechit, masina opreste instant.

### GPS
- ##### **Pe arduino**
Pentru navigare GPS, un modul U-Blox Neo-6M este conectat la Pololu 328PB, care extrage informatiile despre latitudine, longitudine, viteza si directia din propozitiile NMEA folosind biblioteca TinyGPS++. Coordonatele destinatiei vor fi trimise de ESP32 de la webapp-ul de pe raspberry pi. Dupa selectarea destinatiei, informatiile de navigare/viraje sunt obtinute prin propozitia NMEA GPRMB. Pentru planificarea rutei (pe raspberry pi), locatia initiala este oferita de telefon. 
- ##### **Pe raspberry pi**
Comunicarea dintre telefon si raspberry pi se realizeaza printr-un server Flask, care poate fi accesat prin conectarea la router-ul wifi al robotului atunci cand sunteti in proximitatea acestuia. Webapp-ul ii permite utilizatorului sa introduca adresa destinatiei si, daca aceasta este valida, o ruta va fi selectata de raspberry pi, iar latitudinea si longitudinea destinatie sunt transmise la arduino. Backend-ul pentru acestea utilizeaza API-ul REST de la HERE Maps.

### Aplicatie pentru telefon
O aplicatie pentru telefon dezvoltata in platforma Blynk este disponibilia, oferind un joystick, kill switch, slider de viteza, precum si cateva functii care momentan nu sunt implementate pe partea de arduino. Desi sunt disponibile atat bluetooth cat si wifi, noi am observat ca bluetooth-ul este mai fiabil, si are avantajul suplimentar de a nu necesita o conexiune la internet pe ESP32. Desi aceasta aplicatie ramane disponibila pentru utilizarea in lipsa unui controller de PS4, pentru moment am abandonat-o datorita conectivitatii sale instabile si naturii closed-source.

### Securitate
Securitatea lui Alset este eficienta pe cat este de simpla. Exista trei moduri de a interactiona cu masina: controller-ul de PS4, aplicatia pentru telefon, si server-ul de pe raspberry pi. Atat aplicatia cat si controller-ul utilizeaza bluetooth si necesita impereche initiala, implicand proximitate fata de dispozitiv si *cunoasterea PIN-ului de imperechere*. In plus, ESP32 se va conecta doar la adresa MAC a controller-ului, sau, pentru aplicatie, la *API key-ul atribuit telefonului*. In plus, raspberry pi este conectat la un router wifi portabil plasat pe masina, care este protejat cu o parola puternica. Aceste functii fac orice atac asupra Alset dificil.

### Comunicatii
Toate placile sunt conectate la ESP32 prin UART (intrucat ESP32 nu are suport adecvat pentru slave mode la I2C), mai putin 328PB, care se conecteaza la 32u4 prin I2C si este transmisa mai departe catre ESP32 prin acelasi UART folosit si in comunicatiile 32u4-ESP32 (din lipsa interfetelor seriale). ESP32 functioneaza ca centrala si controlor de logica pentru tot robotul, primind orice date de la modulele care nu sunt conectate in mod direct la el prin biblioteca SerialTransfer library. Acest lucru ne permite sa actualizam codul intr-un singur loc, totusi folosindu-ne in continuare de placie suplimentare, astfel incat sa nu suprasolicitam ESP32, atat ca putere de procesare cat si GPIO/UART.

### Viteza automata
Controlul automat al vitezei este inca in lucru. Scopul este de a permite masinii sa mentina aceeasi viteza indiferent de teren, baterie sau orice alti factori. Momentan, viteza unei roti (in rotatii pe secunda) poate fi masurata cu un senzor optic de viteza conectat la un pin capabil de interrupts de pe 32u4. Algoritmul pentru controlul vitezei in sine ramane sa fie implementat. De asemenea, Un slider pentru setarea intervalului de viteze va fi activat curand.

### Pathfinding
Pentru evitarea obstacolelor, exista sase senzori ultrasonici: unul in fata, doua la 45 de grade stanga/dreapta in fata, doua in stanga/dreapta si unul in spate. Asezarea curenta a senzorilor a fost aleasa pentru maximizarea acoperirii, vitezei si potentialului de pathfinding. Algoritmul utilizat este unul custom, creat special pentru Alset. Acesta ia in calcul datele de la fiecare senzor si incearca sa aleaga directia optima pentru a evita coliziunile. Senzorii ultrasonici sunt impartiti pe 32u4 si 328PB datorita latentei lor


### Alimentare
Exista trei surse de alimentare utilizate pe Alset, in felul urmator: masina rc este alimentata de propriul ei battery pack NiMH, circuitul arduino este alimentat de doi acumulatori 18650 in paralel, iar un powerbank alimenteaza raspberry pi-ul si router-ul. Motivele pentru utilizarea a trei surse diferite sunt:
- intrucat masinile RC necesita baterii speciale cu rate de descarcare foarte mari si aproape nicio protectie, acestea nu sunt o alegere buna pentru alimentarea a orice altceva decat motoarele
- circuitul arduino nu poate procura destul curent prin usb, in plus, ar fi nevoie de multe cabluri
- pentru raspberry pi si router, un powerbank comercial are mai mult sens decat un circuit special
- mai mult, cele doua dispozitive mentionate mai sus ar putea fi considerate de alti utilizatori si sarite complet
Pentru a incarca bateriile circuitului arduino, doar conectati o sursa usb obisnuita la modulul de pe placa. Procedati la fel pentru powerbank. Pentru incarcarea bateriei masinii RC, intrucat exista nenumarate tipuri de incarcatoare, baterii si conectori, si intrucat Alset incearca sa fie cat mai cross-platform, recomandam sa utilizaati hardware OEM pentru incarcarea masinii.

### Debugging
Debugging-ul lui Alset v2 este facut simplu de design-ul placii, care permite testarea usoara a conexiunilor electrice si indepartarea rapida a componentelor, precum si de flag-ul pentru debugging disponibil pentru activarea comunicatiilor usb, in scopul inspectarii fiecarei valori. Ecranul LCD afiseaza, deasemeanea, multe dintre aceste valori, oferind insight util fara nevoia de un PC. LED-uri de status vor fi in curand activate pentru a asista suplimentar in acest scop. In plus, toate porturile USB sunt pozitionate in aceeasi parte exterioara a robotului pentru a facilita accesul.

---

### Componente
- [Raspberry Pi 3 B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) (openCV, server)
- [Sparkfun ESP32 Thing](https://www.sparkfun.com/products/13907) (procesor principal, comunicatii, IMU)
- [Pololu A-Star Micro 32u4](https://www.pololu.com/product/3101) (motorare, senzori ultrasonici & de viteza)
- [Pololu A-Star Micro 328PB](https://www.pololu.com/category/239/a-star-328pb-micro) (GPS, LCD, senzori ultrasonici & de tensiune)
- [Maverick Quantum MT](https://hpi-racing.ro/automodele-rc/7332-automodel-maverick-quantum-mt-110-brushed-albastru-rtr-rc.html) (masina RC)
- [Router Portabil 4G TP-Link](https://www.emag.ro/router-wireless-n300-tp-link-3g-4g-portabil-tl-mr3020/pd/EC23TBBBM/) (optional)
- [Controller PS4 DualShock](https://www.playstation.com/en-ro/accessories/dualshock-4-wireless-controller/)
- [6x Senzori Ultrasonici HC-SR04](https://cleste.ro/senzor-ultrasonic-hc-sr04.html)
- [GPS U-blox NEO-6M](https://www.u-blox.com/en/product/neo-6-series)
- [IMU Adafruit NXP 9DOF](https://www.adafruit.com/product/3463)
- [LCD I2C 20x04](https://cleste.ro/ecran-oled-0-91.html)
- [Regulator Step-Up Pololu 3.3V/5V](https://www.pololu.com/product/2872/specs)
- [Incarcator Li-ion TP4056](https://cleste.ro/modul-incarcare-baterii-litiu-1a-usb-c-tp4056.html)
- [2x Intreupatoare](https://cleste.ro/buton-panou.html)
- [2x Convertoare Nivel Logic I2C](https://cleste.ro/modul-ic-i2c-nivel-conversie.html)
- [1 Suport Acumulatori 18650 2P](https://www.optimusdigital.ro/ro/suporturi-de-baterii/12108-suport-de-baterii-2-x-18650-conectare-in-paralel.html)
- Cateva lucruri care nu sunt inca implementate, precum:
LED-uri, shift registers, intrerupatoare DIP 8p, bara LED, buzzer

---

### Resurse
- #### [Mapillary Traffic Sign Dataset](https://www.mapillary.com/dataset/trafficsign)
- #### [HERE Maps API](https://developer.here.com/develop/rest-apis)
- #### [OpenCV Library](https://opencv.org/)
- #### [Cascade Trainer GUI](https://amin-ahmadi.com/cascade-trainer-gui/)
- #### [MJPG Streamer](https://github.com/jacksonliam/mjpg-streamer)
- #### [SerialTransfer Library](https://github.com/PowerBroker2/SerialTransfer)
- #### [PS4 Controller Library](https://github.com/aed3/PS4-esp32)
- #### [TinyGPS++ Library](https://github.com/mikalhart/TinyGPSPlus)
- #### [Adafruit AHRS Library](https://github.com/adafruit/Adafruit_AHRS)

---
### Licenta
Copyright 2021 Robert Saramet, Bogdan Maciuca

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

---

### Roluri
- #### Robert Saramet
  - Controller PS4
  - Pathfinding
  - Comunicatii
  - App pentru telefon
  - Hardware & electronica
  - Documentatie
  - Tot codul de pe arduino
- #### Bogdan Maciuca
  - Urmarirea strazii
  - Recunoasterea semnelor
  - Webapp navigatie GPS
  - Servere Flask & Node
  - Documentatie
  - Tot codul de pe raspberry pi

---

### Support us
- ##### BTC: bc1q9zjrnzd04w27sx4d0hy9n06hu624dmvjc495wc
[![219971756-519102119360658-4905840849781814643-n.png](https://i.postimg.cc/nLCd65GD/219971756-519102119360658-4905840849781814643-n.png)](https://postimg.cc/JyCcXpgr)
