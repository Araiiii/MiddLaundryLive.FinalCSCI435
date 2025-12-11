# MiddLaundryLive.FinalCSCI435
## Developers: Arai Hardy, Sebastian Cruz
### CSCI 435 MiddLaundry Live
#### December 10,2025

This is the final write-up and all information related to our embedded system and code

### Introduction
Laundry availability is a recurring challenge across many college campuses, and Middlebury is no exception. As residents, we frequently found ourselves walking to the laundry room only to discover that every washer was in use. With roughly 300 residents in some dorms and as few as six washers available- approximately fifty students assigned to each machine in some cases -the lack of information about real-time machine usage often led to frustration, wasted time, and inconsistent access. Through this project, we aimed to address this problem directly by building an embedded system and web interface that track washer and dryer availability and make that information accessible to students in real time BEFORE they even leave their rooms.

Our project, MiddLaundryLive, integrates a compact embedded sensor system with a cloud-connected website. The embedded system uses an ESP32 microcontroller paired with an Adafruit LSM6DSOX 6-DoF accelerometer to detect whether a laundry machine is running based on its vibration state. This data of that acceleration is transmitted over WiFi using the Middlebury IoT network and MQTT protocol, and our website retrieves and visualizes these machine states for students to view instantly.

We chose this project because it directly improves everyday student life while allowing us to explore multiple domains of embedded systems design: sensor data processing, microcontroller programming, WiFi communication, MQTT messaging, and frontend web development. The project offered a natural progression from hardware understanding to real-world application- something we both felt would work as a perfect final for this course. We consulted multiple resources to understand the LSM6DSOX sensor, the MQTT protocol, ESP32 WiFi programming using Arduino IDE, and other sources that helped us compare methods and refine our vibration detection algorithm. These included the Adafruit LSM6DSOX datasheet, documentation for HiveMQ and MQTT client libraries, and many more which will all be linked at the end of our report.

Our report explains how we built MiddLaundryLive from the ground up, including hardware design, software logic, interface development, testing, and eventual results. We aim to make this report detailed and instructional enough so that anyone with similar hardware could recreate our prototype.

### Methods
#### Overview

Our ES consists of three main components:

##### 1. ESP32 Microcontroller
 Responsible for sensor reading, network connection, and MQTT communication.


##### 2. Adafruit LSM6DSOX 6-DoF IMU (Accelerometer + Gyroscope)
Attaches to the ESP32 to measure vibration and motion.


##### 3. Optional: Lithium Battery Pack & Magnets
Powers the above 2 components and connects magnetically for portable use in laundry rooms.

###### see reference image 1.1

We connected the LSM6DSOX to the ESP32 over I²C using custom SDA/SCL pins (SDA = GPIO 23, SCL = GPIO 22 using Twowire; otherwise 21 and 22 as default) or if using the ESP32 V2 we connected directly with a QT wire. The device was housed in a small enclosure that we adhered to machines with strong magnets, allowing for a flat surface for accurate readings, easy mounting, and removal.

#### The LSM6DSOX

We decided on the LSM6DSOX because of its precise accelerometer capabilities, low power consumption, and flexible data transmission ranges. Its robust documentation, STEMMA QT connectors, and I²C compatibility made it ideal for prototyping on the ESP32. The sensor’s selective high sensitivity allowed us to reliably distinguish between machines that were running versus idle without relying on audio or power draw monitoring, which would require invasive modifications to campus equipment- and the fact that false positives are a pain to deal with.

#### Bill Of Materials (BOM)

link here: https://docs.google.com/spreadsheets/d/1i75-UsUPBVsZZJmRdL-ENZNd7RExQrMRsME6GZocTyc/edit

###### see reference image 1.2

This leaves us with a cost per unit of 47.35. 

The Beadboard, LED, and resistors were meant to be used for a LED board that would dynamically show the states of the machines outside of the laundry room. However we scrapped this idea, as the website functionality makes it obsolete. 

#### Software and Data Flow

Our system consisted of three layers:

##### 1. Embedded Vibration Detection (ESP32 + LSM6DSOX)


##### 2. MQTT Communication via Middlebury IoT Network


##### 3. MiddLaundryLive Web Interface

The ESP32 continuously sampled acceleration data, processed it, and determined the machine state (“on” or “off”). First the pre-computed long average was subtracted from the acceleration vector to remove orientation plus gravity. Then the centered acceleration is integrated over a short average to "exaggerate" the measurements and have more confidence in the state setting. Finally, an Exponential Moving Average(EMA) value slowly follows the integrated average, rising quickly to be sensitive enough to change, while falling slowly to accommodate for machine pausing or slowing down. After testing the machines we defined vibration thresholds that distinguished normal idle motion from active wash-cycle movement. In essence, if the EMA rises above a certain tuned threshold, the system classified the machine as on, and if it fell below a lower threshold, it classified the machine as off. It is important to remember that the conditions of the machines or cycles are NOT constant, because of this we had to optimize our algorithm to be general enough to capture various cases- i.e when a machine is in delicate mode versus heavy duty. Or likewise, when a machine pauses during the rinse or spin cycle that pause should not be considered as “machine off”. After determining the state, the ESP32 transmits  JSON payload every few seconds to an MQTT topic associated with the specific building and machine.

Our simple website subscribes to these topics and updates the UI accordingly.

#### Sensor Data Processing and Logic

The core challenge in vibration detection is distinguishing normal idle vibrations (background noise, minor building vibrations, people walking near the machines) from machine-active vibrations. To accomplish this, we designed a two-stage algorithm that transitions across three system states:

##### 1. Calibration State

Immediately after startup, the system spends ~30 seconds determining a baseline for the vibration magnitude. During this time, the sensor uses a running average to compute the mean magnitude of sensor readings while the machine is presumed off. Then in detection mode this mean magnitude constant is subtracted from the sensor readings to remove the orientation the sensor was installed and the acceleration the sensor experiences due to gravity. This helps center the readings about 0, which allows the sensor to spot variations in the vibrations. 

###### see reference image 1.3

##### Detection State

Once calibrated, the system enters the detection mode. Here we implemented an exponential moving average (EMA) smoothing technique on a short-window RMS-like value:

- Remove pre-computed long-term average (orientation + gravity)
- Compute absolute magnitude deviation by integrating over a short buffer
- Smooth EMA value increases quickly and decreases slowly (using different values for rising vs falling energy)
- Set the machine's state by comparing the smoothed EMA to set thresholds. 
- Send the machine’s state to the MQTT broker


This enables responsive yet stable detection of machine states. Thresholds were manually tuned from machine tests; When the smoothed energy exceeded the ON threshold, the machine was considered running. When it fell below the OFF threshold, the machine was considered idle. Off and On thresholds are not continuous to avoid rapid changes in state. 

##### 3. Polling State (Backup Mode)

We also designed a simpler polling-based algorithm for testing and fallback purposes. Here, only the presence of movement above a minimal threshold and a timeout interval were used to define active vs inactive states. If no motion was detected for two minutes, the machine was determined to be off. This backup mode was useful during early testing when our main detection loop was still being tuned. It helps save computations as there is no need to maintain a running EMA value. 

#### MQTT Communication

After computing the machine state, the ESP32 published a JSON payload to HiveMQ’s public MQTT broker under a topic structured as:

laundry/<building>/<machineID>/data

For example:
laundry/Battell/02/data

###### see reference image 1.4

Messages were published every couple hundred cycles(about 5 seconds), ensuring a near-real-time update rate without excessive network traffic.

We initially tested MQTT publishing using the IoT MQTT Panel mobile app. The ability to visualize the real-time graphs, values, and state changes confirmed that our embedded system was transmitting accurately long before the website was complete.

###### see reference image 1.5-6

#### Website Interface

The website was built in React and used the MQTT JavaScript library to connect via secure WebSockets to the same broker. When the user selected a building, the site subscribed to all machine topics for that building.
Each machine defaulted to “No Data” until an MQTT message was received, after which its card updated to:

##### Available (machine off)

##### Unavailable (machine on)

The UI was clean, text-based, and fast, updating automatically on each MQTT event. This interface bridged the embedded system with an accessible digital platform intended for daily student use.

### Results 

Our final prototype successfully detected when washers were active and broadcast these states over MQTT, which the website displayed accurately in real time. The LSM6DSOX proved sensitive enough to differentiate between machine idle and active states, and the thresholds we tuned provided stable detection even when the machines vibrated inconsistently or briefly slowed down during wash cycles, or even if they were set on a different axis (x,y,z).

#### Successful Outcomes

- Our embedded system successfully connected to Middlebury IoT WiFi- even if the received signal strength was weak (RSSI dBm).


- MQTT messages were consistently updated, delivered, and received without noticeable delay.


- The website updated nearly instantly upon receiving sensor data.


- Users could switch between buildings and see machine availability.


- Our detection algorithm correctly classified machine states in multiple real-world tests.

##### Failure Cases

We observed that:

- If the device was not firmly magnetically mounted, detection accuracy dropped.

- Certain wash cycles with unusual vibration patterns occasionally produced borderline readings, causing short flickers between on/off states.

- WiFi signal strength in basement laundry rooms could occasionally affect MQTT publishing.

- We understood these failures as natural properties of low-power IoT devices operating in challenging environments. In future iterations, these issues can be mitigated with better enclosures, smoothing logic, or local caching.

### Accessibility Concerns

From the beginning, we wanted MiddLaundryLive to be helpful for all users. Several accessibility considerations guided our website design:

- We used clear color-coded indicators and semiotics but paired them with text labels (“Available,” “Unavailable”) to aid all users

- The website layout is simple and uncluttered, allowing for easy navigation on mobile devices, which students are most likely to use. 

- The system reduces unnecessary physical travel, which supports users who may have mobility limitations.

- We ensured everyone on campus can access the site on ANY kind of device whether personal or provided by the school.

### Ethical Considerations

Since our embedded system collects only vibration data and does not store identifiers, audio, or personal information, it raises minimal privacy concerns. Still, IoT devices in shared spaces often introduce questions regarding monitoring and data security.

We addressed these concerns by:

- Ensuring no personally identifiable information is collected.
- Using open MQTT topics only for non-sensitive data.
- Designing the system to monitor machines not people.
- The site is limited to Midd students through a simple go link to maintain campus privacy without storing or transmitting any sensitive or potentially personal data to outsiders.

If deployed at scale, we would need to consider device authentication, secure MQTT brokers, and perhaps local network encryption. Ethically, the goal is to enhance convenience without compromising privacy or institutional policies.

### Schedule and Reflection

##### Actual Schedule

The vibration detection and MQTT communication tasks generally followed our planned timeline. However, the website took longer than expected due to time spent learning how to parse MQTT messages accurately and then use the MQTT protocol in React. The LED display was ultimately deprioritized due to time constraints and our focus on getting the website and embedded system stable. We realized that an LED indicator inside the laundry room would only assist students who were already present, which did not align with our primary goal of providing REMOTE machine availability. 

##### Reflection

We collaborated closely and jointly on almost every task, which made debugging easier and sped up development. The unexpected hiccup from our schedule came from the time spent refining the detection algorithm for either of our boards. Since the models of our ESP32’s were slightly different, the wiring required one of us to use Twowire - which then required slightly different pin connections that we mentioned in the overview. 

### Issues Encountered

Several challenges arose during development:

- WiFi Registration: We had to register each ESP32 on Middlebury IoT network, which occasionally reset connections.

- Accelerometer Noise: Early readings were noisy and inconsistent until we implemented long-window averaging and smoothing.

- Website MQTT Syntax Issues: Parsing machine IDs from topic strings caused early bugs where multiple machines overwrote each other.

- Physical Mounting: The system needed strong magnetic mounting to detect vibration consistently.

###### see reference image 1.7-8

### Future Implementations

If we had more time, we would expand MiddLaundryLive with:

1. Multiple sensor units for entire dorms rather than a single prototype.
2. Push notifications for when a machine becomes available.
3. Crowdsourced reporting, allowing users to manually flag broken machines after however many days.
4. Improved casing with waterproofing and better magnet mounts.
5. Machine learning–based vibration classification for more precise detection of wash cycles and detecting peak times.

## We are so proud of the way this project turned out, and you can visit it through our go link at: go/MiddLaundryLive/

### Our Github Pages Website link is: MiddLaundryLive.FinalCSCI435

#####References 

- ESP32 Arduino Core: GitHub Repository
- MQTT Protocol Specification: mqtt.org
- Web Accessibility Initiative Guidelines: w3.org/WAI
- https://www.youtube.com/watch?v=ABaWFOmk1Hk&t=190s
- https://www.youtube.com/watch?v=yyDuGxI6ir8
- https://www.youtube.com/watch?v=RiRyap-EVg0
- https://www.researchgate.net/figure/Mechanism-used-in-MEMS-accelerometer_fig2_362072478
- https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
- https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/downloads
- https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/pinouts
- https://www.st.com/resource/en/datasheet/lsm6dsox.pdf

