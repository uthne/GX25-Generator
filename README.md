# GX25-Generator
ESP32 controller for 12 volt generator based on Honda GX25 four-stroke engine and a belt driven BLDC (brushless) motor as dynamo through a 3-phase to DC rectifier.

The controller uses two servos, a hall sensor and three INA226 voltage monitors to monitor and govern the engine.
One servo controls the throttle to generate approx 20 volts 3-phase current. One INA226 sensor reads the rectified 3-phase voltage from the dynamo and adjust the servo accoringly. Another INA226 sensor monitors the regulated 12 volt output voltage and a hall sensor reads the engine RPM. These measurements are mainly for display and fault detection. 
The other servo operates the engine choke below 3000 RPM. The choke can be engaged by a button on the generator or through a web-interface.
The generator also has an internal batterypack consisting of three 18650 3.7 volt batteries, and an Uninterupted Power Supply/Charger. Total voltage of batteries are monitored with one INA226 voltage sensor, and outputs from charger circuit status LEDs (through opto-couplers on pin 32 and 33). 

The generator can connect to an existing wifi network or set up it own Wifi Access Point. Through wifi the generator serve a basic website to monitor and control the operation.

One webpage displays output voltage, dynamo voltage, battery status, RPM and a STOP button and a CHOKE button. Below 3000 RPM, pressing the CHOKE button reveal slider controls for the choke- and the throttle servo. When the choke-panel is open the throttle is also fully manual.
Pressing the CHOKE button on the generator will set choke to 80% and open the choke-panel in the web interface.
Pressing the real hardware buttons buttons on the generator will also be reflected in the web-interface.

A Settings page shows the current network status and options to set up credetials to a local wifi-network. There are also a button to SCAN for local networks and a button to DELETE stored credentials. The web interface use WebSockets to update between the webpage and the generator. Several clients can be connected simultaniously and get updates in realtime. 
In the web interface you can also restart the controller, even while the generator is running. Current throttle setting will be saved and reloaded on RESTART if the engine is running. Holding both buttons on the generator till the end of "sensor restart" sequence will trigger the same soft restart.

There is an OLED display on the generator showing basic status and current network name (SSID) and current IP-address. By default the generator will try to connect o a local WiFi with stored credentials. If connecting to local WiFi fails it will set up its own Wifi Access Point called GENERATOR. Credentials for local wifi can be set up on the preference page by connecting to the network and IP address shown in the display.

Pressing the CHOKE button on the generator when turned on and holding until CHOKE light is lit, will skip connecting to local WiFi and set up its own Access Point. The generator will also set up an Access Point if connecting to local WiFi fails.

Pressing the STOP and CHOKE button at the same time will force the generator to run a simple self-diagonse and restart its sensors. 

Pressing the STOP button at any time (also when not running) will short out the ignition and stop engine. A red light on the generator indicate STOP button is engaged. Pressing the STOP button again will disengage the "stopped" mode. 
A relay is driven by pin 14 through a level shifter to 5 volt, and connected to the ignition (shortcircuit) wire. Whenever STOP-light is on solid the relay is activated and ignition is grounded. 

Since buttons and web inteface are not directly controlling the engine computer, but rather putting factors into its conrol cycle, there may be up to one second delay between input and confitmation of input. Especially buttons on the generator should be pressed for at least half a second to register.

The process is not using threading (multitasking). It is controlling a physical engine with inherent inertia. Adjustments of throttle and choke are not time critical, neighter is updating display and web interface.

Lights on generator have several fault modes with both lights blinking:
Both lights on with short cut-off every second:       Display could not be initialized.
Both lights giving short burst every second:          Fault in a voltage sensor. Error in display.
Both lights blinking on and of every second:          Filesystem could not be initialized. Error in display.
Both lights alernating on and off every second:       Emergency stop. Error in display.
Lights givig short burst in succession every second:  Voltage sensors are resetting. Message in display.
                                                      Continuing to press both buttons will soft restart the controller (se restart button above).





Tested on
ESP32-D0WD-V3 (revision 3) Chip
Features: Wi-Fi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
Crystal is 40MHz
MAC: f4:65:0b:47:e1:08
