# ESP32C3 based bluetooth to Flysky IBUS bridge

My combat robots are designed to interface with a Flysky FS-A8S receiver in IBUS mode, however I typically use a pistol grip/wheel controller rather than the more typical stick controllers, and most people don't find it particularly intuitive.

I wrote this Arduino sketch to allow me to use a Seeed Xiao ESP32C3 as a bluetooth receiver, paired to a Google Stadia controller (currently the cheapest BLE compatible game controller available on ebay, once flashed with the Bluetooth firmware). Other controllers probably work, but you will need to experiment with button mapping and trigger/stick ranges.

For the Stadia controller I've mapped typical video game controls to the standard RC channels, so that the right trigger is forward, left is reverse (with a speed limit to prevent accidentally reversing into the pit at top speed), left stick X axis is steer, and the A button controls a lifter servo on channel 4. The left and right bumpers can be used to trim the servo in, so that the lifter arm touches the ground but does not push into it. This setting is not currently persistent, but could be since the ESP32C3 has flash.

Power consumption of the ESP32C3 is about 2-3 times higher than my Flysky receivers, so ensure the ESP has a good stable 5V voltage source - if insufficient the ESP will reboot upon pairing. Bluetooth also has much less range than the AFHDS-2A protocol that Flysky uses, so while you can get away without an antenna on the Flysky receiver in most situations, the ESP32 requires an antenna for more than 2-3m of stable connection.

Most combat robotics rulesets require robots to failsafe if they lose connection to the controller. While the bluepad library offers a callback for controller disconnection, this can take some time (10-15s). Since the controller only sends updates on change of state, I've set a timeout of two seconds since the last change of state before going to failsafe (pausing the sending of ibus packets - my robot controller board will failsafe as though the wire has broken). This means if you're simply holding the same button or trigger down for two seconds the robot will stop moving. This is fine for combat robotics, since you're constantly varying control inputs, but could cause issues for other types of vehicle. This can be changed as a define at the top of the sketch.

Software requirements are the Bluepad32 board package (https://github.com/ricardoquesada/bluepad32) and TaskScheduler (https://github.com/arkhipenko/TaskScheduler). Bluepad32 requires you to use its own board package, rather than the generic ESP32 board package, so double check it's selected in Arduino IDE if the code fails to compile.

If you don't see any serial debug output, enable "USB CDC on boot" in the Arduino Tools menu.

To pair your controller:

* Hard code the MAC address of the controller into the sketch. The easiest way to find this is to pair it to a computer or phone and copy it from the Bluetooth settings menu.
* Power up the ESP32, either in a robot/RC vehicle or via USB.
* Hold Y and the Stadia button for 2-3 seconds. The Stadia button will pulse orange. When it goes solid white, the controller is paired to the ESP32.
* The ESP32 should send the ibus channel values over the USB serial connection on receipt of data from the controller.
