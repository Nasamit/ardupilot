# Ardupilot on 86Duino One #
This ported Ardupilot is version 3.6 dev. (2017.09.15)
The Ardupilot flight stack is modified to run on DuinOS?(FreeDOS)

# Requirements #
1. a DuinOS SD card (http://www.86duino.com/index.php?p=11878)
2. 9-axis IMU, Barometers (Recommend GY-91 sensor module)
3. 6 channel RC Transmitter/Receiver
4. GPS module (optional)

# Supported hardware
GY-86 sensor module (I2C, MPU6050, MS5611, HMC5883L)
GY-91 sensor module (SPI, MPU9250 [pin.9], BMP280 [pin.8])
VL53L0X range finder (I2C)
Ublox-M8N GPS module (UART)
6-CH RC receiver(PWM)

# (Windows) Build ArduPilot for 86duino #
Only ArduCopter had been tested on 86duino One.

1. Download the toolchain (url)
2. Run Ardupilot.bat (for setup environment)
3. Navigate to the ArduCopter source code folder (eg. C:\86ardupilot\ArduCopter\)
4. run "make 86duino –j4"

First time installation:
Copy ArduCopter.exe (in ArduCopter folder) into the SD card, then rename it to _86duino.exe (replace the original one).
	
Upload firmware by usb connection(for developer):
2. Run Ardupilot.bat
3. Navigate to the Tools folder (eg. C:\86ardupilot\Tools\)
2. run "upx ..\..\ArduCopter\ArduCopter.exe"
3. run "v86dude com'N' 20 ..\..\ArduCopter\ArduCopter.exe standalone" ('N' is the usb comport number)

# Setup Ardupilot #
Same as other ardupilot flight controller, please refer the official tutorial
http://ardupilot.org/copter/docs/configuring-hardware.html

# Known Issues #
1. Currently the system is running in single thread, thus the SD card R/W may cause the drone crash. Strongly recommend disable the Log function.(set LOG_BACKEND_TYPE to none)
