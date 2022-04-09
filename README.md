# Using Xensiv PAS CO2 with Raspberry Pi

Example Python code for Infineon Xensiv PAS CO2 sensor using I2C.

Connections (sensor → RPi):
* VDD3.3 → 3.3V
* SCL → SCL
* TX/SDA → SDA
* GND → GND (of RPi and external 12 V supply)
* PSEL → GND
* VDD12 → external 12 V supply

Resources:
* [Web site](https://www.infineon.com/cms/en/product/sensor/co2-sensors/pasco2v01/)
* [Data sheet](https://www.infineon.com/dgdl/Infineon-EVAL_PASCO2_SENSOR-DataSheet-v01_00-EN.pdf?fileId=5546d462758f5bd10175934ec4215c6a)
* [Register map](https://www.infineon.com/dgdl/Infineon-Registermap_description_PASCO2_MA2-ApplicationNotes-v02_00-EN.pdf?fileId=5546d4627600a6bc017604238d967785)
* Evaluation board [EVAL_PASCO2_MINIBOARD](https://www.infineon.com/cms/en/product/evaluation-boards/eval_pasco2_miniboard/) ([application note](https://www.infineon.com/dgdl/Infineon-UM_XENSIV_PAS_CO2_miniboard-UserManual-v01_01-EN.pdf?fileId=5546d4627a0b0c7b017a43155685460e))

Official C/C++ libraries:
* [PAS CO2 Sensor](https://github.com/Infineon/pas-co2-sensor)
* [XENSIV PAS CO2 sensor](https://github.com/Infineon/sensor-xensiv-pasco2)
* [PAS CO2 Sensor Arduino Library](https://github.com/Infineon/arduino-pas-co2-sensor)
