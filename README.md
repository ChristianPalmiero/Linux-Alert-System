# Linux-Alert-System
A proximity alert system has been designed and implemented using the following elements:<br />
• The HC-SR04 Ultrasonic Ranging Module;<br />
• The STM32F746-DISCOVERY board;<br />
• The Linux OS.<br />
The STM32F746-DISCOVERY reads the distance of any object located in front of the HCSR04
sensor, and then it blinks the user led with a period proportional to the distance,
according to the following table:<br />
| Distance [cm] | <10 | [10-25) | [25-50) | [50-75) | [75-100] | >100  |
|:-------------:|:---:|:-------:|:-------:|:-------:|:--------:|:-----:|
| Period [ms]   | ∞   | 400     | 600     |800      | 1.00 0   | 2.000 |
The STM32F746-DISCOVERY prints on its serial console the measured distance.<br />
