# Linux-Alert-System
A proximity alert system must be designed and implemented using the following elements:<br />
• The HC-SR04 Ultrasonic Ranging Module;<br />
• The STM32F746-DISCOVERY board;<br />
• The Linux OS.<br />
The STM32F746-DISCOVERY shall read the distance of any object located in front of the HCSR04
sensor, and then it shall blink the user led with a period proportional to the distance,
according to the following table:<br />
Distance [cm] < 10 [10-25) [25-50) [50-75) [75-100] >100<br />
Period [ms] ∞ 400 600 800 1.000 2.000<br />
The STM32F746-DISCOVERY shall print on its serial console the measured distance.<br />
