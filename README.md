# Obstacle-Detection-

# Problem Statement :

The objective of this project is to develop an obstacle detection system using an STM32 microcontroller and an HC-SR04 ultrasonic sensor, aimed at accurately measuring distances to nearby objects and providing real-time visual alerts. The system will utilize ultrasonic waves to detect obstacles and calculate distances, triggering specific LEDs based on proximity thresholds. If an obstacle is detected at a distance greater than 100 cm, the internal blue LED on the STM32 board will illuminate, indicating a safe zone. For distances less than 100 cm, an external green LED will light up, warning of nearby obstacles. If the distance falls below 25 cm, a critical alert will be indicated by one green LED remaining steady and a second green LED blinking rapidly to signal imminent collision risk. The solution will involve integrating the STM32 board with the ST-Link debugger for programming and debugging, ensuring real-time responsiveness for practical applications in fields such as robotics and automation.

# Procedure :

Step 1: Connect the STM32 Board to the ST-Link Debugger
	1.1: Connect the STM32 board to the ST-Link debugger
		1.1.1: Connect GND pin on the STM32 board to GND pin 6 on the ST-Link debugger.
		1.1.2: Connect SCK pin on the STM32 board to SWCLK pin 2 on the ST-Link debugger.
		1.1.3: Connect DIO pin on the STM32 board to SWDIO pin 4 on the ST-Link debugger.
		1.1.4: Connect SW pin on the STM32 board to 3.3V pin 8 on the ST-Link debugger.
Step 2: Connect the Ultrasonic Sensor to the STM32 Board
	2.1: Connect Vcc pin of the ultrasonic sensor to the 3V pin on the STM32 board.
	2.2: Connect GND pin of the ultrasonic sensor to the GND pin on the STM32 board.
	2.3: Connect Trigger pin of the ultrasonic sensor to A0 pin on the STM32 board.
	2.4: Connect Echo pin of the ultrasonic sensor to A1 pin on the STM32 board.

Step 3: Run the Code in Keil uVision5

Step 4: Load the ST-Link Utility

Step 5: Process Explanation
	5.1: The ultrasonic sensor emits sound waves. If these waves encounter an obstacle, they reflect back to the sensor and are detected at the receiver end.
	5.2: The formula to calculate the distance is:

	Distance = (Speed of sound × Time ) / 2 

	This gives the actual distance between the obstacle and the sensor.

Step 6: Response Based on Distance
	6.1: If an obstacle is more than 100 cm away, the internal blue LED on the STM32 board will turn on, indicating a safe distance.
	6.2: If an obstacle is less than 100 cm away, the internal blue LED will turn off, and an external green LED will turn on, indicating that an obstacle is within a 100 cm radius.
	6.3: If an obstacle is less than 25 cm away:
		6.3.1: The first green LED remains on.
		6.3.2: The second green LED will start blinking rapidly, indicating that you are very close to the obstacle.
