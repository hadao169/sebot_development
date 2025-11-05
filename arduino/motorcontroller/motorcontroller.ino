#include "Motor.h"
#include <Servo.h>

const int trigPin = A3;
const int echoPin = A2;
const int servoPin = A4;

Servo servo;
Motor *Motor::instances[2] = {nullptr, nullptr};
int Motor::instanceCount = 0;

// Motor motor1(4,5,9,2,3);
Motor motor1(6, 8, 5, 3, 4);
// Motor motor2(11, 12, 10, 2, 9);
Motor motor2(7, 12, 11, 2, 9);

int mode = 0; // 0=PWM 1=SPEED
int alive = 0;
int aliveSignal = 10 * 3; // 3s
int printDelay = 0;

enum ScanState {
  DRIVING,
  SWEEPING
};
ScanState scanState = DRIVING;
unsigned long lastScanTime = 0;
const int SWEEP_TIME = 800; // 800ms per action
int sweepIndex = 0; // 0: start, 1: left, 2: middle, 3: right, 4: report
int leftDistance = 1000;
int rightDistance = 1000;

unsigned long startTime = 0;
String msg;

void setup()
{
	motor1.begin();
	motor2.begin();
	servo.write(90);
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	servo.attach(servoPin);
	Serial.begin(115200);
}

void loop()
{
	handleObstacleScan(millis());
	readSerialPort();

	if (millis() - startTime >= 1)
	{
		startTime++;
		if (printDelay == 0)
		{
			printDelay = 100; // 100ms

			if (alive > 0)
			{
				alive--;
			}

			if (aliveSignal == 0)
			{
				alive = 1;
			}

			motor1.run(mode, alive);
			motor2.run(mode, alive);

			sendData();
		}
		printDelay--;
	}
}

void readSerialPort()
{
	msg = "";
	String sa[4];
	int r = 0, t = 0;

	if (Serial.available())
	{
		msg = Serial.readStringUntil('\n');

		for (int i = 0; i < msg.length(); i++)
		{
			if (msg.charAt(i) == ';')
			{
				sa[t] = msg.substring(r, i);
				r = (i + 1);
				t++;
			}
		}

		if (sa[0] == "PWM")
		{
			mode = 0;
			motor1.setPWM(sa[1].toInt());
			motor2.setPWM(sa[2].toInt());
		}
		else if (sa[0] == "SPD")
		{
			mode = 1;
			alive = aliveSignal;
			motor1.setSPD(sa[1].toInt());
			motor2.setSPD(sa[2].toInt());
		}
		else if (sa[0] == "PID")
		{
			motor1.setPID(sa[1].toFloat(), sa[2].toFloat(), sa[3].toFloat());
			motor2.setPID(sa[1].toFloat(), sa[2].toFloat(), sa[3].toFloat());
		}
		else if (sa[0] == "ZERO")
		{
			motor1.zeroEncoder();
			motor2.zeroEncoder();
		}
		else if (sa[0] == "ALIVE")
		{
			aliveSignal = sa[1].toInt() * 10;
		}
		else if (sa[0] == "SCAN")
		{
			scanState = SWEEPING;
			sweepIndex = 0;
			lastScanTime = millis();
		}

		Serial.flush();
	}
}

void sendData()
{
	Serial.print(motor1.getEncoder());
	Serial.print(";");
	Serial.print(motor2.getEncoder());
	Serial.print(";");
	Serial.print(motor1.getSpeed());
	Serial.print(";");
	Serial.print(motor2.getSpeed());
	Serial.print(";");
	Serial.print(motor1.getMotorSpeed());
	Serial.print(";");
	Serial.print(motor2.getMotorSpeed());
	Serial.print(";");
	Serial.print(readDistance());
	Serial.print("\n");
}

void sendScanData()
{
	Serial.print("SCAN;");
	Serial.print(leftDistance);
	Serial.print(";");
	Serial.print(rightDistance);
	Serial.print("\n");
}

int readDistance()
{
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	long duration = pulseIn(echoPin, HIGH);
	return (duration / 2 / 29.41);
}

void handleObstacleScan(unsigned long currentTime)
{
	if (scanState == DRIVING)
	{
		return;
	}

	if (scanState == SWEEPING)
	{
		if (currentTime - lastScanTime < SWEEP_TIME)
		{
			return;
		}
		lastScanTime = currentTime;

		switch (sweepIndex)
		{
		case 0:
			servo.write(30);
			sweepIndex = 1;
			break;
		case 1:
			rightDistance = readDistance();
			servo.write(90);
			sweepIndex = 2;
			break;
		case 2:
			servo.write(150);
			sweepIndex = 3;
			break;
		case 3:
			leftDistance = readDistance();
			servo.write(90);
			sweepIndex = 4;
			break;
		case 4:
			sendScanData();
			sweepIndex = 0;
			scanState = DRIVING;
			break;
		}
	}
}
