#include <AFMotor.h>
#include "RingBuffer.h"

#define toggleSwitch 11
#define IRSensor A0
#define potMeter A1
#define baudrate 9600

// Store previous values
RingBuffer<float, 5> distValues;
RingBuffer<float, 5> distErrorValues;
RingBuffer<float, 5> velErrorValues;

// PID tuning constants
float dist_Kp = 0.f;
float dist_Ki = 0.f;
float dist_Kd = 0.f;
float vel_Kp = 0.f;
float vel_Ki = 0.f;
float vel_Kd = 0.f;

// Controller limits
const float distIntLimit = 1000000.f;
const float velIntLimit = 1000000.f;
const float velSetpointLimit = 1.f; // m/s

// Integral accumulators
float distErrorInt = 0.f;
float velErrorInt = 0.f;

// Setpoints
float distSetpoint = 0.1f; // meters

// Loop control
const unsigned int loopPeriodMs = 20;
float dt = loopPeriodMs / 1000.f;
unsigned long lastMillis = 0;
unsigned int cycleCount = 0;
const unsigned int displayUpdateCycles = 10;

void setup() {
	Serial.begin(baudrate);
	pinMode(toggleSwitch, INPUT);
	digitalWrite(toggleSwitch, HIGH);
}

void loop() {
    /* Use a position feedback loop to control the setpoint of a velocity feedback loop */    
    // Position loop
    distValues.push(getDistance());
    float distError = distSetpoint - distValues[0];
    distErrorValues.push(distError);
    
    distErrorInt += distError * dt;
    distErrorInt = constrain(distErrorInt, -distIntLimit, distIntLimit);

    float velSetpoint = dist_Kp * distError + dist_Ki * distErrorInt + dist_Ki * derivative(distErrorValues, dt);
    velSetpoint = constrain(velSetpoint, -velSetpointLimit, velSetpointLimit);
    
    // Velocity loop
    float velError = velSetPoint - derivative(distanceValues, dt);
    velErrorValues.push(velError);
    
    velErrorInt += velError * dt;
    velErrorInt = constrain(velErrorInt, -velIntLimit, velIntLimit);
    
    float velSetpoint = vel_Kp * velError + vel_Ki * velErrorInt + vel_Ki * derivative(velErrorValues, dt);
    
    
    // Interface logic
	if (switchPressed(toggleSwitch)) {
		while (switchPressed(toggleSwitch)) {
			clearScreen();
			Serial.print("?x00?y0");
			Serial.print("Release to tune");
			delay(100);
		}
		clearScreen();
		tuneParameters();
	} else {
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Kp = ");
		Serial.print("?x05?y0");
		Serial.print(Kp);
		Serial.print("?x00?y1");
		Serial.print("Ki = ");
		Serial.print("?x05?y1");
		Serial.print(Ki);
	}
    
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (millis() - lastMillis < loopPeriodMs) {}
    
    lastMillis = millis();
    ++cycleCount;
}


float getPosition() {
    return 0.f;
}


float derivative(RingBuffer<float, 5>& x, float dt) {
    // Uses a 4th order backward finite difference method the approximate the derivative of a set of data
    return ( 2.083333f * x[0] - 4.f * x[1] + 3.f * x[2] - 1.333333f * x[3] + 0.25f * x[4] ) / dt;
}


void clearScreen() {
	Serial.print("?f");
}


void putCursor(int x, int y) {
	String line = "?x" + String(x) + "?y" + String(y);
	Serial.print(line);
}


void printValue(String label, int data) {
	clearScreen();
	putCursor(0,0);
	Serial.print(label);
	putCursor(label.length(),0);//intentional +1 for spacing
	Serial.print(data);
}


bool switchPressed(int button) {
	if (!digitalRead(button))
		return true;
	else
		return false;
}


void tuneParameters() {
	while (!switchPressed(toggleSwitch)) {
		Kp = tune("Kp = ");
		delay(100);
	}
	clearScreen();
	while (switchPressed(toggleSwitch)) {
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Release to tune next");
	}
	clearScreen();
	while (!switchPressed(toggleSwitch)) {
		Ki = tune("Ki = ");
		delay(100);
	}
	while (switchPressed(toggleSwitch)) {
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Release to exit");
	}
}


int tune(String label) {
		clearScreen();
		Serial.print("?x00?y0");
		Serial.print("Turn potmeter");
		Serial.print("?x00?y1");
		Serial.print(label);
		Serial.print("?x05?y1");
		int parameter = analogRead(potMeter);
		Serial.print(parameter);
		return parameter;
}