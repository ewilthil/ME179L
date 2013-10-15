#include <AFMotor.h>
#include <Servo.h>
#include <cmath>
#include "RingBuffer.h"

#define paramSwitch 11
#define exponentSwitch 12
#define mantissaSwitch 13
#define IRSensor A0
#define potMeter A1
#define baudrate 9600

// Motors and servos
Servo servo;

// Store previous values
RingBuffer<float, 5> distValues;
RingBuffer<float, 5> distErrorValues;
RingBuffer<float, 5> velErrorValues;

// PID tuning constants
float dist_Kp = 10.f;
float dist_Ki = 0.f;
float dist_Kd = 0.f;
float vel_Kp = 100.f;
float vel_Ki = 0.f;
float vel_Kd = 0.f;

// Controller limits
float distIntLimit = 1000000.f;
float velIntLimit = 1000000.f;
float velSetpointLimit = 1.f; // m/s

// Integral accumulators
float distErrorInt = 0.f;
float velErrorInt = 0.f;

// Setpoints
float distSetpoint = 0.2f; // meters

// Loop control
const unsigned int loopPeriodMs = 20;
float dt = loopPeriodMs / 1000.f;
unsigned long lastMillis = 0;
unsigned int cycleCount = 0;
const unsigned int displayUpdateCycles = 10;

// Steering parameters
int servoCenter = 65;
int servoRight = 100;
int servoLeft = 30;

// Function prototypes
float getDistance();
float derivative(RingBuffer<float, 5>& x, float dt);

// Tuning data structures
struct Parameter {
	float* var;
	const char* name;
};
Parameter params[] = {
	{ &dist_Kp, "dist_Kp" },
	{ &dist_Ki, "dist_Ki" },
	{ &dist_Kd, "dist_Kd" },
	{ &vel_Kp, "vel_Kp" },
	{ &vel_Ki, "vel_Ki" },
	{ &vel_Kd, "vel_Kd" },
	{ &distIntLimit, "distIntLim" },
	{ &velIntLimit, "velIntLim" },
	{ &velSetpointLimit, "velSPLim" },
	{ &distSetpoint, "distSP" },
};
int currentParam = 0;
bool switchParamsPressed = false;
bool setExponentPressed = false;
bool setMantissaPressed = false;
enum InterfaceMode {
    viewMode,
    exponentMode,
    mantissaMode,
};
InterfaceMode interfaceMode;
int currentExponent;
float currentMantissa;
const int maxExponent = 6;
const int minExponent -3;



void setup() {
	Serial.begin(baudrate);
	pinMode(toggleSwitch, INPUT);
	digitalWrite(toggleSwitch, HIGH);
	servo.attach(10);
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
    float velError = velSetpoint - derivative(distValues, dt);
    velErrorValues.push(velError);
    
    velErrorInt += velError * dt;
    velErrorInt = constrain(velErrorInt, -velIntLimit, velIntLimit);
    
    float controlValue = vel_Kp * velError + vel_Ki * velErrorInt + vel_Ki * derivative(velErrorValues, dt);
    
    // Use velocity loop output to control the steering angle
	int servoValue = servoCenter + (int)controlValue;
	servoValue = constrain(servoValue, servoLeft, servoRight);
	servo.write(servoValue);
    
    
    // Handle buttons
    bool temp = switchPressed(exponentSwitch);
    
    if (temp && !paramSwitchPressed) {
        interfaceMode = viewMode;
        ++currentParam;
        currentParam %= sizeof params / sizeof (Parameter);
    }
    paramSwitchPressed = temp;
    
    if (temp && !exponentSwitchPressed) {
        if (interfaceMode == exponentMode) {
            interfaceMode = viewMMode;
            *params[currentParam].var = currentMantissa * std::pow(10.f, (float)currentExponent);
        } else {
            interfaceMode = exponentMode;
            currentMantissa = *params[currentParam].var / std::pow(10.f, (float)(int)std::log10(*params[currentParam].var));
        }
    }
    exponentSwitchPressed = temp;
    
    if (temp && !mantissaSwitchPressed) {
        if (interfaceMode == mantissaMode) {
            interfaceMode = mantissaMode;
            *params[currentParam].var = currentMantissa * std::pow(10.f, (float)currentExponent);
        } else {
            interfaceMode = exponentMode;
            currentExponent = (int)std::log10(*params[currentParam].var);
        }
    }
    mantissaSwitchPressed = temp;
    
    // Draw interface
    if (cycleCount % displayUpdateCycles == 0) {
        clearScreen();
        
        switch (interfaceMode) {
        case viewMode:
            break;
        case exponentMode:
            currentExponent = minExponent + (int)((long)analogRead(potMeter) * (maxExponent - minExponent) / 1024);
            putCursor(15, 0);
            Serial.print("E");
            break;
        case mantissaMode:
            currentMantissa = analogRead(potMeter) / 1024.f;
            if (currentMantissa < 0.1f) {
                currentMantissa = 0.f;
            }
            putCursor(15, 0);
            Serial.print("M");
            break;
        }
        
        putCursor(0, 0);
        Serial.print(params[currentParam].name);
        putCursor(0, 1);
        Serial.print(*params[currentParam].var);
    }
    
	/*
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
	*/
    
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (millis() - lastMillis < loopPeriodMs) {}
    
    lastMillis = millis();
    ++cycleCount;
}


float getDistance() {
    const float c1 = 0.0291f;
	const float c2 = 0.00076f;
	
	float value = analogRead(IRSensor);
	
	return 1.f / (value * c1 + c2);
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
	return !digitalRead(button)
}

/*
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
*/