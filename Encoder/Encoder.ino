#include <AFMotor.h>
#include "RingBuffer.h"
#include <cmath>
#include <cstdio>


// Pin assignments
const int rightInterruptPin = 2;
const int leftInterruptPin = 3;
const int startPin = 10;
const int modeChangePin = 11;
const int analogPin = 0;

// Initialize motors and LCD screen
AF_DCMotor rightMotor(3, MOTOR34_1KHZ);
AF_DCMotor leftMotor(4, MOTOR34_1KHZ);

// Encoder variables
struct EncoderData
{
    EncoderData() { count = 0; }
    void reset() { count = 0; pulseTimes.clear(); pulseTimes.push(0ul); }
    volatile long count;
    volatile RingBuffer<unsigned long, 6> pulseTimes;
    bool forward;
};

EncoderData rightData;
EncoderData leftData;
long startButtonCount = 0;
long modeButtonCount = 0;
const long buttonCountThreshold = 3;
unsigned long stoppedTime = 0;
const unsigned long allowButtonsTime = 500000; // microseconds
const unsigned long debounceMicros = 250;

// Physical parameters
const float wheelDiameter = 0.1f; // meters
const long pulsesPerRev = 6 * 25;
const float ticksPerSecond = 1000000.f;

// Function prototypes
void rightPulse();
void leftPulse();
float getPosition(EncoderData& data);
float getVelocity(EncoderData& data);
void startMoving();
void stopMoving();

// Command variables
float rightPositionCommand;
float rightVelocityCommand;
float leftPositionCommand;
float leftVelocityCommand;
enum CommandMode
{
    positionMode,
    velocityMode
};
CommandMode commandMode;

// Integral terms
const float integralThreshold = 255.f;
float rightVelInt = 0.f;
float leftVelInt = 0.f;

// Tuning parameters
const float pl_kp = 1.f;
const float pl_ki = 0.f;
const float pl_kd = 0.f;
const float vl_kp = 0.01f;
const float vl_ki = 0.01f;
const float vl_kd = 0.f;

// Loop control
unsigned long lastMillis = 0;
unsigned int loopPeriodMs = 20;
unsigned int cycleCount = 0;
const unsigned int displayUpdateCycles = 10;

// Movement control
bool started = false;
bool showResults = false;
unsigned long startedMillis;
const float positionThreshold = 0.01; // meters
const float velocityThreshold = 0.01; // m/s
const unsigned long positionTimeout = 20000; // milliseconds
const unsigned long velocityRunTime = 5000; // milliseconds
const float positionMin = -10.f; // meters
const float positionMax = 10.f; // meters
const float velocityMin = -1.f; // m/s
const float velocityMax = 1.f; // m/s

// Button interrupt routines
const unsigned long debounceMillis = 500;
unsigned long startReleaseTime = 0;
unsigned long modeReleaseTime = 0;


void setup()
{
    // Set up pins
    pinMode(rightInterruptPin, INPUT);
    pinMode(leftInterruptPin, INPUT);
    pinMode(startPin, INPUT);
    pinMode(modeChangePin, INPUT);
    digitalWrite(rightInterruptPin, HIGH);
    digitalWrite(leftInterruptPin, HIGH);
    digitalWrite(startPin, HIGH);
    digitalWrite(modeChangePin, HIGH);
    
    rightData.pulseTimes.push(0ul);
    leftData.pulseTimes.push(0ul);
    
    commandMode = positionMode;
    
	attachInterrupt(0, &rightPulse, RISING);
    attachInterrupt(1, &leftPulse, RISING);
    
    Serial.begin(9600);
}


void loop()
{
    if (started)
    {
		if (commandMode == positionMode)
		{
			bool rightReached = (rightPositionCommand > 0.f) ? 
								getPosition(rightData) >= rightPositionCommand :
								getPosition(rightData) <= rightPositionCommand;
			bool leftReached = (leftPositionCommand > 0.f) ? 
								getPosition(leftData) >= leftPositionCommand :
								getPosition(leftData) <= leftPositionCommand;
			
			if (rightPositionCommand > 0.f)
				rightMotor.run(FORWARD);
			else
				rightMotor.run(BACKWARD);
				
			if (leftPositionCommand > 0.f)
				leftMotor.run(FORWARD);
			else
				leftMotor.run(BACKWARD);
				
			rightMotor.setSpeed(rightReached ? 0 : 255);
			leftMotor.setSpeed(leftReached ? 0 : 255);
			
			if ( rightReached && leftReached )
				stopMoving();
			
			rightData.forward = rightPositionCommand > 0.f;
			leftData.forward = leftPositionCommand > 0.f;
		}
		else // velocityMode
		{
			rightMotor.setSpeed( (unsigned char)(fabs(rightVelocityCommand) * 255.f / velocityMax) );
			leftMotor.setSpeed( (unsigned char)(fabs(leftVelocityCommand) * 255.f / velocityMax) );
			
			if (rightVelocityCommand > 0.f)
				rightMotor.run(FORWARD);
			else
				rightMotor.run(BACKWARD);
				
			if (leftVelocityCommand > 0.f)
				leftMotor.run(FORWARD);
			else
				leftMotor.run(BACKWARD);
			
			if (millis() - startedMillis >= velocityRunTime)
				stopMoving();
			
			rightData.forward = rightVelocityCommand > 0.f;
			leftData.forward = leftVelocityCommand > 0.f;
		}
		
		/*
	
        // Position loop
        float rightPosErr = rightPositionCommand - getPosition(rightData);
        float leftPosErr = leftPositionCommand - getPosition(leftData);
        
        if (commandMode == positionMode)
        {
            rightVelocityCommand = pl_kp * rightPosErr;
            leftVelocityCommand = pl_kp * leftPosErr;
        }
        
        // Velocity loop
        float rightVelErr = rightVelocityCommand - getVelocity(rightData);
        float leftVelErr = leftVelocityCommand - getVelocity(leftData);
        
        rightVelInt += rightVelErr * (loopPeriodMs / 1000.f);
        leftVelInt += leftVelErr * (loopPeriodMs / 1000.f);
        
        // Limit integral terms to prevent windup
        rightVelInt = constrain(rightVelInt, -integralThreshold, integralThreshold);
        leftVelInt = constrain(leftVelInt, -integralThreshold, integralThreshold);
        
        float rightAccel = vl_kp * rightVelErr + vl_ki * rightVelInt;
        float leftAccel = vl_kp * leftVelErr + vl_ki * leftVelInt;
        
        rightMotor.run((rightAccel > 0.f) ? FORWARD : BACKWARD);
        rightMotor.setSpeed( (unsigned char)fabs(rightAccel) );
        leftMotor.run((leftAccel > 0.f) ? FORWARD : BACKWARD);
        leftMotor.setSpeed( (unsigned char)fabs(leftAccel) );
        
        rightData.forward = rightAccel > 0.f;
        leftData.forward = leftAccel > 0.f;
        
        // Handle stopping
        if (commandMode == positionMode)
        {
            // Stop when close to the target distance with a low speed or after a timeout
            if ( (fabs(rightPosErr) < positionThreshold && fabs(leftPosErr) < positionThreshold &&
                  fabs(rightVelErr) < velocityThreshold && fabs(leftVelErr) < velocityThreshold) ||
                millis() - startedMillis >= positionTimeout )
            {
                started = false;
                showResults = true;
            }
        }
        else // velocityMode
        {
            // Stop after a certain time
            if (millis() - startedMillis >= velocityRunTime)
            {
                started = false;
                showResults = true;
            }
        }
		*/
    }
    else
    {
        rightMotor.run(RELEASE);
        leftMotor.run(RELEASE);
        
        // Update command value
        float value = analogRead(analogPin) / 1023.f;
        
        if (commandMode == positionMode)
        {
            rightPositionCommand = positionMin + value * (positionMax - positionMin);
            leftPositionCommand = positionMin + value * (positionMax - positionMin);
        }
        else // velocityMode
        {
            rightVelocityCommand = velocityMin + value * (velocityMax - velocityMin);
            leftVelocityCommand = velocityMin + value * (velocityMax - velocityMin);
        }
    }
    
    
    // Update display once every few cycles 
    if (cycleCount % displayUpdateCycles == 0)
    {
        Serial.print("?f");
    
        if (commandMode == positionMode)
        {
            if (started || showResults)
            {
                Serial.print("?x00?y0RPos: ");
                Serial.print(getPosition(rightData));
                Serial.print("?x00?y1LPos: ");
                Serial.print(getPosition(leftData));
            }
            else
            {
                Serial.print("?x00?y0RPCmd: ");
                Serial.print(rightPositionCommand);
                Serial.print("?x00?y1LPCmd: ");
                Serial.print(leftPositionCommand);
            }
        }
        else // velocityMode
        {
            if (started)
            {
                Serial.print("?x00?y0RVel: ");
                Serial.print(getVelocity(rightData));
                Serial.print("?x00?y1LVel: ");
                Serial.print(getVelocity(leftData));
            }
            else if (showResults)
            {
                Serial.print("?x00?y0RCnt: ");
                Serial.print(rightData.count);
                Serial.print("?x00?y1LCnt: ");
                Serial.print(leftData.count);
            }
            else
            {
                Serial.print("?x00?y0RVCmd: ");
                Serial.print(rightVelocityCommand);
                Serial.print("?x00?y1LVCmd: ");
                Serial.print(leftVelocityCommand);
            }
        }
    }
    
    
    // Limit loop speed to a consistent value to make timing and integration simpler
    while (millis() - lastMillis < loopPeriodMs)
    {
    }
    
    lastMillis = millis();
    ++cycleCount;
}


void rightPulse()
{
    unsigned long time = micros();
    
    if (time - rightData.pulseTimes[0] > debounceMicros)
    {
		rightData.pulseTimes.push(time);
		if (rightData.forward)
			rightData.count++;
		else
			rightData.count--;
		
        if (time - stoppedTime < allowButtonsTime)
        {
            stoppedTime = time;
        }
        else if (!started)
		{
			if ( abs(rightData.count - startButtonCount) >= buttonCountThreshold )
				startMoving();
		}
    }
}


void leftPulse()
{
    unsigned long time = micros();
    
    if (time - leftData.pulseTimes[0] > debounceMicros)
    {
        leftData.pulseTimes.push(time);
		if (leftData.forward)
            leftData.count++;
		else
		    leftData.count--;
			
		if (time - stoppedTime < allowButtonsTime)
        {
            stoppedTime = time;
        }
        else if (!started)
		{
			if ( abs(rightData.count - modeButtonCount) >= buttonCountThreshold )
			{
				if (showResults)
					showResults = false;
				else
					commandMode = (commandMode == positionMode) ? velocityMode : positionMode;
			}
		}
    }
}


float getPosition(EncoderData& data)
{
    noInterrupts();
    long count = data.count;
    interrupts();
    return count * wheelDiameter / pulsesPerRev;
}


float getVelocity(EncoderData& data)
{
    // Assume zero velocity before enough data is gathered to estimate
    if (data.pulseTimes.size() != data.pulseTimes.capacity())
        return 0.f;
    
    noInterrupts();
    unsigned long lastDiff = data.pulseTimes[0] - data.pulseTimes[4];
    unsigned long newDiff = micros() - data.pulseTimes[0];
    interrupts();
    
    /**
     * If the current pulse diff is taking longer than the previous pulse diff, 
     * use it for the velocity calculation instead. This causes the velocity to 
     * go to zero when the robot is stationary (no pulses are generated).
     */
    if (newDiff > lastDiff / 4ul * 2ul)
        return (data.forward ? 1.0f : -1.0f) * wheelDiameter / ((newDiff * pulsesPerRev) / ticksPerSecond);
    else
        return (data.forward ? 1.0f : -1.0f) * wheelDiameter / (((lastDiff * pulsesPerRev) / 4ul) / ticksPerSecond);
}

void startMoving()
{
	started = true;
    startedMillis = millis();
    noInterrupts();
    rightData.reset();
    leftData.reset();
    interrupts();
	rightVelInt = 0;
	leftVelInt = 0;
}

void stopMoving()
{
	started = false;
    showResults = true;
	stoppedTime = micros();
}
