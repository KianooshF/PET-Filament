#include <PID_v1.h>
#include <thermistor.h> //https://github.com/miguel5612/Arduino-ThermistorLibrary
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Encoder.h>
#include <AccelStepper.h>
#include <Arduino.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

const int temperaturePin = A0; // Define the remaining IO pins for motor, pushbutton & thermistor
const int pwmPin = 3;
const int motDirPin = PD6;
const int motStepPin = PD7;
#define motorInterfaceType 1

AccelStepper myStepper(AccelStepper::DRIVER, motStepPin, motDirPin);

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define initialTemp 220 // Define parameters for each edjustable setting
#define minTemp 190
#define maxTemp 250
#define initialSpeed 22
#define minSpeed 1
#define maxSpeed 40
#define initialMot 1
#define minMot 0
#define maxMot 2

int encLowLim = minTemp; // Variables to store the encoder limits and increment
int encHighLim = maxTemp;
int encCurrent = initialTemp;
int oldencCurrent = initialTemp;
int dataInputNo = 0; // Data input tracking, 0 - temp, 1 - speed, 2 - motor

#define Clock PCINT11 // Clock pin connected to D9
#define Data PCINT10  // Data pin connected to D8
#define Push 15  // Push button pin connected to D10

unsigned long TimePuch = 0;
Encoder myEnc(Data, Clock);

byte oldButtonState = HIGH;            // First button state is open because of pull-up resistor
const unsigned long debounceTime = 10; // Debounce delay time
unsigned long buttonPressTime;         // Time button has been pressed for debounce
boolean pressed = false;

int loopTime = 500; // Define time for each loop cycle
unsigned long currentTime = 0;

double Kp = 80.0; // Define PID constants
double Ki = 35.0;
double Kd = 80.0;

THERMISTOR thermistor(temperaturePin, // Analog pin
                      50,             // Nominal resistance at 25 ºC
                      3100,           // thermistor's beta coefficient
                      54);            // Value of the series resistor
                                      // Connect thermistor on A2

double setpoint = initialTemp; // Define PID variables & establish PID loop
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int motSpeed = initialSpeed; // Define motor parameters
int motDir = initialMot;
int motMaxDelay = 100;

void updateDataDisplay();
void runMotor();

void setup()
{
  // Serial.begin(115200); // Initialize serial communication
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    // Serial.println("SSD1306 allocation failed");
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay(); // Clear the buffer

  pinMode(pwmPin, OUTPUT); // Configure PWM pin

  pid.SetMode(AUTOMATIC); // Set the PID parameters
  pid.SetOutputLimits(0, 255);

  input = thermistor.read(); // Read and set the initial input value

  // pinMode(Clock,INPUT_PULLUP);
  // pinMode(Data,INPUT_PULLUP);
  pinMode(Push, INPUT_PULLUP);

  myStepper.setMaxSpeed(50);
  myStepper.setSpeed(10);
  // Serial.println("Setup complete"); // Write to serial monitor to indicate the setup function is complete
}

void loop()
{
  // Record the start time for the loop
  currentTime = millis();

  // Read the temperature
  input = thermistor.read(); // read temperature

  // Compute the PID output
  pid.Compute();

  // Update the PWM output
  analogWrite(pwmPin, output);
  // int temp = input;

  // Print the temperature and PWM output
  // Serial.print("Temperature: ");
  // Serial.print(temp);
  // Serial.print(" \u00B0C");
  // Serial.print("\tPWM Output: ");
  // Serial.print(output);
  // Serial.print("\tEncoder: ");
  // Serial.println(encCurrent);

  // Update the OLED display
  updateDataDisplay();
  // Check for input on the pushbutton
  while (millis() < currentTime + loopTime)
  {
    encCurrent = (myEnc.read()/4);
    byte buttonState = digitalRead(Push);
    if (buttonState != oldButtonState)
    {
      // Serial.println("Button change");
      if (millis() - buttonPressTime >= debounceTime) // Debounce button
      {
        buttonPressTime = millis();   // Time when button was pushed
        oldButtonState = buttonState; // Remember button state for next time
        if (buttonState == LOW)
        {
          pressed = true;
          // Serial.println("Button Pressed");
        }
        else
        {
          if (pressed == true) // Confirm the input once the button is released again
          {
            pressed = false;
            // Serial.println("Button Released");
            if (dataInputNo == 0) // Set which parameter is being edited and define limits
            {
              dataInputNo = 1;
              encCurrent = motSpeed;
              encLowLim = minSpeed;
              encHighLim = maxSpeed;
            }
            else if (dataInputNo == 1)
            {
              dataInputNo = 2;
              encCurrent = motDir;
              encLowLim = minMot;
              encHighLim = maxMot;
            }
            else
            {
              dataInputNo = 0;
              encCurrent = setpoint;
              encLowLim = minTemp;
              encHighLim = maxTemp;
            }
          }
        }
      }
    }

    // Set the parameter being edited equal to the current encoder position
    if (dataInputNo == 0)
    {
      setpoint = encCurrent + initialTemp;
    }
    else if (dataInputNo == 1)
    {
      motSpeed = encCurrent + initialSpeed;
      // motDelayTime = 100 * (1 + maxSpeed - motSpeed);
      myStepper.setSpeed(motSpeed);
    }
    else
    {
      motDir = encCurrent;
    }

    // Set the motor direction
    if (motDir == 0)
    {
      // pinMode(enablePin, OUTPUT);   // Enable motor
      // digitalWrite(motDirPin, LOW); // Reverse motor direction
      myStepper.moveTo(-1);
      updateDataDisplay();
    }
    else if (motDir == 2)
    {
      // pinMode(enablePin, OUTPUT);    // Enable motor
      // digitalWrite(motDirPin, HIGH); // Forward motor direction
      myStepper.moveTo(1);
      updateDataDisplay();
    }
    else
    {
      // pinMode(enablePin, INPUT); // Disable motor
      myStepper.disableOutputs();
      updateDataDisplay();
    }

    // Pulse the stepper motor if forward or reverse is selected
    if (motDir != 1)
    {
      while (true)
      {
        encCurrent = (myEnc.read());

          if (encCurrent >= 2)
          {
            encCurrent = 2;
          }
          else if (encCurrent <= 0)
          {
            encCurrent = 0;
          }
        if (oldencCurrent != encCurrent)
        {
          oldencCurrent = encCurrent;
          myEnc.write(encCurrent);
        }
        byte buttonState = digitalRead(Push);
        if (buttonState != oldButtonState && millis() - buttonPressTime >= debounceTime)
        {
          buttonPressTime = millis(); 
          oldButtonState = buttonState;
          encCurrent = 0;
          myEnc.write(encCurrent);
        }
        motDir = encCurrent;
        runMotor();
        updateDataDisplay();
      }
    }
  }
}

// IRAM_ATTR void EnterVoid()
// {
//   if (millis() - TimePuch > 50)
//   {
//     TimePuch = millis();
//     buttonState = !buttonState;
//   }
// }

// Update the OLED display contents
void updateDataDisplay()
{
  display.clearDisplay();              // Clear display
  display.setTextSize(1);              // Set the text size
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(2, 20);            // Set the display cursor position
  display.print("Current Temp: ");     // Set the display text
  display.setCursor(2, 30);
  display.print(F("Set Temp: "));
  display.setCursor(2, 40);
  display.print(F("Extrude Speed: "));
  display.setCursor(2, 50);
  display.print(F("Extrude: "));
  int temp = input;
  int setPointInt = setpoint;
  int selected = 0;
  if (dataInputNo == 0) // Set the cursor position
  {
    selected = 30;
  }
  else if (dataInputNo == 1)
  {
    selected = 40;
  }
  else
  {
    selected = 50;
  }
  display.setCursor(87, selected); // Set the display cursor position
  display.print(F(">"));
  display.setCursor(97, 20);
  display.print(temp);
  display.print(F("C"));
  display.setCursor(97, 30);
  display.print(setPointInt);
  display.print(F("C"));
  display.setCursor(97, 40);
  display.print(motSpeed);
  display.setCursor(97, 50);
  if (motDir == 0)
    display.print(F("Rev"));
  else if (motDir == 2)
    display.print(F("FWD"));
  else
    display.print(F("OFF"));
  display.display(); // Output the display text
}

// Turn the reel motor and maintain hot end temperature
void runMotor()
{
  // Move the motor one step
  myStepper.run();

  // Read the temperature
  input = thermistor.read(); // read temperature

  // Compute the PID output
  pid.Compute();

  // Update the PWM output
  analogWrite(pwmPin, output);
}