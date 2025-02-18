#include <PID_v1.h>
#include <thermistor.h> //https://github.com/miguel5612/Arduino-ThermistorLibrary
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Encoder.h>
#include <Arduino.h>

//-------------------LCD Define-------------------

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define first_Menu 0
#define secend_Menu 1
#define third_Menu 2
#define fourth_Menu 3
#define five_Menu 4

//-------------------Temp Out/In-------------------

const int pwmPin = 3;
const int temperaturePin = A0; // Define the remaining IO pins for motor, pushbutton & thermistor

//-------------------Motor Out-------------------

#define initialSpeed 10
#define initialMot 0

const int stepsPerRevolution = 200;      // تعداد گام‌ها در هر دور
const unsigned long minFrequency = 80;   // حداقل فرکانس پالس‌ها (۱۰۰ هرتز)
const unsigned long maxFrequency = 3000; // حداکثر فرکانس پالس‌ها (۱۰ کیلوهرتز)
const unsigned long maxOCRValue = 65535; // حداکثر مقدار OCR1A
const unsigned long minOCRValue = 80;    // حداقل مقدار OCR1A (برای فرکانس‌های عملی)

const int motDirPin = PD6;
const int motStepPin = PD7;

unsigned int ocrValue = 0;

int motSpeed = initialSpeed; // Define motor parameters
int motDir = initialMot;

//-------------------Oled-------------------

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int OLED_status = 0;

//-------------------PID-------------------

#define initialTemp 220 // Define parameters for each edjustable setting

double setpoint = initialTemp; // Define PID variables & establish PID loop
double temp, output;
double Kp = 80.0; // Define PID constants
double Ki = 35.0;
double Kd = 80.0;
PID pid(&temp, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//-------------------Rotary-------------------

#define Clock A3 // Clock pin connected to D9  //FIXME
#define Data A2  // Data pin connected to D8
#define Push A1  // Push button pin connected to D10

long position = 0;
bool flag_change = false;

Encoder myEnc(Data, Clock);

byte oldButtonState = HIGH; // First button state is open because of pull-up resistor

//-------------------System-------------------

int state_program = 0;
int Data_input = initialTemp;

bool Start_Stop = true;
bool Motor_status = true;

//-------------------
#define minTemp 190
#define maxTemp 250
#define minSpeed 1
#define maxSpeed 40
#define minMot 0
#define maxMot 2

int encCurrent = initialTemp;
int oldencCurrent = initialTemp;
int dataInputNo = 0; // Data input tracking, 0 - temp, 1 - speed, 2 - motor

unsigned long TimePuch = 0;

const unsigned long debounceTime = 10; // Debounce delay time
unsigned long buttonPressTime;         // Time button has been pressed for debounce
boolean pressed = false;

int loopTime = 500; // Define time for each loop cycle
unsigned long currentTime = 0;

THERMISTOR thermistor(temperaturePin, // Analog pin
                      50,             // Nominal resistance at 25 ºC
                      3100,           // thermistor's beta coefficient
                      54);            // Value of the series resistor
                                      // Connect thermistor on A2

int motMaxDelay = 100;

void updateDataDisplay();
void HotEnd();
void roteryread();
void Input();

void setup()
{
  Serial.begin(115200); // Initialize serial communication
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

  temp = thermistor.read(); // Read and set the initial input value

  // pinMode(Clock,INPUT_PULLUP);
  // pinMode(Data,INPUT_PULLUP);
  pinMode(Push, INPUT_PULLUP);
}

ISR(TIMER1_COMPA_vect)
{
  digitalWrite(motStepPin, !digitalRead(motStepPin)); // تولید پالس
}

void loop()
{

  // Update the OLED display
  updateDataDisplay();

  // Read rotary
  roteryread();

  // program input state
  Input();

  if (!Start_Stop)
  {
    // Temperature set
    HotEnd();
    if (Motor_status)
    {
      Motor_status = false;
      noInterrupts();
      TCCR1A = 0;
      TCCR1B = 0;
      TCNT1 = 0;
      TCNT1 = 0;               // ریست تایمر
      OCR1A = ocrValue;        // تنظیم مقدار OCR1A
      TCCR1B |= (1 << WGM12);  // حالت CTC (Clear Timer on Compare Match)
      TCCR1B |= (1 << CS10);   // بدون پیش‌تقسیم‌کننده (Prescaler = 1)
      TIMSK1 |= (1 << OCIE1A); // فعال کردن اینتراپت Compare Match
      interrupts();
    }
  }

  // // Check for input on the pushbutton
  // while (millis() < currentTime + loopTime)
  // {
  //   encCurrent = (myEnc.read() / 4);
  //   byte buttonState = digitalRead(Push);
  //   if (buttonState != oldButtonState)
  //   {
  //     // Serial.println("Button change");
  //     if (millis() - buttonPressTime >= debounceTime) // Debounce button
  //     {
  //       buttonPressTime = millis();   // Time when button was pushed
  //       oldButtonState = buttonState; // Remember button state for next time
  //       if (buttonState == LOW)
  //       {
  //         pressed = true;
  //         // Serial.println("Button Pressed");
  //       }
  //       else
  //       {
  //         if (pressed == true) // Confirm the input once the button is released again
  //         {
  //           pressed = false;
  //           // Serial.println("Button Released");
  //           if (dataInputNo == 0) // Set which parameter is being edited and define limits
  //           {
  //             dataInputNo = 1;
  //             encCurrent = motSpeed;
  //             encLowLim = minSpeed;
  //             encHighLim = maxSpeed;
  //           }
  //           else if (dataInputNo == 1)
  //           {
  //             dataInputNo = 2;
  //             encCurrent = motDir;
  //             encLowLim = minMot;
  //             encHighLim = maxMot;
  //           }
  //           else
  //           {
  //             dataInputNo = 0;
  //             encCurrent = setpoint;
  //             encLowLim = minTemp;
  //             encHighLim = maxTemp;
  //           }
  //         }
  //       }
  //     }
  //   }
  //   // Set the parameter being edited equal to the current encoder position
  //   if (dataInputNo == 0)
  //   {
  //     setpoint = encCurrent + initialTemp;
  //   }
  //   else if (dataInputNo == 1)
  //   {
  //     motSpeed = encCurrent + initialSpeed;
  //     // motDelayTime = 100 * (1 + maxSpeed - motSpeed);
  //     myStepper.setSpeed(motSpeed);
  //   }
  //   else
  //   {
  //     motDir = encCurrent;
  //   }
  //   // Set the motor direction
  //   if (motDir == 0)
  //   {
  //     // pinMode(enablePin, OUTPUT);   // Enable motor
  //     // digitalWrite(motDirPin, LOW); // Reverse motor direction
  //     myStepper.moveTo(-1);
  //     updateDataDisplay();
  //   }
  //   else if (motDir == 2)
  //   {
  //     // pinMode(enablePin, OUTPUT);    // Enable motor
  //     // digitalWrite(motDirPin, HIGH); // Forward motor direction
  //     myStepper.moveTo(1);
  //     updateDataDisplay();
  //   }
  //   else
  //   {
  //     // pinMode(enablePin, INPUT); // Disable motor
  //     myStepper.disableOutputs();
  //     updateDataDisplay();
  //   }
  //   // Pulse the stepper motor if forward or reverse is selected
  //   if (motDir != 1)
  //   {
  //     while (true)
  //     {
  //       encCurrent = (myEnc.read());
  //       if (encCurrent >= 2)
  //       {
  //         encCurrent = 2;
  //       }
  //       else if (encCurrent <= 0)
  //       {
  //         encCurrent = 0;
  //       }
  //       if (oldencCurrent != encCurrent)
  //       {
  //         oldencCurrent = encCurrent;
  //         myEnc.write(encCurrent);
  //       }
  //       byte buttonState = digitalRead(Push);
  //       if (buttonState != oldButtonState && millis() - buttonPressTime >= debounceTime)
  //       {
  //         buttonPressTime = millis();
  //         oldButtonState = buttonState;
  //         encCurrent = 0;
  //         myEnc.write(encCurrent);
  //       }
  //       motDir = encCurrent;
  //       runMotor();
  //       updateDataDisplay();
  //     }
  //   }
  // }
}

void HotEnd()
{
  // Read the temperature
  temp = thermistor.read(); // read temperature

  // Compute the PID output
  pid.Compute();

  // Update the PWM output
  analogWrite(pwmPin, output);
}

// Update the OLED display contents
void updateDataDisplay()
{

  // write constants :

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
  display.setCursor(97, 20);
  display.print((int)temp);
  display.print(F("C"));
  display.setCursor(97, 30);
  display.print((int)setpoint);
  display.print(F("C"));
  display.setCursor(97, 40);
  display.print(motSpeed);

  // write deffernces :

  switch (state_program)
  {
  case first_Menu:

    display.setCursor(87, 30);
    display.print(F(">"));
    display.setCursor(97, 50);
    break;

  case secend_Menu:

    display.setCursor(87, 40);
    display.print(F(">"));
    display.setCursor(97, 50);
    break;

  case third_Menu:

    display.setCursor(87, 50);
    display.print(F(">"));
    display.setCursor(97, 50);
    break;

  default:
    break;
  }

  switch (motDir)
  {
  case 0:
    display.print(F("OFF"));
    display.display();
    break;
  case 1:
    display.print(F("Fw"));
    display.display();
    break;
  case -1:
    display.print(F("Rv"));
    display.display();
    break;
  default:
    break;
  }
}

// Read Rotary Enc
void roteryread()
{

  // key
  byte buttonState = digitalRead(Push);

  if (buttonState != oldButtonState && millis() - buttonPressTime >= debounceTime)
  {

    if (buttonState == LOW && Start_Stop)
    {
      Serial.print("Push");
      state_program++;
      if (state_program >= 3)
      {
        state_program = 0;
      }
      if (state_program < 0)
      {
        state_program = 2;
      }
      if (state_program == 0)
      {
        Data_input = setpoint;
      }
      if (state_program == 1)
      {
        Data_input = motSpeed;
      }
      if (state_program == 2)
      {
        Data_input = motDir;
      }
    }
    if (buttonState == HIGH)
    {
      if (millis() - buttonPressTime >= 3000)
      {
        if (!Start_Stop)
        {
          TCCR1B &= ~(1 << CS12); // turn off the clock altogether
          TCCR1B &= ~(1 << CS11);
          TCCR1B &= ~(1 << CS10);
          TIMSK1 &= ~(1 << OCIE1A); 
          Serial.print("sdfsdf");
        }
        else
        {
          Serial.print("hghg");
          Motor_status = true;
        }

        Start_Stop = !Start_Stop;
        Serial.print(Start_Stop);
      }
    }

    buttonPressTime = millis();
    oldButtonState = buttonState;
  }

  // positon
  if (Start_Stop)
  {
    long newPos = 0;

    for (size_t i = 0; i < 50; i++)
    {
      newPos += myEnc.read();
    }

    newPos = newPos / 130;

    if (newPos > position)
    {
      position = newPos;
      Serial.println(position);
      Data_input++;
    }
    else if (newPos < position)
    {
      position = newPos;
      Serial.println(position);
      Data_input--;
    }
  }
}

void Input()
{
  switch (state_program)
  {
  case 0:
    setpoint = Data_input;
    break;
  case 1:
  {
    if (Data_input > 100)
    {
      Data_input = 100;
    }
    if (Data_input < 0)
    {
      Data_input = 0;
    }

    unsigned long stepFrequency = map(Data_input, 0, 100, minFrequency, maxFrequency);

    ocrValue = (16000000UL / (stepFrequency * 2)) - 1;

    // اعمال محدودیت‌ها
    if (ocrValue < minOCRValue)
    {
      ocrValue = minOCRValue;
      Serial.println("Warning: OCR1A value too low. Capped at minimum value.");
    }
    else if (ocrValue > maxOCRValue)
    {
      ocrValue = maxOCRValue;
      Serial.println("Warning: OCR1A value too high. Capped at maximum value.");
    }
    motSpeed = Data_input;
    break;
  }
  case 2:
    if (Data_input > 1)
    {
      Data_input = 1;
    }
    else if (Data_input < -1)
    {
      Data_input = -1;
    }
    motDir = Data_input;
  default:
    break;
  }
}