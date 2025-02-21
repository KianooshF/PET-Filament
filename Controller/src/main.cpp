#include <PID_v1.h>
#include <thermistor.h> // https://github.com/miguel5612/Arduino-ThermistorLibrary
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ticker.h> // برای تایمر نرم‌افزاری در ESP8266
#include <Encoder.h>

//-------------------LCD Define-------------------

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define first_Menu 0
#define secend_Menu 1
#define third_Menu 2
#define fourth_Menu 3
#define five_Menu 4

//-------------------Temp Out/In-------------------

const int temperaturePin = A0; // Define the remaining IO pins for motor, pushbutton & thermistor

THERMISTOR thermistor(temperaturePin, // Analog pin
                      50,             // Nominal resistance at 25 ºC
                      3100,           // thermistor's beta coefficient
                      50);            // Value of the series resistor

const int pwmPin = D8; // PWM pin on ESP8266

//-------------------Motor Out-------------------

#define initialSpeed 10
#define initialMot 0

const int stepsPerRevolution = 200;      // تعداد گام‌ها در هر دور
const unsigned long minFrequency = 80;   // حداقل فرکانس پالس‌ها (۱۰۰ هرتز)
const unsigned long maxFrequency = 3000; // حداکثر فرکانس پالس‌ها (۱۰ کیلوهرتز)

const int motDirPin = D3;  // DIR pin on ESP8266
const int motStepPin = D4; // STEP pin on ESP8266

unsigned int ocrValue = 0;

int motSpeed = initialSpeed; // Define motor parameters
int motDir = initialMot;

volatile bool stepState = false;
unsigned long stepInterval = 8000; // مقدار اولیه برای تناسب با سرعت

void stepMotorInterrupt()
{
  digitalWrite(motStepPin, stepState);
  stepState = !stepState;
}
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

#define Clock D7 // Clock pin connected to D5
#define Data D6  // Data pin connected to D6
#define Push D5  // Push button pin connected to D7

long position = 0;
bool flag_change = false;

Encoder myEnc(Data, Clock);

byte oldButtonState = HIGH; // First button state is open because of pull-up resistor

//-------------------System-------------------

int state_program = 0;
int Data_input = initialTemp;

bool Start_Stop = true;
bool Motor_status = true;

//-------------------Bottun-------------------

const unsigned long debounceTime = 10; // Debounce delay time
unsigned long buttonPressTime;         // Time button has been pressed for debounce

Ticker stepTicker; // تایمر نرم‌افزاری برای تولید پالس‌های STEP

void updateDataDisplay();
void HotEnd();
void roteryread();
void Input();
void stepMotor();

void setup()
{
  Serial.begin(115200); // Initialize serial communication

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay(); // Clear the buffer

  // pinMode(pwmPin, OUTPUT); // Configure PWM pin

  pid.SetMode(AUTOMATIC); // Set the PID parameters
  pid.SetOutputLimits(0, 250);

  temp = thermistor.read() / 10.0; // Read and set the initial input value

  pinMode(Push, INPUT_PULLUP);

  pinMode(motDirPin, OUTPUT);
  pinMode(motStepPin, OUTPUT);

  timer1_disable();
  timer1_attachInterrupt(stepMotorInterrupt);
  timer1_isr_init();
}

void stepMotor()
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
  pid.Compute();

  if (!Start_Stop)
  {
    // Temperature set
    HotEnd();
    if (Motor_status)
    {
      Motor_status = false;
      unsigned long stepFrequency = map(motSpeed, 0, 100, 8000, 200000); // تبدیل سرعت به فرکانس
      stepInterval = 80000000 / stepFrequency;                           // محاسبه مقدار تایمر
      timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
      timer1_write(stepInterval); // مقدار اولیه تایمر
    }
  }
  else
  {
    timer1_disable(); // توقف تایمر در حالت غیرفعال
  }
}

void HotEnd()
{
  // Read the temperature
  temp = thermistor.read() / 10.0; // read temperature

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
    break;

  case secend_Menu:
    display.setCursor(87, 40);
    break;

  case third_Menu:
    display.setCursor(87, 50);
    break;

  default:
    break;
  }

  display.print(F(">"));
  display.setCursor(97, 50);
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
      buttonPressTime = millis();
    }
    if (buttonState == HIGH) // FIXME
    {
      if (millis() - buttonPressTime >= 2000)
      {
        Motor_status = true;
        Start_Stop = !Start_Stop;
        Serial.println(Start_Stop);
      }
      else
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
    }

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
    if (Data_input > 100)
    {
      Data_input = 100;
    }
    if (Data_input < 0)
    {
      Data_input = 0;
    }
    motSpeed = Data_input;
    break;
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
    digitalWrite(motDirPin, motDir == -1 ? LOW : HIGH);
    break;
  default:
    break;
  }
}