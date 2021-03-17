#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <ClosedCube_HDC1080.h>
#include <semphr.h>
#include <queue.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Stepper.h>
#include "SevSegNum.h"
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 24
#define NUM_LEDS 4
#define BRIGHTNESS 25

#define BUTTON1 2
#define BUTTON2 28
#define BUTTON3 29

#define SevenSegCC1 44
#define SevenSegCC2 46

#define SevenSegA 4
#define SevenSegB 5
#define SevenSegC 6
#define SevenSegD 7
#define SevenSegE 8
#define SevenSegF 9
#define SevenSegG 10
#define SevenSegDP 11

#define DIP1 53 // DIP starts at 1 instead of 0 so it matches the dip switch board for clarity
#define DIP2 51
#define DIP3 49
#define DIP4 47
#define DIP5 45
#define DIP6 43
#define DIP7 41
#define DIP8 39
//#define FRAME_RATE 1

#define FRAME_RATE 2/3

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel single = Adafruit_NeoPixel(1, PIN, NEO_GRBW + NEO_KHZ800);

byte neopix_gamma[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

// task prototypes
void vSevSegDisplay(void *pvParameters);
void vDipSwitch(void *pvParameters);
void vMoveStepper(void *pvParameters);
void vPixelCommands(void *pvParameters);


// function prototypes
void sevSegNumbers(int);
void printSevSeg(int, int, int);
void segManager(int, int);
void checkQueueIsFull(int);
void displayPixelCommand(int, int);
void displayPixel(int, int);
int pixelCommand(int);
int pixelManager(int);
int rainbowCycle(uint8_t);
int pulseWhite(uint8_t);
void colorWipe();
uint32_t Wheel(byte);
uint8_t red(uint32_t);
uint8_t green(uint32_t);
uint8_t blue(uint32_t);



QueueHandle_t leftDigQueue = 0;
QueueHandle_t rightDigQueue = 0;
QueueHandle_t tempOrHumQueue = 0;
QueueHandle_t stepperQueue = 0;
QueueHandle_t pixelCommandQueue = 0;

SemaphoreHandle_t xBinarySemaphore;

TaskHandle_t LeftTask_Handle;
TaskHandle_t RightTask_Handle;

ClosedCube_HDC1080 hdc1080;

Stepper step_motor(2048, 24, 28, 26, 30);


void setup() {

  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000)clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();

  // configures button
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);

  // configures pins to be used as outputs for 7 seg display
  pinMode(SevenSegA, OUTPUT);
  pinMode(SevenSegB, OUTPUT);
  pinMode(SevenSegC, OUTPUT);
  pinMode(SevenSegD, OUTPUT);
  pinMode(SevenSegE, OUTPUT);
  pinMode(SevenSegF, OUTPUT);
  pinMode(SevenSegG, OUTPUT);
  pinMode(SevenSegDP, OUTPUT);

  // configures pins to be used as outputs to turn on each digit of 7 seg display
  pinMode(SevenSegCC1, OUTPUT); // right digit
  pinMode(SevenSegCC2, OUTPUT); // left digit

  // configures dip switch pins
  pinMode(DIP1, INPUT);
  pinMode(DIP2, INPUT);
  pinMode(DIP3, INPUT);
  pinMode(DIP4, INPUT);
  pinMode(DIP5, INPUT);
  pinMode(DIP6, INPUT);
  pinMode(DIP7, INPUT);
  pinMode(DIP8, INPUT);

  Serial.begin(9600);

  // initialize HDC1080
  hdc1080.begin(0x40);

   // put your setup code here, to run once:
  while(!Serial)
  {
    ;
  }

  // step motor set to fast speed.
  step_motor.setSpeed(1000);

  leftDigQueue = xQueueCreate(5, sizeof (int)); // queues initialized to hold 1 value
  rightDigQueue = xQueueCreate(5, sizeof (int));
  stepperQueue = xQueueCreate(2, sizeof (int));
  pixelCommandQueue = xQueueCreate(4, sizeof (int));

  xBinarySemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xBinarySemaphore);

  xTaskCreate(vDipSwitch, "Dip", 512, NULL, 3, NULL); 
  xTaskCreate(vSevSegDisplay, "Display", 128, NULL, 1, &LeftTask_Handle);
  xTaskCreate(vMoveStepper, "Stepper", 1024, NULL, 1, NULL);
  xTaskCreate(vPixelCommands, "Pixels", 256, NULL, 4, NULL);

  vTaskStartScheduler();
}

void loop() {
  //not needed in RTOS
}

/*********************************************************
 * void vDipSwitch(void *pvParameters)
 * 
 * This function runs the dip switch 
 * *******************************************************/
void vDipSwitch(void *pvParameters)
{
  (void) pvParameters;

  int stepCount = 0;
  int i;
  int x, y;
  int test = 1;
  int state = 0;
  int prevState = -1;

  //delay(5000);
  // display hex numbers on sevSegment Display
  /*for(i = 0; i < 256; i++)
  {
    x = i / 16;
    y = i % 16;
    segManager(x,y);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }*/
  i = 0;
  // check dip switches 1,2,3,4
  for(;;)
  {
    BREAKRAINBOW: if(digitalRead(DIP5) == LOW)
    {
      if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == LOW) && digitalRead(DIP5) == LOW)
      {
        state = 0;
        if(state == prevState)
        {
          // do nothing
        }
        else
        {
          i = 0;
          int temp1 = 0;
          int temp2 = 0;

          segManager(0,19); // sets display digits
          // checkQueueIsFull(test); // checks if queue is full or not

          temp1 = hdc1080.readTemperature();
          vTaskDelay(500 / portTICK_PERIOD_MS);
          temp2 = hdc1080.readTemperature();
          if(temp1 > temp2) // don't do anything if temp if different
          {
            Serial.print("T=");
            Serial.print(temp1); // prints collected temp to serial monitor
            Serial.println("C");
          }
          else if(temp1 < temp2) // don't do anything if temp is different
          {
            Serial.print("T=");
            Serial.print(temp1); // prints collected temp to serial monitor
            Serial.println("C");
          }
          else // print if temp is different
          {
            Serial.print("T=");
            Serial.print(temp1); // prints collected temp to serial monitor
            Serial.println("C");
            while(i <= temp1)
            {
              stepCount = -1;
              xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
              taskYIELD();

              xSemaphoreGive(xBinarySemaphore);
              vTaskDelay(1);
              xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
              i++;
            }
          }
          prevState = state;
          Serial.println(state);
        }
        // (0,0,0,0)
      }
      else if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == HIGH))
      {
        segManager(5, 19);
        checkQueueIsFull(test);
        Serial.println("Stop");

        // DO NOTHING 
        // (0,0,0,1)
      }
      else if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == HIGH) && (digitalRead(DIP4) == LOW))
      {
        segManager(3, 16);
        while(i <= 2048 && digitalRead(DIP1) == LOW && digitalRead(DIP2) == LOW && digitalRead(DIP3) == HIGH && digitalRead(DIP4) == LOW)
        {
          stepCount = -1; // stepper motor direction CCW
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY); // sends direction to motor task
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        i=0;
        // (0,0,1,0)
      }
      else if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == HIGH) && (digitalRead(DIP4) == HIGH))
      {
        segManager(5, 19);
        checkQueueIsFull(test);
        Serial.println("Stop");

        // DO NOTHING 
        // (0,0,1,1)
      }
      else if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == HIGH) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == LOW))
      {
        segManager(2,17);
        while(i <= 2048 && digitalRead(DIP1) == LOW && digitalRead(DIP2) == HIGH && digitalRead(DIP3) == LOW && digitalRead(DIP4) == LOW)
        {
          stepCount = 1;
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        i = 0;
        // (0,1,0,0)
      }
      else if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == HIGH) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == HIGH))
      {
        segManager(5,19);
        checkQueueIsFull(test);
        Serial.println("Stop");
        
        // DO NOTHING 
        // (0,1,0,1)
      }
      else if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == HIGH) && (digitalRead(DIP3) == HIGH) && (digitalRead(DIP4) == LOW))
      {
        //Serial.println("Move CW then CCW"); 
        //checkQueueIsFull(test);
        segManager(4, 17);
        while(i <= 2048 && digitalRead(DIP1) == HIGH && digitalRead(DIP2) == LOW && digitalRead(DIP3) == LOW && digitalRead(DIP4) == LOW)
        {
          stepCount = 1;
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        i = 0;
        segManager(4, 16);

        while(i <= 2048 && digitalRead(DIP1) == LOW && digitalRead(DIP2) == HIGH && digitalRead(DIP3) == HIGH && digitalRead(DIP4) == LOW)
        {
          stepCount = -1;
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        i = 0;
        // (0,1,1,0)
      }
      else if((digitalRead(DIP1) == LOW) && (digitalRead(DIP2) == HIGH) && (digitalRead(DIP3) == HIGH) && (digitalRead(DIP4) == HIGH))
      {
        segManager(5,19);
        checkQueueIsFull(test);
        Serial.println("Stop");
        // DO NOTHING 
        // (0,1,1,1)
      }
      else if((digitalRead(DIP1) == HIGH) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == LOW))
      {
          int hum1 = 0;
          int hum2 = 0;
          i = 0;

          segManager(1, 17);
          checkQueueIsFull(test);
          
          hum1 = hdc1080.readHumidity();
          
          vTaskDelay(500 / portTICK_PERIOD_MS); // delay between readings to find note change in temps

          hum2 = hdc1080.readHumidity();

          if(hum2 > hum1 + 2) // move on humidity change
          {
            Serial.print("RH=");
            Serial.print(hum1);
            Serial.println("%");
            while(i <= hum1)
            {
              stepCount = 1;
              xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
              taskYIELD();

              xSemaphoreGive(xBinarySemaphore);
              vTaskDelay(1);
              xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
              i++;
            }
          }
          else if(hum2 < hum1 - 2) // move on humidity change
          {
            Serial.print("RH=");
            Serial.print(hum1);
            Serial.println("%");
            while(i <= hum1)
            {
              //Serial.println("check if it gets to the while loop");
              stepCount = 1;
              xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
              taskYIELD();

              xSemaphoreGive(xBinarySemaphore);
              vTaskDelay(1);
              xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
              i++;
            }
          }
          else // don't move on humidity change
          {
            Serial.print("RH=");
            Serial.print(hum1);
            Serial.println("%");
          }
        i = 0;
        // (1,0,0,0)
      }
      else if((digitalRead(DIP1) == HIGH) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == HIGH))
      {
        segManager(5,19);
        checkQueueIsFull(test);
        Serial.println("Stop");
        // DO NOTHING 
        // (1,0,0,1)
      }
      else if((digitalRead(DIP1) == HIGH) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == HIGH) && (digitalRead(DIP4) == LOW))
      { 
        segManager(3, 16);
        while(i <= 2048 && digitalRead(DIP1) == HIGH && digitalRead(DIP2) == LOW && digitalRead(DIP3) == HIGH && digitalRead(DIP4) == LOW)
        {
          stepCount = -1;
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        i = 0;
        // (1,0,1,0)
      }
      else if((digitalRead(DIP1) == HIGH) && (digitalRead(DIP2) == LOW) && (digitalRead(DIP3) == HIGH) && (digitalRead(DIP4) == HIGH))
      {
        segManager(5, 19);
        checkQueueIsFull(test);
        Serial.println("Stop");
        // DO NOTHING 
        // (1,0,1,1)
      }
      else if((digitalRead(DIP1) == HIGH) && (digitalRead(DIP2) == HIGH) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == LOW))
      {
        segManager(2, 17);
        while(i <= 2048 && digitalRead(DIP1) == HIGH && digitalRead(DIP2) == HIGH && digitalRead(DIP3) == LOW && digitalRead(DIP4) == LOW)
        {
          //Serial.println("check if it gets to the while loop");
          stepCount = 1;
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        i = 0;
        // (1,1,0,0)
      }
      else if((digitalRead(DIP1) == HIGH) && (digitalRead(DIP2) == HIGH) && (digitalRead(DIP3) == LOW) && (digitalRead(DIP4) == HIGH))
      {
        segManager(5,19);
        checkQueueIsFull(test);
        Serial.println("Stop");
        // DO NOTHING 
        // (1,1,0,1)
      }
      else if((digitalRead(DIP1) == HIGH) && (digitalRead(DIP2) == HIGH) && (digitalRead(DIP3) == HIGH) && (digitalRead(DIP4) == LOW))
      { 
        //checkQueueIsFull(test);
        i = 0;
        segManager(4, 17);

        while (i <= 2048 && digitalRead(DIP1) == HIGH && digitalRead(DIP2) == HIGH && digitalRead(DIP3) == HIGH && digitalRead(DIP4) == LOW)
        {
          stepCount = 1;
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        segManager(4, 16);
        i = 0;
        while(i <= 2048 && digitalRead(DIP1) == HIGH && digitalRead(DIP2) == HIGH && digitalRead(DIP3) == HIGH && digitalRead(DIP4) == LOW)
        {
          stepCount = -1;
          xQueueSend(stepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();

          xSemaphoreGive(xBinarySemaphore);
          vTaskDelay(1);
          xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
          i++;
        }
        i = 0;
        // (1,1,1,0)
      }
      else
      {
        segManager(5,19);
        checkQueueIsFull(test);
        Serial.println("Stop");
        // DO NOTHING 
        // (1,1,1,1)
      }
    }else
    { 
      int lastState = LOW;
      int lastState2 = LOW;
      int lastState3 = LOW;
      int currentState, currentState2, currentState3;
      int counter; //= 0;
      currentState = digitalRead(BUTTON1);
      currentState3 = digitalRead(BUTTON3);
      if(currentState == LOW) // button1 not pressed goes into regular routine
      {
        // sets all LED's to red.
        // on button 3 press, shows individual control of LED's
        if(digitalRead(DIP6) == LOW && digitalRead(DIP7) == LOW && digitalRead(DIP8) == LOW) // dips set to 0, 0, 0 
        {
          state = 2;
          if(state == prevState)
          {
            // do nothing
          }
          else
          {
            if(currentState3 == HIGH)
            {
              pixelManager(7);
              vTaskDelay(500 / portTICK_PERIOD_MS);
              pixelManager(8);
              vTaskDelay(500 / portTICK_PERIOD_MS);
              pixelManager(9);
              vTaskDelay(500 / portTICK_PERIOD_MS);
              pixelManager(0);
              vTaskDelay(500 / portTICK_PERIOD_MS);
              pixelManager(9);
              vTaskDelay(500 / portTICK_PERIOD_MS);
              pixelManager(8);
              vTaskDelay(500 / portTICK_PERIOD_MS);
              pixelManager(7);
              vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            else
            {
              pixelManager(0);
            }
          }
          prevState = state;
          //Serial.print("State ");
          //Serial.println(state);
          //Serial.print("PrevState ");
          //Serial.println(prevState);
    
          //Serial.println(counter);
        }
        // sets all LED's to green
        else if(digitalRead(DIP6) == LOW && digitalRead(DIP7) == LOW && digitalRead(DIP8) == HIGH) // 0, 0, 1
        {
          pixelManager(1);
        }
        // sets all LEDS to blue
        else if(digitalRead(DIP6) == LOW && digitalRead(DIP7) == HIGH && digitalRead(DIP8) == LOW) // 0, 1, 0
        {
          pixelManager(2);
        }
        // sets all LEDS to white
        else if(digitalRead(DIP6) == LOW && digitalRead(DIP7) == HIGH && digitalRead(DIP8) == HIGH) // 0, 1, 1
        {
          pixelManager(3);
        }
        // sets LED's to different colors
        else if(digitalRead(DIP6) == HIGH && digitalRead(DIP7) == LOW && digitalRead(DIP8) == LOW) // 1, 0, 0
        {
          pixelManager(4);
        }
        // sets each LED brightness unique
        else if(digitalRead(DIP6) == HIGH && digitalRead(DIP7) == LOW && digitalRead(DIP8) == HIGH) // 1, 0, 1
        {
          pixelManager(5);
        }
        // sets pulse white
        else if(digitalRead(DIP6) == HIGH && digitalRead(DIP7) == HIGH && digitalRead(DIP8) == LOW) // 1, 1, 0
        {
          //pixelManager(6);
          if(pixelManager(11) == 1)
          {
            goto BREAKRAINBOW; // YIKES. Rework code to get rid of goto. its late so i don't feel like it right now. 
          }
        }
        // rainbow command
        else // 1, 1, 1
        {
          if(pixelManager(6) == 1)
          {
            goto BREAKRAINBOW; // YIKES. Rework code to get rid of goto. its late so i don't feel like it right now.
          }
        }
      }
      else // button1 is HIGH so blanks screen for a short time and resets to whatever the switch is set to.
      {
        while(currentState == HIGH)
        {
          pixelManager(10);
          lastState = currentState;
          vTaskDelay(500 / portTICK_PERIOD_MS);
          currentState = digitalRead(BUTTON1);
        }
      }
      
    } state = prevState;
  }
}

/*******************************************************
 * void vSevSegDisplay(void *pvParameters)
 * 
 *  Task to display digits on seven seg display 
 * ****************************************************/
void vSevSegDisplay(void *pvParameters)
{
  (void) pvParameters;
  int num = 0, num2 = 0;
  for(;;)
  {
    Serial.println("Sev Seg");
    xQueueReceive(leftDigQueue, &num, portMAX_DELAY); // receives left digit from queue
    xSemaphoreTake(xBinarySemaphore,portMAX_DELAY);
    printSevSeg(SevenSegCC2, SevenSegCC1, num);
    xQueueReceive(rightDigQueue, &num2, portMAX_DELAY); // receives right digit from queue
    printSevSeg(SevenSegCC1, SevenSegCC2, num2);
    xSemaphoreGive(xBinarySemaphore);
    Serial.println("SevSeg Gave Semaphore");
  }
}

/****************************************************
 * void vMoveStepper(void *pvParameters)
 * 
 *  Task to move stepper motor
 * *************************************************/
void vMoveStepper(void *pvParameters)
{
  (void) pvParameters;
  int num_steps = 0;
  for(;;)
  {
    Serial.println("Stepper");
    if(!xQueueReceive(stepperQueue, &num_steps, portMAX_DELAY))
    {
      Serial.println("Task Not received.");
    }
    step_motor.step(num_steps); //move stepper number of steps
    taskYIELD();
  }
}

/***************************************************
 * void vPixelCommands(void *pvParameters)
 *
 *  Task to give commands to display pixel colors
 * *************************************************/
void vPixelCommands(void *pvParameters)
{
  (void) pvParameters;
  int command = 0;
  for(;;)
  {
    //Serial.println("Pixels");
    if(!xQueueReceive(pixelCommandQueue, &command, portMAX_DELAY))
    {
      Serial.println("Pixel Queue not receiving");
    }
    Serial.println("Pixels");
    displayPixelCommand(PIN, command);
    //xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
    //displayPixelCommand(PIN, command);
    //xSemaphoreGive(xBinarySemaphore);
    //Serial.println("Pixel gave semaphore");
    //taskYIELD();
  }
}

// function to display pixel Command
void displayPixelCommand(int pixFlag, int command) // display command
{
  //digitalWrite(pixFlag, HIGH);
  pixelCommand(command);
  vTaskDelay(FRAME_RATE);
  //digitalWrite(pixFlag, HIGH);
}

// function to manage pixel commands
int pixelManager(int pix)
{
  xQueueSend(pixelCommandQueue, &pix, portMAX_DELAY);
  //taskYIELD();
}

// function to change pixels
int pixelCommand(int command)
{
  switch(command)
  {
    case 0: // display all red
      strip.setPixelColor(0, 255, 0, 0, 0);
      strip.setPixelColor(1, 255, 0, 0, 0);
      strip.setPixelColor(2, 255, 0, 0, 0);
      strip.setPixelColor(3, 255, 0, 0, 0);
      strip.show();
      return 0;
      break;
    case 1: // display all green
      strip.setPixelColor(0, 0, 255, 0, 0);
      strip.setPixelColor(1, 0, 255, 0, 0);
      strip.setPixelColor(2, 0, 255, 0, 0);
      strip.setPixelColor(3, 0, 255, 0, 0);
      strip.show();
      return 0;
      break;
    case 2: // display all blue
      strip.setPixelColor(0, 0, 0, 255, 0);
      strip.setPixelColor(1, 0, 0, 255, 0);
      strip.setPixelColor(2, 0, 0, 255, 0);
      strip.setPixelColor(3, 0, 0, 255, 0);
      strip.show();
      return 0;
      break;
    case 3: // display all white
      strip.setPixelColor(0, 0, 0, 0, 255);
      strip.setPixelColor(1, 0, 0, 0, 255);
      strip.setPixelColor(2, 0, 0, 0, 255);
      strip.setPixelColor(3, 0, 0, 0, 255);
      strip.show();
      return 0;
      break;
    case 4: // display pix 0 -> red, pix1 -> green, pix2 -> blue, pix3 -> white for a short amount of time then repeat
      strip.setPixelColor(0, 255, 0, 0, 0);
      strip.setPixelColor(1, 0, 255, 0, 0);
      strip.setPixelColor(2, 0, 0, 255, 0);
      strip.setPixelColor(3, 0, 0, 0, 255);
      strip.show();
      return 0;
      break;
    case 5: // display individual pixel brightness
      strip.setPixelColor(0, 63, 0, 0 , 0);
      strip.setPixelColor(1, 128, 0, 0, 0);
      strip.setPixelColor(2, 191, 0, 0, 0);
      strip.setPixelColor(3, 255, 0, 0, 0);
      strip.show();
      return 0;
      break;
    case 6: // display rainbow affect
      //rainbow(1);
      if(rainbowCycle(1) == 1) // case where another dip is turned
      {
        return 1;
      }
      return 0;
      break;
    case 7:
      strip.setPixelColor(0, 255, 0, 0, 0);
      strip.setPixelColor(1, 0, 0, 0, 0);
      strip.setPixelColor(2, 0, 0, 0, 0);
      strip.setPixelColor(3, 0, 0, 0, 0);
      strip.show();
      return 0;
      break;
    case 8:
      strip.setPixelColor(0, 255, 0, 0, 0);
      strip.setPixelColor(1, 255, 0, 0, 0);
      strip.setPixelColor(2, 0, 0, 0, 0);
      strip.setPixelColor(3, 0, 0, 0, 0);
      strip.show();
      return 0;
      break;
    case 9:
      strip.setPixelColor(0, 255, 0, 0, 0);
      strip.setPixelColor(1, 255, 0, 0, 0);
      strip.setPixelColor(2, 255, 0, 0, 0);
      strip.setPixelColor(3, 0, 0, 0, 0);
      strip.show();
      return 0;
      break;
    case 10:
      colorWipe();
      return 0;
      break;
    case 11:
      if(pulseWhite(1) == 1)
      {
        return 1;
      }
      return 0;
      break;
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    vTaskDelay(wait); //////////////////////////////
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85)
  {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3, 0);
  }
  if(WheelPos < 170)
  {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}

// Slightly different, this makes the rainbow equally distributed throughout
int rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  strip.begin();
  for(j=0; j<256; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      if(digitalRead(BUTTON1) == HIGH) return 0;
      if(digitalRead(DIP6) == LOW || digitalRead(DIP7) == LOW || digitalRead(DIP8) == LOW) return 1;
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
      //strip.show();
      //taskYIELD();
    }
    strip.show();
    vTaskDelay(wait);
  }
}

// Fill the dots one after the other with a color
void colorWipe() {
  for(uint16_t i = 0; i < strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
}

int pulseWhite(uint8_t wait) {
  for(int j = 0; j < 256 ; j++){
      for(uint16_t i=0; i<strip.numPixels(); i++) {
          if(digitalRead(BUTTON1) == HIGH) return 0;
          if(digitalRead(DIP6) == LOW || digitalRead(DIP7) == LOW || digitalRead(DIP8) == HIGH) return 1;
          strip.setPixelColor(i, strip.Color(0,0,0, neopix_gamma[j] ) );
        }
        vTaskDelay(wait);
        strip.show();
      }

  for(int j = 255; j >= 0 ; j--){
      for(uint16_t i=0; i<strip.numPixels(); i++) {
          if(digitalRead(BUTTON1) == HIGH) return 0;
          if(digitalRead(DIP6) == LOW || digitalRead(DIP7) == LOW || digitalRead(DIP8) == HIGH) return 1;
          strip.setPixelColor(i, strip.Color(0,0,0, neopix_gamma[j] ) );
        }
        vTaskDelay(wait);
        strip.show();
      }
}

uint8_t red(uint32_t c)
{
  return (c >> 16);
}

uint8_t green(uint32_t c)
{
  return (c >> 8);
}

uint8_t blue(uint32_t c)
{
  return (c);
}

// function to switch between digits
void sevSegNumbers(int num)
{
  switch(num) // switches received queue data to display whatever left dig needed
  {
    case 0:
      printNum0on();
      printNum0off();
      break;
    case 1:
      printNum1on();
      printNum1off();
      break;
    case 2:
      printNum2on();
      printNum2off();
      break;
    case 3:
      printNum3on();
      printNum3off();
      break;
    case 4:
      printNum4on();
      printNum4off();
      break;
    case 5:
      printNum5on();
      printNum5off();
      break;
    case 6:
      printNum6on();
      printNum6off();
      break;
    case 7:
      printNum7on();
      printNum7off();
      break;
    case 8:
      printNum8on();
      printNum8off();
      break;
    case 9:
      printNum9on();
      printNum9off();
      break;
    case 10:          // A
      printNumAon();
      printNumAoff();
      break;
    case 11:          // B
      printNumBon();
      printNumBoff();
      break;
    case 12:          // C
      printNumCon();
      printNumCoff();
      break;
    case 13:          // D
      printNumDon();
      printNumDoff();
      break;
    case 14:          // E
      printNumEon();
      printNumEoff();
      break;
    case 15:          // F
      printNumFon();
      printNumFoff();
      break;
    case 16:          // L
      printNumLon();
      printNumLoff();
      break;
    case 17:          // R
      printNumRon();
      printNumRoff();
      break;
    case 18:          // dp
      printDPon();
      printDPoff();
      break;
    case 19:
      printDashon();
      printDashoff();
      break;
  }
}

// function to print digits to seven seg display
void printSevSeg(int side, int off, int num)
{
  digitalWrite(off, HIGH);
  digitalWrite(side, LOW);
  sevSegNumbers(num);
  vTaskDelay(FRAME_RATE);
  sevSegNumbers(num);
}

// function manages 7 seg Display
void segManager(int lNum, int rNum)
{
  xQueueSend(leftDigQueue, &lNum, portMAX_DELAY);
  xQueueSend(rightDigQueue, &rNum, portMAX_DELAY);
}

// function checks if queue is full
void checkQueueIsFull(int test)
{
  int lDig = 0;
  int rDig = 0;
  if(test == 0) // resets if test is 0
  {
    for(int i = 0; i == 255; i++)
    {
      lDig = i / 16;
      rDig = i % 16;
      segManager(lDig,rDig);
    }
    for(int i = 0; i < 10; i++) // if full, display OF
    {
      segManager(0, 15);
    }
  }
  if(xQueueIsQueueFullFromISR(leftDigQueue) == pdTRUE) // if function is full from ISR, reset queues
  {
    xQueueReset(leftDigQueue);
    xQueueReset(rightDigQueue);
    xSemaphoreGive(xBinarySemaphore);
    segManager(0,15);
    vTaskDelay((1000 / portTICK_PERIOD_MS) * 5);
    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
  }
  else
  {
    xSemaphoreGive(xBinarySemaphore);
    vTaskDelay((1000 / portTICK_PERIOD_MS) * 5);
    xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
  }
}