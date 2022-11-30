

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#if defined(__AVR__)
#include <avr/io.h>
#endif
// #include "WProgram.h"
#endif

#if defined(ESP_PLATFORM)
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LOW 0
#define HIGH 1
#define INPUT GPIO_MODE_INPUT
#define OUTPUT GPIO_MODE_OUTPUT

void pinMode(gpio_num_t gpio_num, gpio_mode_t mode){
  gpio_config_t config = {};
  config.pin_bit_mask = (1ULL<<gpio_num);
  config.mode = mode; 
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&config);
}

// #define digitalWrite(pin,level)
// #define pinMode(pin,mode)
#define digitalWrite(pin, level) gpio_set_level(pin, level);
// #define pinMode(pin, mode) gpio_set_direction(pin, mode);
#define _BV(bit) (1 << (bit))
#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS)


#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros()
{
    return (unsigned long) (esp_timer_get_time());
}
void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

#endif

#include "Motor.h"

static uint8_t latch_state;

#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

AFMotorController::AFMotorController(void)
{
  TimerInitalized = false;
}

void AFMotorController::enable(void)
{
  // setup the latch
  /*
  LATCH_DDR |= _BV(LATCH);
  ENABLE_DDR |= _BV(ENABLE);
  CLK_DDR |= _BV(CLK);
  SER_DDR |= _BV(SER);
  */
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  latch_state = 0;

  latch_tx(); // "reset"

  // ENABLE_PORT &= ~_BV(ENABLE); // enable the chip outputs!
  // digitalWrite(MOTORLATCH, LOW);
  digitalWrite(MOTORENABLE, LOW);
  // digitalWrite(MOTORDATA, LOW);
  // digitalWrite(MOTORCLK, LOW);

  #ifdef ESP_PLATFORM
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.timer_num = LEDC_TIMER_0;
  ledc_timer.duty_resolution = LEDC_TIMER_8_BIT;
  ledc_timer.freq_hz = 7000; // Set output frequency at 5 kHz
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
#endif
}

void AFMotorController::latch_tx(void)
{
  uint8_t i;
  printf("latch_state %x\r\n",latch_state);

  // LATCH_PORT &= ~_BV(LATCH);                                                                                                                                                                                 
  digitalWrite(MOTORLATCH, LOW);
  // delayMicroseconds(10);

  // SER_PORT &= ~_BV(SER);
  digitalWrite(MOTORDATA, LOW);
  // delayMicroseconds(10);

  for (i = 0; i < 8; i++)
  {
    // CLK_PORT &= ~_BV(CLK);
    digitalWrite(MOTORCLK, LOW);
    // delayMicroseconds(10);

    if (latch_state & _BV(7 - i))
    {
      // SER_PORT |= _BV(SER);
      digitalWrite(MOTORDATA, HIGH);
      // delayMicroseconds(10);
    }
    else
    {
      // SER_PORT &= ~_BV(SER);
      digitalWrite(MOTORDATA, LOW);
      // delayMicroseconds(10);
    }
    // CLK_PORT |= _BV(CLK);
    digitalWrite(MOTORCLK, HIGH);
    // delayMicroseconds(10);
  }
  // LATCH_PORT |= _BV(LATCH);
  digitalWrite(MOTORLATCH, HIGH);
  // delayMicroseconds(10);

  // digitalWrite(MOTORCLK, HIGH);
  //   delayMicroseconds(10);
}

static AFMotorController MC;

/******************************************
               MOTORS
******************************************/
inline void initPWM1(uint8_t freq)
{
#if !defined(PIC32_USE_PIN9_FOR_M1_PWM) && !defined(PIC32_USE_PIN10_FOR_M1_PWM)
  pinMode(PWM2A, OUTPUT);
#endif

#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer2A on PB3 (Arduino pin #11)
  TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
  TCCR2B = freq & 0x7;
  OCR2A = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 11 is now PB5 (OC1A)
  TCCR1A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc1a
  TCCR1B = (freq & 0x7) | _BV(WGM12);
  OCR1A = 0;
#elif defined(__PIC32MX__)
#if defined(PIC32_USE_PIN9_FOR_M1_PWM)
  // Make sure that pin 11 is an input, since we have tied together 9 and 11
  pinMode(9, OUTPUT);
  pinMode(11, INPUT);
  if (!MC.TimerInitalized)
  {                                        // Set up Timer2 for 80MHz counting fro 0 to 256
    T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
    TMR2 = 0x0000;
    PR2 = 0x0100;
    MC.TimerInitalized = true;
  }
  // Setup OC4 (pin 9) in PWM mode, with Timer2 as timebase
  OC4CON = 0x8006; // OC32 = 0, OCTSEL=0, OCM=6
  OC4RS = 0x0000;
  OC4R = 0x0000;
#elif defined(PIC32_USE_PIN10_FOR_M1_PWM)
  // Make sure that pin 11 is an input, since we have tied together 9 and 11
  pinMode(10, OUTPUT);
  pinMode(11, INPUT);
  if (!MC.TimerInitalized)
  {                                        // Set up Timer2 for 80MHz counting fro 0 to 256
    T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
    TMR2 = 0x0000;
    PR2 = 0x0100;
    MC.TimerInitalized = true;
  }
  // Setup OC5 (pin 10) in PWM mode, with Timer2 as timebase
  OC5CON = 0x8006; // OC32 = 0, OCTSEL=0, OCM=6
  OC5RS = 0x0000;
  OC5R = 0x0000;
#else
  // If we are not using PWM for pin 11, then just do digital
  digitalWrite(11, LOW);
#endif
#elif defined(ESP_PLATFORM)
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num = PWM2A;
  ledc_channel.duty = 0; // Set duty to 0%
  ledc_channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
#else
#error "This chip is not supported!"
#endif
}

inline void setPWM1(uint8_t s)
{
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer2A on PB3 (Arduino pin #11)
  OCR2A = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 11 is now PB5 (OC1A)
  OCR1A = s;
#elif defined(__PIC32MX__)
#if defined(PIC32_USE_PIN9_FOR_M1_PWM)
  // Set the OC4 (pin 9) PMW duty cycle from 0 to 255
  OC4RS = s;
#elif defined(PIC32_USE_PIN10_FOR_M1_PWM)
  // Set the OC5 (pin 10) PMW duty cycle from 0 to 255
  OC5RS = s;
#else
  // If we are not doing PWM output for M1, then just use on/off
  if (s > 127)
  {
    digitalWrite(11, HIGH);
  }
  else
  {
    digitalWrite(11, LOW);
  }
#endif
#elif defined(ESP_PLATFORM)
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, s));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
#else
#error "This chip is not supported!"
#endif
}

inline void initPWM2(uint8_t freq)
{
  pinMode(PWM2B, OUTPUT);

#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer2B (pin 3)
  TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2b
  TCCR2B = freq & 0x7;
  OCR2B = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 3 is now PE5 (OC3C)
  TCCR3A |= _BV(COM1C1) | _BV(WGM10); // fast PWM, turn on oc3c
  TCCR3B = (freq & 0x7) | _BV(WGM12);
  OCR3C = 0;
#elif defined(__PIC32MX__)
  if (!MC.TimerInitalized)
  {                                        // Set up Timer2 for 80MHz counting fro 0 to 256
    T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
    TMR2 = 0x0000;
    PR2 = 0x0100;
    MC.TimerInitalized = true;
  }
  // Setup OC1 (pin3) in PWM mode, with Timer2 as timebase
  OC1CON = 0x8006; // OC32 = 0, OCTSEL=0, OCM=6
  OC1RS = 0x0000;
  OC1R = 0x0000;
#elif defined(ESP_PLATFORM)
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_1;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num = PWM2B;
  ledc_channel.duty = 0; // Set duty to 0%
  ledc_channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
#else
#error "This chip is not supported!"
#endif

  
}

inline void setPWM2(uint8_t s)
{
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer2A on PB3 (Arduino pin #11)
  OCR2B = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 11 is now PB5 (OC1A)
  OCR3C = s;
#elif defined(__PIC32MX__)
  // Set the OC1 (pin3) PMW duty cycle from 0 to 255
  OC1RS = s;
#elif defined(ESP_PLATFORM)
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, s));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
#else
#error "This chip is not supported!"
#endif
}

inline void initPWM3(uint8_t freq)
{
   pinMode(PWM0A, OUTPUT);
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer0A / PD6 (pin 6)
  TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on OC0A
  // TCCR0B = freq & 0x7;
  OCR0A = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 6 is now PH3 (OC4A)
  TCCR4A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc4a
  TCCR4B = (freq & 0x7) | _BV(WGM12);
  // TCCR4B = 1 | _BV(WGM12);
  OCR4A = 0;
#elif defined(__PIC32MX__)
  if (!MC.TimerInitalized)
  {                                        // Set up Timer2 for 80MHz counting fro 0 to 256
    T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
    TMR2 = 0x0000;
    PR2 = 0x0100;
    MC.TimerInitalized = true;
  }
  // Setup OC3 (pin 6) in PWM mode, with Timer2 as timebase
  OC3CON = 0x8006; // OC32 = 0, OCTSEL=0, OCM=6
  OC3RS = 0x0000;
  OC3R = 0x0000;
#elif defined(ESP_PLATFORM)
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_2;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num = PWM0A;
  ledc_channel.duty = 0; // Set duty to 0%
  ledc_channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
#else
#error "This chip is not supported!"
#endif
 
}

inline void setPWM3(uint8_t s)
{
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer0A on PB3 (Arduino pin #6)
  OCR0A = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 6 is now PH3 (OC4A)
  OCR4A = s;
#elif defined(__PIC32MX__)
  // Set the OC3 (pin 6) PMW duty cycle from 0 to 255
  OC3RS = s;
#elif defined(ESP_PLATFORM)
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, s));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
#else
#error "This chip is not supported!"
#endif
}

inline void initPWM4(uint8_t freq)
{
   pinMode(PWM0B, OUTPUT);
   
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer0B / PD5 (pin 5)
  TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01); // fast PWM, turn on oc0a
  // TCCR0B = freq & 0x7;
  OCR0B = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 5 is now PE3 (OC3A)
  TCCR3A |= _BV(COM1A1) | _BV(WGM10); // fast PWM, turn on oc3a
  TCCR3B = (freq & 0x7) | _BV(WGM12);
  // TCCR4B = 1 | _BV(WGM12);
  OCR3A = 0;
#elif defined(__PIC32MX__)
  if (!MC.TimerInitalized)
  {                                        // Set up Timer2 for 80MHz counting fro 0 to 256
    T2CON = 0x8000 | ((freq & 0x07) << 4); // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=<freq>, T32=0, TCS=0; // ON=1, FRZ=0, SIDL=0, TGATE=0, TCKPS=0, T32=0, TCS=0
    TMR2 = 0x0000;
    PR2 = 0x0100;
    MC.TimerInitalized = true;
  }
  // Setup OC2 (pin 5) in PWM mode, with Timer2 as timebase
  OC2CON = 0x8006; // OC32 = 0, OCTSEL=0, OCM=6
  OC2RS = 0x0000;
  OC2R = 0x0000;
#elif defined(ESP_PLATFORM)
  ledc_channel_config_t ledc_channel = {};
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = LEDC_CHANNEL_3;
  ledc_channel.timer_sel = LEDC_TIMER_0;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.gpio_num = PWM0B;
  ledc_channel.duty = 0; // Set duty to 0%
  ledc_channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
#else
#error "This chip is not supported!"
#endif
 
}

inline void setPWM4(uint8_t s)
{
#if defined(__AVR_ATmega8__) ||   \
    defined(__AVR_ATmega48__) ||  \
    defined(__AVR_ATmega88__) ||  \
    defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega328P__)
  // use PWM from timer0A on PB3 (Arduino pin #6)
  OCR0B = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // on arduino mega, pin 6 is now PH3 (OC4A)
  OCR3A = s;
#elif defined(__PIC32MX__)
  // Set the OC2 (pin 5) PMW duty cycle from 0 to 255
  OC2RS = s;
#elif defined(ESP_PLATFORM)
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, s));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
#else
#error "This chip is not supported!"
#endif
}

AF_DCMotor::AF_DCMotor(uint8_t num, uint8_t freq)
{
  motornum = num;
  pwmfreq = freq;

  MC.enable();



  switch (num)
  {
  case 1:
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM1(freq);
    break;
  case 2:
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM2(freq);
    break;
  case 3:
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM3(freq);
    break;
  case 4:
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM4(freq);
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd)
{
  uint8_t a, b;
  switch (motornum)
  {
  case 1:
    a = MOTOR1_A;
    b = MOTOR1_B;
    break;
  case 2:
    a = MOTOR2_A;
    b = MOTOR2_B;
    break;
  case 3:
    a = MOTOR3_A;
    b = MOTOR3_B;
    break;
  case 4:
    a = MOTOR4_A;
    b = MOTOR4_B;
    break;
  default:
    return;
  }

  switch (cmd)
  {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b);
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b);
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a); // A and B both low
    latch_state &= ~_BV(b);
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed)
{
  printf("motor %d speed %d\n",motornum, speed );
  switch (motornum)
  {
  case 1:
    setPWM1(speed);
    break;
  case 2:
    setPWM2(speed);
    break;
  case 3:
    setPWM3(speed);
    break;
  case 4:
    setPWM4(speed);
    break;
  }
}

/******************************************
               STEPPERS
******************************************/

AF_Stepper::AF_Stepper(uint16_t steps, uint8_t num)
{
  MC.enable();

  revsteps = steps;
  steppernum = num;
  currentstep = 0;

  if (steppernum == 1)
  {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
                   ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();

    // enable both H bridges
    pinMode(PWM2A, OUTPUT);
    pinMode(PWM2B, OUTPUT);
    digitalWrite(PWM2A, HIGH);
    digitalWrite(PWM2B, HIGH);

    // use PWM for microstepping support
    initPWM1(STEPPER1_PWM_RATE);
    initPWM2(STEPPER1_PWM_RATE);
    setPWM1(255);
    setPWM2(255);
  }
  else if (steppernum == 2)
  {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
                   ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();

    // enable both H bridges
    pinMode(PWM0B, OUTPUT);
    pinMode(PWM0A, OUTPUT);
    digitalWrite(PWM0B, HIGH);
    digitalWrite(PWM0A, HIGH);

    // use PWM for microstepping support
    // use PWM for microstepping support
    initPWM3(STEPPER2_PWM_RATE);
    initPWM4(STEPPER2_PWM_RATE);
    setPWM3(255);
    setPWM4(255);
  }
}

void AF_Stepper::setSpeed(uint16_t rpm)
{
  usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
  steppingcounter = 0;
}

void AF_Stepper::release(void)
{
  if (steppernum == 1)
  {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
                   ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
  }
  else if (steppernum == 2)
  {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
                   ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();
  }
}

void AF_Stepper::step(uint16_t steps, uint8_t dir, uint8_t style)
{
  uint32_t uspers = usperstep;
  uint8_t ret = 0;

  if (style == INTERLEAVE)
  {
    uspers /= 2;
  }
  else if (style == MICROSTEP)
  {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
#ifdef MOTORDEBUG
    Serial.print("steps = ");
    Serial.println(steps, DEC);
#endif
  }

  while (steps--)
  {
    ret = onestep(dir, style);
    delay(uspers / 1000); // in ms
    steppingcounter += (uspers % 1000);
    if (steppingcounter >= 1000)
    {
      delay(1);
      steppingcounter -= 1000;
    }
  }
  if (style == MICROSTEP)
  {
    while ((ret != 0) && (ret != MICROSTEPS))
    {
      ret = onestep(dir, style);
      delay(uspers / 1000); // in ms
      steppingcounter += (uspers % 1000);
      if (steppingcounter >= 1000)
      {
        delay(1);
        steppingcounter -= 1000;
      }
    }
  }
}

uint8_t AF_Stepper::onestep(uint8_t dir, uint8_t style)
{
  uint8_t a, b, c, d;
  uint8_t ocrb, ocra;

  ocra = ocrb = 255;

  if (steppernum == 1)
  {
    a = _BV(MOTOR1_A);
    b = _BV(MOTOR2_A);
    c = _BV(MOTOR1_B);
    d = _BV(MOTOR2_B);
  }
  else if (steppernum == 2)
  {
    a = _BV(MOTOR3_A);
    b = _BV(MOTOR4_A);
    c = _BV(MOTOR3_B);
    d = _BV(MOTOR4_B);
  }
  else
  {
    return 0;
  }

  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE)
  {
    if ((currentstep / (MICROSTEPS / 2)) % 2)
    { // we're at an odd step, weird
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS / 2;
      }
      else
      {
        currentstep -= MICROSTEPS / 2;
      }
    }
    else
    { // go to the next even step
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS;
      }
      else
      {
        currentstep -= MICROSTEPS;
      }
    }
  }
  else if (style == DOUBLE)
  {
    if (!(currentstep / (MICROSTEPS / 2) % 2))
    { // we're at an even step, weird
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS / 2;
      }
      else
      {
        currentstep -= MICROSTEPS / 2;
      }
    }
    else
    { // go to the next odd step
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS;
      }
      else
      {
        currentstep -= MICROSTEPS;
      }
    }
  }
  else if (style == INTERLEAVE)
  {
    if (dir == FORWARD)
    {
      currentstep += MICROSTEPS / 2;
    }
    else
    {
      currentstep -= MICROSTEPS / 2;
    }
  }

  if (style == MICROSTEP)
  {
    if (dir == FORWARD)
    {
      currentstep++;
    }
    else
    {
      // BACKWARDS
      currentstep--;
    }

    currentstep += MICROSTEPS * 4;
    currentstep %= MICROSTEPS * 4;

    ocra = ocrb = 0;
    if ((currentstep >= 0) && (currentstep < MICROSTEPS))
    {
      ocra = microstepcurve[MICROSTEPS - currentstep];
      ocrb = microstepcurve[currentstep];
    }
    else if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2))
    {
      ocra = microstepcurve[currentstep - MICROSTEPS];
      ocrb = microstepcurve[MICROSTEPS * 2 - currentstep];
    }
    else if ((currentstep >= MICROSTEPS * 2) && (currentstep < MICROSTEPS * 3))
    {
      ocra = microstepcurve[MICROSTEPS * 3 - currentstep];
      ocrb = microstepcurve[currentstep - MICROSTEPS * 2];
    }
    else if ((currentstep >= MICROSTEPS * 3) && (currentstep < MICROSTEPS * 4))
    {
      ocra = microstepcurve[currentstep - MICROSTEPS * 3];
      ocrb = microstepcurve[MICROSTEPS * 4 - currentstep];
    }
  }

  currentstep += MICROSTEPS * 4;
  currentstep %= MICROSTEPS * 4;

#ifdef MOTORDEBUG
  Serial.print("current step: ");
  Serial.println(currentstep, DEC);
  Serial.print(" pwmA = ");
  Serial.print(ocra, DEC);
  Serial.print(" pwmB = ");
  Serial.println(ocrb, DEC);
#endif

  if (steppernum == 1)
  {
    setPWM1(ocra);
    setPWM2(ocrb);
  }
  else if (steppernum == 2)
  {
    setPWM3(ocra);
    setPWM4(ocrb);
  }

  // release all
  latch_state &= ~a & ~b & ~c & ~d; // all motor pins to 0

  // Serial.println(step, DEC);
  if (style == MICROSTEP)
  {
    if ((currentstep >= 0) && (currentstep < MICROSTEPS))
      latch_state |= a | b;
    if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2))
      latch_state |= b | c;
    if ((currentstep >= MICROSTEPS * 2) && (currentstep < MICROSTEPS * 3))
      latch_state |= c | d;
    if ((currentstep >= MICROSTEPS * 3) && (currentstep < MICROSTEPS * 4))
      latch_state |= d | a;
  }
  else
  {
    switch (currentstep / (MICROSTEPS / 2))
    {
    case 0:
      latch_state |= a; // energize coil 1 only
      break;
    case 1:
      latch_state |= a | b; // energize coil 1+2
      break;
    case 2:
      latch_state |= b; // energize coil 2 only
      break;
    case 3:
      latch_state |= b | c; // energize coil 2+3
      break;
    case 4:
      latch_state |= c; // energize coil 3 only
      break;
    case 5:
      latch_state |= c | d; // energize coil 3+4
      break;
    case 6:
      latch_state |= d; // energize coil 4 only
      break;
    case 7:
      latch_state |= d | a; // energize coil 1+4
      break;
    }
  }

  MC.latch_tx();
  return currentstep;
}
