
//  letsarduino.com
//  [Project 17] - Stepper Motor Direction Control Using 2 Buttons with the Arduino

/*
 * http://www.letsarduino.com/project-17-stepper-motor-direction-control-using-2-buttons-with-the-arduino/
 * http://howtomechatronics.com/tutorials/arduino/how-to-control-stepper-motor-with-a4988-driver-and-arduino/
 */


// https://img00.deviantart.net/8f5c/i/2015/233/1/a/_arduino_r___like_pro_mini_pinout_diagram_by_adlerweb-d96h91k.png
/* LCD
 *
 *
 * Verbindungen
 * <Kabel> <LCD Platine> => <Arduino>
 * 1 D7 => 2
 * 2 D6 => 3
 * 3 D5 => 4
 * 4 D4 => 5
 * 5 RS => 6  (??? auf D8)
 * 6 EN -> 13
 * 7 analog pin as button input ===> A0 // 23
 * 8 reset als normale taste
 * 9
 * 10
 * 11
 * 12
 * 13 VCC -> VCC
 * 14 GND -> GND
 */
/* settings */

#define USE_LCD_SHIELD 1

/* **************************************************************************
 /  * lcd/inputs
 /  ************************************************************************** */

#if USE_LCD_SHIELD
#include <LiquidCrystal.h>
#define LCD_PIN_ANALOG 14
#define LCD_PIN_RS 6
#define LCD_PIN_ENABLE 13
#define LCD_PIN_D0 5
#define LCD_PIN_D1 4
#define LCD_PIN_D2 3
#define LCD_PIN_D3 2
LiquidCrystal lcd ( LCD_PIN_RS, LCD_PIN_ENABLE, LCD_PIN_D0, LCD_PIN_D1, LCD_PIN_D2, LCD_PIN_D3 );


#define LCD_BTN_UP     0
#define LCD_BTN_DOWN   1
#define LCD_BTN_RIGHT  2
#define LCD_BTN_LEFT   3
#define LCD_BTN_SELECT 4
#define LCD_BTN_NONE   5

int lcd_adc_key       = 0;
int lcd_adc_key_old   = 0;
int lcd_button_old;
unsigned long lcd_check_ts;
unsigned long lcd_update_ts;

enum LcdSettings
{
  LCD_ADC_HYST                  =   10,
  LCD_BUTTON_CHECK_INTERVALL_US = 200000,
};

#endif


/* **************************************************************************
 /  * old inputs
 /  ************************************************************************** */


/* button up/down
 * short press: one step
 * long press: faster
 */
#define BUTTON_UP_PIN   7
#define BUTTON_DOWN_PIN 3

/* button home
 * short press: go to eject position
 * long press: set eject position
 */
#define BUTTON_HOME_PIN 4

/* button position
 * uhm.. same as home but set a second position?
 */
#define BUTTON_POSITION_PIN 5


/* others */
#define LED_GREEN 13


/* motor out pins */
#define MOTOR_PIN_DIR      8
#define MOTOR_PIN_STEP     9
#define MOTOR_PIN_EN      10
#define MOTOR_PIN_SLEEP   11
#define MOTOR_PIN_RESET   12


/* **************************************************************************
 /  * settings
 /  ************************************************************************** */

enum Settings
{
  /* speed */
  MOTOR_STEP_WIDTH_MIN_US       =    1000, /* minimum step width (fastest */
  MOTOR_STEP_WIDTH_MAX_US       =   10000, /* maximum step width (slowest) */
  MOTOR_STEP_WIDTH_STEP_US      =      50,

  MOTOR_STEPS_PER_REV           =     200,

  MOTOR_NM_PER_REV              = 1250000, /*  M8: 1.25mm */
  MOTOR_NM_PER_STEP             =    6250, /* MOTOR_NM_PER_REV /  MOTOR_STEPS_PER_REV */
  MOTOR_NM_TO_MM                = 1000000,

  MOTOR_NBM_PER_REV              = 1280000, /* 1250um x 1024 */
  MOTOR_NBM_PER_STEP             =    6400, /* MOTOR_NBM_PER_REV /  MOTOR_STEPS_PER_REV */
  MOTOR_NBM_TO_UM                =    1024,


  /* faster */
  MOTOR_STEP_COUNT_FASTER_MIN   = 100, /* when to start faster stepping */
  MOTOR_STEP_COUNT_FASTER_MOD   = 7, /* keep 2^n-1, when to ++ step width */

  /* intervals */
  MOTOR_INTERVAL_US            =    100, /* 0.1ms */

  BUTTON_INTERVAL_US           = 100000, /* 100ms */

  LCD_UPDATE_INTERVAL_US       = 100000, /* 100ms */
};


/* **************************************************************************
 /  * enum/structs
 /  ************************************************************************** */
enum MotorState
{
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_RUN_1,
  MOTOR_STATE_RUN_2,
};

enum ButtonState
{
  BUTTON_NEUTRAL = 0,
  BUTTON_PRESSED,
  BUTTON_LONG_PRESSED,
};

enum UpDown
{
  UP    =  1,
  DOWN  = -1,
};



struct Button
{
  unsigned int state;
  unsigned int dir;
  unsigned int pin;
};

enum Buttons
{
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_HOME,
  BUTTON_POS,
  BUTTON_NUM,
};

/* **************************************************************************
 /  * global vars
 /  ************************************************************************** */

/* position */
long pos_nbm;


/* vars motor */
unsigned long motor_ts;
unsigned long motor_check_ts;
unsigned long motor_steps;
unsigned int motor_state;
unsigned int motor_step_width_us;
unsigned int motor_dir;

/* vars buttons */
unsigned long button_check_ts;
unsigned int button_up_state;
struct Button buttons [ BUTTON_NUM ];



void pos_update ( int dir )
{
  if ( dir == UP )
  {
    pos_nbm += MOTOR_NBM_PER_STEP;
  }
  else
  {
    pos_nbm -= MOTOR_NBM_PER_STEP;
  }
}

int pos_get_mm ( void )
{
  return ( pos_nbm >> 10 ) / 1000;
}

int pos_get_um ( void )
{
  return ( pos_nbm >> 10 ) % 1000;
}

void motor_set_dir ( unsigned int dir )
{
  if ( dir == UP )
  {
    digitalWrite ( MOTOR_PIN_DIR, HIGH );
    motor_dir = UP;
  }
  else
  {
    digitalWrite ( MOTOR_PIN_DIR, LOW );
    motor_dir = DOWN;
  }
}

void motor_run (unsigned int dir)
{
  if ( motor_state == MOTOR_STATE_IDLE )
  {
    motor_set_dir ( dir );
    motor_state = MOTOR_STATE_RUN_1;
    motor_step_width_us = MOTOR_STEP_WIDTH_MAX_US;
    digitalWrite ( MOTOR_PIN_STEP, HIGH );
    motor_steps = 0;
    motor_ts = micros ();
  }
}

void motor_stop ( void )
{
  motor_state = MOTOR_STATE_IDLE;
  digitalWrite ( MOTOR_PIN_STEP, LOW );
}

void motor_check ( void )
{
  unsigned long now = micros ();

  switch ( motor_state )
  {
    case MOTOR_STATE_IDLE:
    {

      break;
    }
    case MOTOR_STATE_RUN_1:
    {
      if ( motor_ts + motor_step_width_us < now )
      {
        digitalWrite ( MOTOR_PIN_STEP, LOW );
        motor_ts = now;
        motor_state = MOTOR_STATE_RUN_2;
      }
      break;
    }

    case MOTOR_STATE_RUN_2:
    {
      if ( motor_ts + motor_step_width_us < now )
      {
        digitalWrite ( MOTOR_PIN_STEP, HIGH );
        motor_steps++;
        pos_update ( motor_dir );
        motor_ts = now;
        motor_state = MOTOR_STATE_RUN_1;

        /* faster every n steps */
        if ( ( motor_steps > MOTOR_STEP_COUNT_FASTER_MIN ) && ( motor_steps & MOTOR_STEP_COUNT_FASTER_MOD ) )
        {
          motor_step_width_us -= MOTOR_STEP_WIDTH_STEP_US;
          if ( motor_step_width_us < MOTOR_STEP_WIDTH_MIN_US )
          {
            motor_step_width_us = MOTOR_STEP_WIDTH_MIN_US;
          }
        }
      }
      break;
    }
  }
}


void button_check ( void )
{
  /* direction buttons */
  unsigned int i;
  for ( i = 0; i < 1; i++ )
  {
    switch ( buttons [ i ].state )
    {
      case BUTTON_NEUTRAL:
      {
        if ( !digitalRead ( buttons [ i ].pin ) )
        {
          buttons [ i ].state = BUTTON_PRESSED;
          motor_run ( buttons [ i ].dir );
          Serial.print("pressed\n");
        }

        break;
      }
      case BUTTON_PRESSED:
      {
        if ( !digitalRead ( buttons [ i ].pin ) )
        {
          buttons [ i ].state = BUTTON_PRESSED;
        }
        else
        {
          buttons [ i ].state = BUTTON_NEUTRAL;
          Serial.print("=>neutral\n");
          motor_state = MOTOR_STATE_IDLE;
        }
        break;
      }
    }
  }

  /* home button */
}

#if USE_LCD_SHIELD

void lcd_update ( void )
{
  /* print current position value */
  lcd.setCursor ( 0, 0 );
  lcd.print ( pos_get_mm () );
  lcd.print ( "." );
  lcd.print ( pos_get_um () );
  lcd.print ( "mm" );
}

void lcd_button_eval ( int button )
{
  switch ( button )
  {
    case LCD_BTN_UP:
    case LCD_BTN_DOWN:
    {
      motor_run ( button );
      break;
    }


    case LCD_BTN_LEFT:
    {
      break;
    }

    case LCD_BTN_RIGHT:
    {
      break;
    }

    case LCD_BTN_SELECT:
    {
      pos_nbm = 0;
      break;
    }

    case LCD_BTN_NONE:
    {
      motor_stop ();
      break;
    }
  }
}
#endif

void loop ( void )
{
  unsigned long now = micros ();


  /* buttons */
  if ( button_check_ts + BUTTON_INTERVAL_US < now )
  {
    button_check_ts = now;
    button_check ();
  }

  /* motor */
  if ( motor_check_ts + MOTOR_INTERVAL_US < now )
  {
    motor_check_ts = now;
    motor_check ();
  }
#if 1
#if USE_LCD_SHIELD
  if ( lcd_check_ts + LCD_BUTTON_CHECK_INTERVALL_US < now )
  {
    lcd_check_ts = now;
    int button = lcd_read_buttons ();

    /* print adc value */
    //lcd.setCursor ( 0, 0 );
      //lcd.begin ( 16, 2 );
  lcd.setCursor ( 0, 0 );
  lcd.print ( "ADC      " );
  lcd.setCursor ( 4, 0 );
    lcd.print ( lcd_adc_key );
    /* print key value */
    lcd.setCursor ( 0, 1 );
    lcd.print ( "key " );
    lcd.print ( button );

    if ( lcd_button_old != button )
    {
      /* button changed => eval button */
      lcd_button_old = button;
      lcd_button_eval ( button );
    }
  }

  if ( lcd_update_ts + LCD_UPDATE_INTERVAL_US < now )
  {
    //lcd_update ();
  }
#endif
#endif

}

void setup ()
{
  Serial.begin ( 57600 );
  

#if USE_LCD_SHIELD
  /* lcd */
  lcd.begin ( 16, 2 );
    delay(500);

#endif
  /* inputs */
  //pinMode ( BUTTON_UP_PIN,        INPUT_PULLUP );
  //pinMode ( BUTTON_HOME_PIN,       INPUT_PULLUP );
 // pinMode ( BUTTON_POSITION_PIN,  INPUT_PULLUP );

  /* led */
  //pinMode ( LED_GREEN, OUTPUT );
  //digitalWrite ( LED_GREEN, HIGH );


  /* motor */
  pinMode ( MOTOR_PIN_EN,   OUTPUT );
  pinMode ( MOTOR_PIN_DIR,  OUTPUT );
  pinMode ( MOTOR_PIN_STEP, OUTPUT );
  pinMode ( MOTOR_PIN_SLEEP, OUTPUT );
  pinMode ( MOTOR_PIN_RESET, OUTPUT );

  digitalWrite ( MOTOR_PIN_EN, LOW );
  digitalWrite ( MOTOR_PIN_DIR, LOW );
  digitalWrite ( MOTOR_PIN_STEP, LOW );
  digitalWrite ( MOTOR_PIN_SLEEP, HIGH );
  digitalWrite ( MOTOR_PIN_RESET, HIGH );/* not reset! */
  
  /* reset vars */
  motor_ts = micros();
  motor_state = MOTOR_STATE_IDLE;
  motor_step_width_us = MOTOR_STEP_WIDTH_MAX_US;
  button_check_ts = 0;
  pos_nbm = 0;


  /* direction buttons setup */
  buttons [ BUTTON_UP ].dir = UP;
  buttons [ BUTTON_UP ].state = BUTTON_NEUTRAL;
  buttons [ BUTTON_UP ].pin = BUTTON_UP_PIN;
  buttons [ BUTTON_DOWN ].dir = DOWN;
  buttons [ BUTTON_DOWN ].state = BUTTON_NEUTRAL;
  buttons [ BUTTON_DOWN ].pin = BUTTON_DOWN_PIN;

  buttons [ BUTTON_HOME ].state = BUTTON_NEUTRAL;
  buttons [ BUTTON_HOME ].pin = BUTTON_HOME_PIN;
}


#if USE_LCD_SHIELD

int lcd_read_buttons ()
{
  lcd_adc_key = analogRead ( LCD_PIN_ANALOG );
  int diff = lcd_adc_key - lcd_adc_key_old;
  lcd_adc_key_old = lcd_adc_key;
  if ( abs ( diff ) > LCD_ADC_HYST )
  {
    /* value is not steady enough */
    return LCD_BTN_NONE;
  }
  else
  {
    /* TODO: check values */

    if ( lcd_adc_key > 1000 )
      return LCD_BTN_NONE; // We make this the 1st option for speed reasons since it will be the most likely result
    if ( lcd_adc_key < 50 )
      return LCD_BTN_RIGHT;
    if ( lcd_adc_key < 195 )
      return LCD_BTN_UP;
    if ( lcd_adc_key < 380 )
      return LCD_BTN_DOWN;
    if ( lcd_adc_key < 555 )
      return LCD_BTN_LEFT;
    if ( lcd_adc_key < 790 )
      return LCD_BTN_SELECT;
    return LCD_BTN_NONE;  // when all others fail, return this...
  }
}
#endif

