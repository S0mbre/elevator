#include "elevator.h"
#include <TMRpcm.h>
#include <SD.h>
#include <Servo.h>
#include <L293.h>
#include <LiquidCrystal.h>
#include <EasyButton.h>
#include <SPI.h>

//**************************************************

#define DEBUG_OUTPUT_ON               // для вывода отладочных сообщений в консоль (закомментировать для релиза!)

#define SERIAL_BAUD 9600
#define FLOORS 4                      // кол-во этажей
#define FLOORMOTOR_SPEED 80           // скорость привода дверей на этажах (0 - 255)
#define ELEV_MOTOR_SPEED 50
#define ELEV_MOTOR_MAXSPEED 100
#define ELEV_MOTOR_ACCEL 20
#define FLOOR_POSITION_ERROR 5        // погрешность позиции кабины для определения этажа (в шагах степпера)
#define DOOR_DELAY 1000               // задержка перед отрытием / после закрытия дверей (в мсек.)
#define FLOOR_DELAY 5000              // задержка перед закрытием дверей для выхода / захода пассажиров (в мсек.)

#define DELAY_LCD 500                 // интервал для вывода на экран текущего этажа и направления (мсек.)

#define ELEV_SERVO_ON 120             // угол (0-180) для активации серво-тормоза (определить экспериментально!!!)
#define ELEV_SERVO_OFF 60             // угол (0-180) для деактивации серво-тормоза (определить экспериментально!!!)

// пины для дисплея
#define PIN_LCD_RS 49
#define PIN_LCD_EN 48
#define PIN_LCD_DATA4 47
#define PIN_LCD_DATA5 46
#define PIN_LCD_DATA6 45
#define PIN_LCD_DATA7 44
// пины для ручного управления лифтом
#define PIN_MAN_CONTROL 28            // ручное управление лифтом
#define PIN_MAN_CONTROL_UP 30         // вверх
#define PIN_MAN_CONTROL_DOWN 32       // вниз
#define PIN_MAN_CONTROL_OPEN 34       // открыть дверь
#define PIN_MAN_CONTROL_CLOSE 35      // закрыть дверь
// пины для управления приводом лифта (через драйвер DIV268N)
#define PIN_MAINMOTOR_EN 22           // привод лифта - активация
#define PIN_MAINMOTOR_DIR 4           // привод лифта - направление
#define PIN_MAINMOTOR_STEP 5          // привод лифта - шаг
// пины для приводов дверей на этажах (через 2 драйвера L293D)
#define PIN_MOTOR_FL_EN 3             // регулировка скорости приводов
#define PIN_MOTOR_FL1_BK 12
#define PIN_MOTOR_FL1_FWD 13
#define PIN_MOTOR_FL2_BK 10
#define PIN_MOTOR_FL2_FWD 11
#define PIN_MOTOR_FL3_BK 8
#define PIN_MOTOR_FL3_FWD 9
#define PIN_MOTOR_FL4_BK 6
#define PIN_MOTOR_FL4_FWD 7
// пины для подключения к шлейфу кабины лифта
#define PIN_ELEV_SERVO A2             // сигнал на сервопривод на кабине лифта (для тормоза)
#define PIN_ELEV_LED 23               // сигнал на светодиод в кабине лифта (через NPN транзистор)
#define PIN_ELEV_BTN A1               // сигнал от кнопок в кабине лифта
// пины прерываний
#define PIN_INT_FL1 18
#define PIN_INT_FL2 19
#define PIN_INT_FL3 21
#define PIN_INT_FL4 20
// пины MicroSD адаптера
#define PIN_SD_MISO 24
#define PIN_SD_MOSI 25
#define PIN_SD_SCK 26
#define PIN_SD_CS 27
// аудиосигнал на усилитель
#define PIN_AUDIO A3

// прочие
#define PIN_FLOOR_BTN A0
#define PIN_FN1 36                    // функциональная (программируемая) кнопка 1

//**************************************************

using namespace Elevatorns;

//**************************************************

LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_DATA4, PIN_LCD_DATA5, PIN_LCD_DATA6, PIN_LCD_DATA7);
RH_ASK main_radio(2000, PIN_MAIN_RADIO_RX, PIN_MAIN_RADIO_TX, 0, false);
AccelStepper elev_stepper(AccelStepper::DRIVER, PIN_MAINMOTOR_STEP, PIN_MAINMOTOR_DIR);
Servo elev_servo;
bool elev_servo_activated;
bool audio_ok;
ErrorCode elev_run_result;
EasyButton btn_elev_up(PIN_MAN_CONTROL_UP, 35, false, false), btn_elev_down(PIN_MAN_CONTROL_DOWN, 35, false, false), 
           btn_doors_open(PIN_MAN_CONTROL_OPEN, 35, false, false), btn_doors_close(PIN_MAN_CONTROL_CLOSE, 35, false, false), btn_fn1(PIN_FN1, 35, false, false);
TMRpcm audiodrv; 
           
struct Audiofiles
{
  const char* Floor1 = "floor1.wav";
  const char* Floor2 = "floor2.wav";
  const char* Floor3 = "floor3.wav";
  const char* Floor4 = "floor4.wav";
} AUDIO_FILES;

struct Lcd_chars
{
  byte up[8] = {0b00100, 0b01110, 0b11111, 0b11111, 0b01110, 0b01110, 0b01110, 0b01110};
  byte down[8] = {0b01110, 0b01110, 0b01110, 0b01110, 0b11111, 0b11111, 0b01110, 0b00100};
} LCD_CHARS;

L293 FLOOR_MOTORS[FLOORS] = {L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL1_FWD, PIN_MOTOR_FL1_BK), L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL2_FWD, PIN_MOTOR_FL2_BK),
  L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL3_FWD, PIN_MOTOR_FL3_BK), L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL4_FWD, PIN_MOTOR_FL4_BK) };
byte FLOOR_DOOR_STATES[FLOORS] = {0, 0, 0, 0};

long FLOOR_POSITIONS[FLOORS] = {1000, 800, 600, 400}; // позиции кабины лифта для привода - изменить при настройке!
unsigned long counter1 = 0;

//**************************************************


void init_pins()
{
  pinMode(PIN_INT_FL1, INPUT); //digitalWrite(PIN_INT_FL1, LOW);
  pinMode(PIN_INT_FL2, INPUT); //digitalWrite(PIN_INT_FL2, LOW);
  pinMode(PIN_INT_FL3, INPUT); //digitalWrite(PIN_INT_FL3, LOW);
  pinMode(PIN_INT_FL4, INPUT); //digitalWrite(PIN_INT_FL4, LOW);
  
  pinMode(PIN_ELEV_SERVO, OUTPUT);
  elev_servo.attach(PIN_ELEV_SERVO);
  pinMode(PIN_ELEV_LED, OUTPUT);
  pinMode(PIN_ELEV_BTN, INPUT);

  pinMode(PIN_FLOOR_BTN, INPUT);
  pinMode(PIN_MAN_CONTROL, INPUT);  

  btn_elev_up.begin();
  btn_elev_down.begin();
  btn_doors_open.begin();
  btn_doors_close.begin();
  btn_fn1.begin();
}

void lcd_show(const Direction dir, long figure)
{
  lcd.clear();
  lcd.setCursor(6, 0);
  switch(dir) {
    case Direction::UP:
      lcd.write(byte(0));
      break;
    case Direction::DOWN:
      lcd.write(byte(1));
      break;
    default:
      ;
  }
  lcd.setCursor(10, 0);
  lcd.print(figure);
}

byte floordoor_control(int fl, const char command)
{
  if(fl < 1 || fl > FLOORS) return 3;
  
  L293* motor = &FLOOR_MOTORS[fl - 1];  
    
  switch(command) {
    
    case 'o': // открыть  
      if(FLOOR_DOOR_STATES[fl - 1]==0) {  
        motor->stop();
        motor->forward(FLOORMOTOR_SPEED); // убедиться в подключении пинов или изменить на back()
        FLOOR_DOOR_STATES[fl - 1] = 1;
        // мотор сам отключится при зажатии одного из концевых выключателей (сработает прерывание)
      }
      return 1;
      
    case 'c': // закрыть
      if(FLOOR_DOOR_STATES[fl - 1]==1) {
        motor->stop();
        motor->back(FLOORMOTOR_SPEED);
        FLOOR_DOOR_STATES[fl - 1] = 0;
        // мотор сам отключится при зажатии одного из концевых выключателей (сработает прерывание)
      }
      return 0;
      
    case 'g': // снять показание 
      return FLOOR_DOOR_STATES[fl - 1]; 
      
    default:
      ;
      
  }
}

void audio_control(const uint8_t _floor)
{
  if(!audio_ok) return;
  switch(_floor) {
    case 1:
      audiodrv.play(AUDIO_FILES.Floor1);
      break;
    case 2:
      audiodrv.play(AUDIO_FILES.Floor2);
      break;
    case 3:
      audiodrv.play(AUDIO_FILES.Floor3);
      break;
    case 4:
      audiodrv.play(AUDIO_FILES.Floor4);
      break;      
  }
}

void elev_led_control(const bool led_on)
{
  digitalWrite(PIN_ELEV_LED, (int)led_on);
}

void elev_brake_control(const bool brake_activate)
{
  elev_servo.write(brake_activate? ELEV_SERVO_ON : ELEV_SERVO_OFF);
  elev_servo_activated = brake_activate;
}

void elev_assign_floor_positions(Elevator* elevator)
{
  // assign floor positions (in steps) to elevator
  for(uint8_t i=0; i<FLOORS; i++) {
    elevator->set_floor_position(i+1, FLOOR_POSITIONS[i]);
  }
}

void stop_motor1()
{
  FLOOR_MOTORS[0].stop();
  //digitalWrite(PIN_INT_FL1, LOW);
}

void stop_motor2()
{
  FLOOR_MOTORS[1].stop();
  //digitalWrite(PIN_INT_FL2, LOW);
}

void stop_motor3()
{
  FLOOR_MOTORS[2].stop();
  //digitalWrite(PIN_INT_FL3, LOW);
}

void stop_motor4()
{
  FLOOR_MOTORS[3].stop();
  //digitalWrite(PIN_INT_FL4, LOW);
}

void init_lcd()
{
  // initialize LCD and set up the number of columns and rows
  lcd.begin(16, 1);
  // show display
  lcd.display();
  // create new characters
  lcd.createChar(0, LCD_CHARS.up);
  lcd.createChar(1, LCD_CHARS.down);
  // set the cursor to the top left
  lcd.setCursor(0, 0);
}

void init_interrupts()
{
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL1), stop_motor1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL2), stop_motor2, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL3), stop_motor3, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL4), stop_motor4, RISING);
}

void init_stepper()
{
  elev_stepper.setEnablePin(PIN_MAINMOTOR_EN);  
  elev_stepper.setSpeed(ELEV_MOTOR_SPEED);
  elev_stepper.setMaxSpeed(ELEV_MOTOR_MAXSPEED);
  elev_stepper.setAcceleration(ELEV_MOTOR_ACCEL);
}

void init_audio()
{  
  audiodrv.speakerPin = PIN_AUDIO;
  audiodrv.setVolume(5); // 0...7  
  audio_ok = SD.begin(PIN_SD_CS);
  //audiodrv.CSPin = PIN_SD_CS;
}

void stepper_reset_position()
{
  elev_stepper.setCurrentPosition(0);
  elev_stepper.setSpeed(ELEV_MOTOR_SPEED);
  elev_stepper.setMaxSpeed(ELEV_MOTOR_MAXSPEED);
  elev_stepper.setAcceleration(ELEV_MOTOR_ACCEL); 
}

bool on_elev_run(Elevator* elevator)
{
  //--- СОБЫТИЕ НАСТУПАЕТ РЕГУЛЯРНО ПРИ ДВИЖЕНИИ ЛИФТА В АВТОМ. РЕЖИМЕ СОГЛАСНО ОЧЕРЕДИ ВЫЗОВОВ (ИНТЕРВАЛ ВРЕМЕНИ = STEP_DELAY) ---//
  
  // снять тормоз лифта если стоит
  if(elev_servo_activated) elev_brake_control(false);
  
  if(counter1 == 0) counter1 = millis();
  unsigned long m = millis();
  if((m - counter1) >= DELAY_LCD) {
    Elevatorns::Direction _dir = elevator->current_direction();
    uint8_t _floor = elevator->current_floor(); 
    lcd_show(_dir, _floor); 
    #ifdef DEBUG_OUTPUT_ON 
    Serial.print("[AUTO] "); Serial.print(Elevator::Direction_2str(_dir)); Serial.print(": ");
    Serial.println(_floor);
    #endif
    counter1 = m;
  }  
  Mode cur_mode = elevator->current_mode();
  int _manual = digitalRead(PIN_MAN_CONTROL);
  if(cur_mode != _manual) elevator->set_current_mode(_manual? Mode::MANUAL : Mode::AUTO);
  return true;
}

bool on_elev_up_run(Elevator* elevator)
{
  //--- СОБЫТИЕ НАСТУПАЕТ РЕГУЛЯРНО ПРИ ДВИЖЕНИИ ЛИФТА ВВЕРХ В РУЧНОМ РЕЖИМЕ (КАЖДЫЙ 1 ШАГ МОТОРА) ---//
  
  // снять тормоз лифта если стоит
  if(elev_servo_activated) elev_brake_control(false)
  
  if(counter1 == 0) counter1 = millis();
  unsigned long m = millis();
  if((m - counter1) >= DELAY_LCD) {
    Elevatorns::Direction _dir = elevator->current_direction();
    long _pos = elevator->current_pos(); 
    lcd_show(_dir, _pos);
    #ifdef DEBUG_OUTPUT_ON
    Serial.print("[MANUAL] "); Serial.print(Elevator::Direction_2str(_dir)); Serial.print(": ");
    Serial.println(_pos);
    #endif
    counter1 = m;
  }  
  return digitalRead(PIN_MAN_CONTROL) && btn_elev_up.isPressed();  
}

bool on_elev_up_stop(Elevator* elevator)
{
  //--- СОБЫТИЕ НАСТУПАЕТ ПРИ ОСТАНОВКЕ ЛИФТА В РУЧНОМ РЕЖИМЕ (ВВЕРХ ИЛИ ВНИЗ) ---//
  
  Elevatorns::Direction _dir = elevator->current_direction();
  long _pos = elevator->current_pos(); 
  lcd_show(_dir, _pos);
  #ifdef DEBUG_OUTPUT_ON
  Serial.print("[MANUAL] "); Serial.print(Elevator::Direction_2str(_dir)); Serial.print(": ");
  Serial.println(_pos);
  #endif  
}

bool on_elev_down_run(Elevator* elevator)
{
  //--- СОБЫТИЕ НАСТУПАЕТ РЕГУЛЯРНО ПРИ ДВИЖЕНИИ ЛИФТА ВНИЗ В РУЧНОМ РЕЖИМЕ (КАЖДЫЙ 1 ШАГ МОТОРА) ---//
  
  // снять тормоз лифта если стоит
  if(elev_servo_activated) elev_brake_control(false)
  
  if(counter1 == 0) counter1 = millis();
  unsigned long m = millis();
  if((m - counter1) >= DELAY_LCD) {
    Elevatorns::Direction _dir = elevator->current_direction();
    long _pos = elevator->current_pos(); 
    lcd_show_floor(_dir, _floor);
    #ifdef DEBUG_OUTPUT_ON
    Serial.print("[MANUAL] "); Serial.print(Elevator::Direction_2str(_dir)); Serial.print(": ");
    Serial.println(_pos);
    #endif
    counter1 = m;
  }
  return digitalRead(PIN_MAN_CONTROL) && btn_elev_down.isPressed();  
}

void on_elev_arrive(Elevator* elevator, const uint8_t _floor)
{
  //--- СОБЫТИЕ НАСТУПАЕТ ПРИ ПРИБЫТИИ ЛИФТА НА СЛЕД. ЭТАЖ В ОЧЕРЕДИ ---//
    
  // лифт на тормоз
  elev_brake_control(true);
  // сообщить о прибытии на этаж
  audio_control(_floor);   
  // открыть дверь (сразу если нажата кнопка ручного открытия)
  btn_doors_open.read();
  elevator->open_door(btn_doors_open.wasPressed());
  // выпустить пассажиров 
  btn_doors_close.read();
  if(!btn_doors_close.wasPressed()) delay(FLOOR_DELAY);
  // закрыть дверь
  elevator->close_door();
}

void on_elev_start_next(Elevator* elevator, const uint8_t _floor)
{
  //--- СОБЫТИЕ НАСТУПАЕТ ПЕРЕД ТЕМ, КАК ЛИФТ ЕДЕТ НА СЛЕД. ЭТАЖ В ОЧЕРЕДИ ---//

  // закрыть все двери
  elevator->close_all_dooors();
}

//**************************************************

ElevatorEvents elevator_events(
  &on_elev_run,       // OnRun
  0,                  // OnEmptyQueue
  0,                  // OnSuspend
  0,                  // OnSwitchMode
  &on_elev_start_next,// OnStartNext
  &on_elev_arrive,    // OnArrive
  0,                  // OnAdd
  0,                  // OnRemove
  &floordoor_control); // DoorControlFunction
Elevator elevator(FLOORS, &elev_stepper, elevator_events);

//**************************************************


void setup() 
{
  #ifdef DEBUG_OUTPUT_ON
  Serial.begin(SERIAL_BAUD);
  #endif
  init_pins();
  init_interrupts();
  init_stepper();
  init_lcd();
  init_audio();

  btn_fn1.onPressed(stepper_reset_position);

  elev_assign_floor_positions(&elevator);
  elevator.set_door_delay(DOOR_DELAY);
  elevator.set_floor_delay(FLOOR_DELAY);

  // лифт на тормоз
  elev_brake_control(true);
  // закрыть двери
  elevator.close_all_dooors();
}

void loop() 
{
  btn_elev_up.read();
  btn_elev_down.read();
  btn_doors_open.read();
  btn_doors_close.read();
  btn_fn1.read();

  elev_run_result = elevator.run();
  if(elev_run_result == ErrorCode::E_NO_DRIVER) {
    return;
  }
  
  Mode cur_mode = elevator.current_mode();
  
  switch(cur_mode) {
    case Mode::AUTO:
      elev_led_control(true);
      break;

    case Mode::MANUAL:
      elev_led_control(true);
      if(digitalRead(PIN_MAN_CONTROL_UP)) {
        elevator.move_up(&on_elev_up_run, &on_elev_up_stop);  
      }
      if(digitalRead(PIN_MAN_CONTROL_DOWN)) {
        elevator.move_up(&on_elev_down_run, &on_elev_up_stop);  
      }
      break;

    case Mode::OFF:
      elev_led_control(false);
      break;
  }
}
