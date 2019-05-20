#include "elevator.h"
#include <TMRpcm.h>
#include <SD.h>
#include <Servo.h>
#include <L293.h>
#include <LiquidCrystal.h>
#include <EasyButton.h>
#include <AnalogMultiButton.h>
#include <NewPing.h>
#include <SPI.h>

//**************************************************

#define DEBUG_OUTPUT_ON                   // для вывода отладочных сообщений в консоль (закомментировать для релиза!)

//**************************************************

using namespace Elevatorns;

//**************************************************

const int SERIAL_BAUD = 9600;
const int FLOORS = 4;                      // кол-во этажей
const int FLOORMOTOR_SPEED = 80;           // скорость привода дверей на этажах (0 - 255)
const int ELEV_MOTOR_SPEED = 50;
const int ELEV_MOTOR_MAXSPEED = 100;
const int ELEV_MOTOR_ACCEL = 20;
const int FLOOR_POSITION_ERROR = 5;        // погрешность позиции кабины для определения этажа (в шагах степпера)
const int DOOR_DELAY = 1000;               // задержка перед отрытием / после закрытия дверей (в мсек.)
const int FLOOR_DELAY = 5000;              // задержка перед закрытием дверей для выхода / захода пассажиров (в мсек.)
const int DELAY_LCD = 500;                 // интервал для вывода на экран текущего этажа и направления (мсек.)
const int DELAY_LCD_OFF = 20000;           // интервал для отключения подсветки ЖК
const int ELEV_SERVO_ON = 120;             // todo: угол (0-180) для активации серво-тормоза (определить экспериментально!!!)
const int ELEV_SERVO_OFF = 60;             // todo: угол (0-180) для деактивации серво-тормоза (определить экспериментально!!!)
const int MAX_DISTANCE = 100;              // макс. корректное расстояние для датчика расстояния (см)
const int MAINLOOP_DELAY = 100;            // задержка после основного цикла в loop()

// пины для дисплея
const int PIN_LCD_RS = 49;
const int PIN_LCD_EN = 48;
const int PIN_LCD_DATA4 = 47;
const int PIN_LCD_DATA5 = 46;
const int PIN_LCD_DATA6 = 45;
const int PIN_LCD_DATA7 = 44;
// пины для ручного управления лифтом
const int PIN_MAN_CONTROL = 28;            // ручное управление лифтом
const int PIN_MAN_CONTROL_UP = 30;         // вверх
const int PIN_MAN_CONTROL_DOWN = 32;       // вниз
const int PIN_MAN_CONTROL_OPEN = 34;       // открыть дверь
const int PIN_MAN_CONTROL_CLOSE = 35;      // закрыть дверь
// пины для управления приводом лифта (через драйвер DIV268N)
const int PIN_MAINMOTOR_EN = 22;           // привод лифта - активация
const int PIN_MAINMOTOR_DIR = 4;           // привод лифта - направление
const int PIN_MAINMOTOR_STEP = 5;          // привод лифта - шаг
// пины для приводов дверей на этажах (через 2 драйвера L293D)
const int PIN_MOTOR_FL_EN = 3;             // регулировка скорости приводов
const int PIN_MOTOR_FL1_BK = 12;
const int PIN_MOTOR_FL1_FWD = 13;
const int PIN_MOTOR_FL2_BK = 10;
const int PIN_MOTOR_FL2_FWD = 11;
const int PIN_MOTOR_FL3_BK = 8;
const int PIN_MOTOR_FL3_FWD = 9;
const int PIN_MOTOR_FL4_BK = 6;
const int PIN_MOTOR_FL4_FWD = 7;
// пины для подключения к шлейфу кабины лифта
const int PIN_ELEV_SERVO = A2;             // сигнал на сервопривод на кабине лифта (для тормоза)
const int PIN_ELEV_LED = 23;               // сигнал на светодиод в кабине лифта (через NPN транзистор)
// пины прерываний
const int PIN_INT_FL1 = 18;
const int PIN_INT_FL2 = 19;
const int PIN_INT_FL3 = 21;
const int PIN_INT_FL4 = 20;
// пины MicroSD адаптера
const int PIN_SD_MISO = 24;
const int PIN_SD_MOSI = 25;
const int PIN_SD_SCK = 26;
const int PIN_SD_CS = 27;
// аудиосигнал на усилитель
const int PIN_AUDIO = A3;
// пины дальномера HC-SR04
const int PIN_US_ECHO = 31;
const int PIN_US_TRIG = 33;
// пины для сигнала от кнопок вызова лифта
const int PIN_FLOOR_BTN = A0;              // сигнал от кнопок на "этажах" (в маш. зале)
const int PIN_ELEV_BTN = A1;               // сигнал от кнопок в кабине лифта
// прочие
const int PIN_FN1 = 36;                    // функциональная (программируемая) кнопка 1

//**************************************************
 
byte FLOOR_DOOR_STATES[FLOORS] = {0, 0, 0, 0};
long FLOOR_POSITIONS[FLOORS] = {1000, 800, 600, 400};               // todo: позиции кабины лифта для привода - изменить при настройке!
long FLOOR_DISTANCES[FLOORS] = {800, 600, 400, 200};                // todo: значения расстояния от датчика расстояния на дне кабины до дна шахты - изменить 1 раз при настройке!
const int FLOOR_BTN_SIGNALS_EL[FLOORS] = {800, 900, 1000, 1020};    // todo: ЗНАЧЕНИЯ СОПРОТИВЛЕНИЯ (0...1023) В ПОРЯДКЕ ВОЗРАСТАНИЯ! заполнить 1 раз при настройке! 
const int FLOOR_BTN_SIGNALS_FL[FLOORS] = {800, 900, 1000, 1020};    // todo: ЗНАЧЕНИЯ СОПРОТИВЛЕНИЯ (0...1023) В ПОРЯДКЕ ВОЗРАСТАНИЯ! заполнить 1 раз при настройке!

// массив объектов L293 для управления приводами дверей
L293 FLOOR_MOTORS[FLOORS] = { L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL1_FWD, PIN_MOTOR_FL1_BK), L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL2_FWD, PIN_MOTOR_FL2_BK),
                              L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL3_FWD, PIN_MOTOR_FL3_BK), L293(PIN_MOTOR_FL_EN, PIN_MOTOR_FL4_FWD, PIN_MOTOR_FL4_BK) };
// объект LiquidCrystal для вывода данных на ЖК дисплей                            
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_DATA4, PIN_LCD_DATA5, PIN_LCD_DATA6, PIN_LCD_DATA7);
// объект для управления шаговым двигателем лифта (подключенным через плату)
AccelStepper elev_stepper(AccelStepper::DRIVER, PIN_MAINMOTOR_STEP, PIN_MAINMOTOR_DIR);
// сервопривод на кабине лифта (для торможения)
Servo elev_servo;
// 5 кнопок для ручного управления: ВВЕРХ, ВНИЗ, ОТКР ДВЕРЬ, ЗАКР ДВЕРЬ, ПРОГРАММИРУЕМАЯ
EasyButton btn_elev_up(PIN_MAN_CONTROL_UP, 35, false, false), btn_elev_down(PIN_MAN_CONTROL_DOWN, 35, false, false), 
           btn_doors_open(PIN_MAN_CONTROL_OPEN, 35, false, false), btn_doors_close(PIN_MAN_CONTROL_CLOSE, 35, false, false), btn_fn1(PIN_FN1, 35, false, false);
// объект TMRpcm для проигрывания аудио (WAV) с SD карты через динамик
TMRpcm audiodrv; 
// ультразвуковой датчик расстояния HC-SR04
NewPing sonar(PIN_US_TRIG, PIN_US_ECHO, MAX_DISTANCE);
// объекты для анализа сигналов от кнопок по 1-проводной схеме (аналоговый сигнал через резисторы на кнопках)
AnalogMultiButton call_btns_el(PIN_ELEV_BTN, FLOORS, FLOOR_BTN_SIGNALS_EL),     // в лифте
                  call_btns_fl(PIN_FLOOR_BTN, FLOORS, FLOOR_BTN_SIGNALS_FL);    // в машзале
// структура для хранения путей к аудиофайлам в формате WAV          
struct Audiofiles
{
  const char* Floor1 = "floor1.wav";  // прибытие на этаж 1
  const char* Floor2 = "floor2.wav";  // прибытие на этаж 2
  const char* Floor3 = "floor3.wav";  // прибытие на этаж 3
  const char* Floor4 = "floor4.wav";  // прибытие на этаж 4
} AUDIO_FILES;

// структура для хранения собственных символов для вывода на ЖК дисплей
struct Lcd_chars
{
  byte up[8] = {0b00100, 0b01110, 0b11111, 0b11111, 0b01110, 0b01110, 0b01110, 0b01110};      // стрелка "вверх"
  byte down[8] = {0b01110, 0b01110, 0b01110, 0b01110, 0b11111, 0b11111, 0b01110, 0b00100};    // стрелка "вниз"
} LCD_CHARS;

bool elev_servo_activated;            // флаг активации тормоза на лифте (через сервопривод)
bool audio_ok;                        // флаг успешной загрузки модуля аудио
unsigned long counter1=0, counter2=0;           // счетчики (мсек)
volatile uint8_t us_distance = 0;     // текущее расстояние от УЗ датчика на кабине лифта до дна шахты (см.)
ErrorCode elev_run_result;            // результат выполнения методов Elevator
volatile int manual_mode = 0;

//**************************************************


void init_pins()
{
  /*
   * Инициирует пины Arduino для вывода / ввода.
   */
  // пины прерываний (18 - 21) - на прослушку
  pinMode(PIN_INT_FL1, INPUT); //digitalWrite(PIN_INT_FL1, LOW);
  pinMode(PIN_INT_FL2, INPUT); //digitalWrite(PIN_INT_FL2, LOW);
  pinMode(PIN_INT_FL3, INPUT); //digitalWrite(PIN_INT_FL3, LOW);
  pinMode(PIN_INT_FL4, INPUT); //digitalWrite(PIN_INT_FL4, LOW);
  // пин управ. сервоприводов на лифте
  pinMode(PIN_ELEV_SERVO, OUTPUT);
  // подсоединить к объекту Servo
  elev_servo.attach(PIN_ELEV_SERVO);
  // пин светодиода в лифте
  pinMode(PIN_ELEV_LED, OUTPUT);
  // пин от переключателя ручного управления
  pinMode(PIN_MAN_CONTROL, INPUT);  

  // инициировать объекты EasyButton
  btn_elev_up.begin();
  btn_elev_down.begin();
  btn_doors_open.begin();
  btn_doors_close.begin();
  btn_fn1.begin();
}

void lcd_print(const char* what, bool clearlcd=true, int col=0, int row=0)
{
  /*
   * Вывод произвольного текста на ЖК дисплей.
   * - what: текст для вывода
   * - clearlcd: требуется ли очистить экран до вывода
   * - col: позиция столбца (0-15)
   * - row: позиция строки (0-1)
   */
  if(clearlcd) lcd.clear();  
  lcd.setCursor(col, row); 
  lcd.print(what);
}

void lcd_show(const Direction dir, long figure)
{
  /*
   * Вывод на ЖК дисплей данных о положении и направлении лифта.
   * - dir: направление (стрелка "вверх" или "вниз" или пустота если не едет)
   * - figure: текущий этаж или положение
   */
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
  /*
   * Управление дверями на этажах.
   * - fl: номер этажа (от 1 до FLOORS)
   * - command: управляющий символ:
   *  - 'o': открыть дверь
   *  - 'c': закрыть дверь
   *  - 'g': получить статус двери
   * Возвращает 0 если дверь была закрыта, 1 - если открыта.
   */
  if(fl < 1 || fl > FLOORS) return 3;
  
  L293* motor = &FLOOR_MOTORS[fl - 1];  
    
  switch(command) {
    
    case 'o': // открыть  
      if(FLOOR_DOOR_STATES[fl - 1]==0) {  
        motor->stop();
        motor->forward(FLOORMOTOR_SPEED); // todo: убедиться в подключении пинов или изменить на back()
        FLOOR_DOOR_STATES[fl - 1] = 1;
        // мотор сам отключится при зажатии одного из концевых выключателей (сработает прерывание)
      }
      return 1;
      
    case 'c': // закрыть
      if(FLOOR_DOOR_STATES[fl - 1]==1) {
        motor->stop();
        motor->back(FLOORMOTOR_SPEED);    // todo: убедиться в подключении пинов или изменить на back()
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
  /*
   * Управление аудио проигрывателем.
   * Проигрывает файл с карты памяти в зависимости от заданного этажа.
   */
  if(!audio_ok) return; // выйти если аудио не инициализировано
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
  /*
   * Управление освещением в лифте (вкл / выкл).
   */
  digitalWrite(PIN_ELEV_LED, (int)led_on);
}

void elev_brake_control(const bool brake_activate)
{
  /*
   * Управление тормозом лифта.
   * Если brake_activate == true, активировать тормоз, если false - снять.
   */
  elev_servo.write(brake_activate? ELEV_SERVO_ON : ELEV_SERVO_OFF);
  elev_servo_activated = brake_activate;
}

void elev_update_queue(Elevator* elevator)
{
  /*
   * Прочитать статус всех кнопок вызова лифта (из кабины и в маш. зале)
   * и добавить соответствующие этажи в очередь.
   */
  call_btns_el.update();      // статус кнопок в лифте
  call_btns_fl.update();      // ... в машзале

  // пройтись по всем кнопкам и добавить этажи в очередь
  for(int i=0; i<FLOORS; i++) {    
    if(call_btns_el.onRelease(i)) elevator->add_target(i+1);
    if(call_btns_fl.onRelease(i)) elevator->add_target(i+1);    
  }
}

void elev_update_distance()
{
  /*
   * Обновить данные расстояния от лифта до дна шахты (прочитав с УЗ датчика).
   */
  us_distance = sonar.ping_cm();
}

void stop_motor1()
{
  /*
   * Обработчик прерывания для управления приводом двери на этаже 1 (остановить мотор).
   */
  FLOOR_MOTORS[0].stop();
  //digitalWrite(PIN_INT_FL1, LOW);
}

void stop_motor2()
{
  /*
   * Обработчик прерывания для управления приводом двери на этаже 2 (остановить мотор).
   */
  FLOOR_MOTORS[1].stop();
  //digitalWrite(PIN_INT_FL2, LOW);
}

void stop_motor3()
{
  /*
   * Обработчик прерывания для управления приводом двери на этаже 3 (остановить мотор).
   */
  FLOOR_MOTORS[2].stop();
  //digitalWrite(PIN_INT_FL3, LOW);
}

void stop_motor4()
{
  /*
   * Обработчик прерывания для управления приводом двери на этаже 4 (остановить мотор).
   */
  FLOOR_MOTORS[3].stop();
  //digitalWrite(PIN_INT_FL4, LOW);
}

void init_lcd()
{
  /*
   * Инициализирует ЖК дисплей.
   */
  lcd.begin(16, 1);       // 16 столбцов, 1 строка (можно 2 строки сделать)
  // отобразить
  lcd.display();
  // создать спец. символы
  lcd.createChar(0, LCD_CHARS.up);
  lcd.createChar(1, LCD_CHARS.down);
  // курсор в начало
  lcd.setCursor(0, 0);
}

void init_interrupts()
{
  /*
   * Инициализация аппаратных прерываний.
   */

  // прерывания для остановки приводов дверей при замыкании концевых кнопок на этажах в крайних положениях двери
  // срабатывают при переключении логического уровня с LOW на HIGH
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL1), stop_motor1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL2), stop_motor2, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL3), stop_motor3, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INT_FL4), stop_motor4, RISING);
}

void init_stepper()
{
  /*
   * Инициализация шагового двигателя (привода лифта).
   */
  elev_stepper.setEnablePin(PIN_MAINMOTOR_EN);      // пин активации мотора 
  elev_stepper.setSpeed(ELEV_MOTOR_SPEED);          // скорость (при движении с пост. скоростью)
  elev_stepper.setMaxSpeed(ELEV_MOTOR_MAXSPEED);    // макс. скорость
  elev_stepper.setAcceleration(ELEV_MOTOR_ACCEL);   // ускорение / замедление (при движении из точки в точку)
}

void init_audio()
{  
  /*
   * Инициализация движка аудио.
   */
  audiodrv.speakerPin = PIN_AUDIO;                  // пин динамика
  audiodrv.setVolume(5);                            // громкость звука (0...7)
  audio_ok = SD.begin(PIN_SD_CS);                   // установить флаг успеха если смогли подключиться к карте памяти
  //audiodrv.CSPin = PIN_SD_CS;
}

void stepper_reset_position()
{
  /*
   * Сброс текущего положения шагового двигателя (текущее положение будет принято в качестве нулевого).
   */
  elev_stepper.setCurrentPosition(0);
  elev_stepper.setSpeed(ELEV_MOTOR_SPEED);
  elev_stepper.setMaxSpeed(ELEV_MOTOR_MAXSPEED);
  elev_stepper.setAcceleration(ELEV_MOTOR_ACCEL); 
}

bool on_elev_up_run(Elevator* elevator)
{
  /*
   * Обработчик события OnRun объекта лифта (Elevator) для ручного управления при движении вверх.
   * СОБЫТИЕ НАСТУПАЕТ РЕГУЛЯРНО ПРИ ДВИЖЕНИИ ЛИФТА ВВЕРХ В РУЧНОМ РЕЖИМЕ (КАЖДЫЙ 1 ШАГ МОТОРА)
   */
   
  // снять тормоз лифта если стоит
  if(elev_servo_activated) elev_brake_control(false);
  
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
  /*
   * Обработчик события OnStop объекта лифта (Elevator) для ручного управления.
   * СОБЫТИЕ НАСТУПАЕТ ПРИ ОСТАНОВКЕ ЛИФТА В РУЧНОМ РЕЖИМЕ (ВВЕРХ ИЛИ ВНИЗ)
   */
  
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
  /*
   * Обработчик события OnRun объекта лифта (Elevator) для ручного управления при движении вниз.
   * СОБЫТИЕ НАСТУПАЕТ РЕГУЛЯРНО ПРИ ДВИЖЕНИИ ЛИФТА ВНИЗ В РУЧНОМ РЕЖИМЕ (КАЖДЫЙ 1 ШАГ МОТОРА)
   */
  
  // снять тормоз лифта если стоит
  if(elev_servo_activated) elev_brake_control(false);
  
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
  return digitalRead(PIN_MAN_CONTROL) && btn_elev_down.isPressed();  
}

bool on_elev_run(Elevator* elevator)
{
  /*
   * Обработчик события OnRun объекта лифта (Elevator) при движении в автоматическом режиме.
   * СОБЫТИЕ НАСТУПАЕТ РЕГУЛЯРНО ПРИ ДВИЖЕНИИ ЛИФТА В АВТОМ. РЕЖИМЕ СОГЛАСНО ОЧЕРЕДИ ВЫЗОВОВ (ИНТЕРВАЛ ВРЕМЕНИ = STEP_DELAY)
   */
  
  // снять тормоз лифта если стоит
  if(elev_servo_activated) elev_brake_control(false);

  // добавить вызванные этажи
  elev_update_queue(elevator);
  
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

void on_elev_arrive(Elevator* elevator, const uint8_t _floor)
{
  /*
   * Обработчик события OnArrive объекта лифта (Elevator) для автоматического режима.
   * СОБЫТИЕ НАСТУПАЕТ ПРИ ПРИБЫТИИ ЛИФТА НА СЛЕД. ЭТАЖ В ОЧЕРЕДИ
   */
    
  // лифт на тормоз
  elev_brake_control(true);
  // сообщить о прибытии на этаж
  audio_control(_floor); 
  // убрать из очереди текущий этаж
  // todo: dq_floor(_floor);  
  // открыть дверь (сразу если нажата кнопка ручного открытия)
  btn_doors_open.read();
  elevator->open_door(btn_doors_open.wasPressed());
  // выпустить пассажиров 
  btn_doors_close.read();
  if(!btn_doors_close.wasPressed()) delay(FLOOR_DELAY);
  // закрыть дверь
  elevator->close_door(false);
}

void on_elev_start_next(Elevator* elevator, const uint8_t _floor)
{
  /*
   * Обработчик события OnStartNext объекта лифта (Elevator) для автоматического режима.
   * СОБЫТИЕ НАСТУПАЕТ ПЕРЕД ТЕМ, КАК ЛИФТ ЕДЕТ НА СЛЕД. ЭТАЖ В ОЧЕРЕДИ
   */
  
  // включить ЖК экран (если не горит)
  lcd.display();
  counter2 = 0;     // обнулить счетчик ожидания перед выключением ЖК
  
  // закрыть все двери
  elevator->close_all_dooors();
}

bool on_elev_idle(Elevator* elevator)
{
   /*
   * Обработчик события OnEmptyQueue объекта лифта (Elevator) для автоматического режима.
   * СОБЫТИЕ НАСТУПАЕТ, КОГДА В ОЧЕРЕДИ НЕТ ЭТАЖЕЙ (ВСЕ = 0).
   */

  if(counter1 == 0) counter1 = millis();
  unsigned long m = millis();
  if((m - counter1) >= DELAY_LCD) {
    // вывести на ЖК текущий этаж 
    uint8_t _floor = elevator->current_floor(); 
    lcd_show(Direction::STOP, _floor); 
    #ifdef DEBUG_OUTPUT_ON 
    Serial.print(elevator->suspended()? "[SUSP]: " : "[IDLE]: "); 
    Serial.println(_floor);
    #endif 
    counter1 = m; 
  }

  if(counter2 == 0) counter2 = millis();
  if((millis() - counter2) >= DELAY_LCD_OFF) {
    lcd.noDisplay();
  }

  return true;
}

//**************************************************

// объект, содержащий обработчики события лифта
ElevatorEvents elevator_events(
  &on_elev_run,       // OnRun                  -- при движении в авто режиме
  &on_elev_idle,      // OnEmptyQueue           -- при опустошении очереди этажей
  &on_elev_idle,      // OnSuspend              -- при приостановке движения
  0,                  // OnSwitchMode           -- при переключении режима
  &on_elev_start_next,// OnStartNext            -- перед отправкой на след. этаж в очереди (авто режим)
  &on_elev_arrive,    // OnArrive               -- при прибытии на след. этаж в очереди (авто режим)
  0,                  // OnAdd                  -- при добавлении этажа в очередь
  0,                  // OnRemove               -- при удалении этажа из очереди
  &floordoor_control); // DoorControlFunction   -- функция управления дверями

// объект лифта
Elevator elevator(FLOORS, &elev_stepper, elevator_events);

//**************************************************


void setup() 
{
  #ifdef DEBUG_OUTPUT_ON
  Serial.begin(SERIAL_BAUD);            // включить вывод на послед. порт в режиме отладки
  #endif
  init_pins();                          // иниц. пины
  init_interrupts();                    // иниц. прерывания
  init_stepper();                       // иниц. шаговый двигатель
  init_lcd();                           // иниц. ЖК дисплей
  init_audio();                         // иниц. аудио

  btn_fn1.onPressed(stepper_reset_position);      // при нажатии на программную кнопку - сбросить текущее положение шаговика

  elevator.set_floor_positions(FLOOR_POSITIONS);  // назначить положения этажей для лифта (в шагах двигателя)
  elevator.set_door_delay(DOOR_DELAY);            // задержки перед открытием / после закрытия дверей
  elevator.set_floor_delay(FLOOR_DELAY);          // задержка для сбора пассажиров в лифт
  elev_brake_control(true);                       // лифт на тормоз
  elevator.close_all_dooors();                    // закрыть все двери
}

void loop() 
{
  // обновить статус кнопок управления
  btn_elev_up.read();
  btn_elev_down.read();
  btn_doors_open.read();
  btn_doors_close.read();
  btn_fn1.read();

  // прочесть текущий режим
  Mode cur_mode = elevator.current_mode();

  manual_mode = digitalRead(PIN_MAN_CONTROL);
  if(cur_mode != manual_mode) 
    elevator.set_current_mode(manual_mode? Mode::MANUAL : Mode::AUTO);

  // запустить основной цикл лифта (авто режим)
  elev_run_result = elevator.run();

  // если не назначен драйвер - вывести ошибку и уснуть на 5 сек.
  if(elev_run_result == ErrorCode::E_NO_DRIVER) {
    lcd_print("NO ELEV DRIVER!");    
    delay(5000);
    return;
  }
  
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
      lcd_print("OFF");
      break;
  }

  delay(MAINLOOP_DELAY);
}
