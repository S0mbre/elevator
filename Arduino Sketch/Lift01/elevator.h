#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <Arduino.h>
#include <AccelStepper.h>

// ----------------------------------------------------------------------
namespace Elevatorns {
// ----------------------------------------------------------------------

class Elevator;

enum ErrorCode : uint8_t {E_OK, E_UNKNOWN, E_OUT_OF_RANGE, E_Q_FULL, E_Q_HAS_DUPLICATE, E_Q_NEXTTARGET_EMPTY, E_NO_FLOORS, 
                          E_NO_DRIVER, E_WRONG_MODE, E_NO_EVENT_ASSIGNED, E_FLOOR_POS_UNASSIGNED};
enum Direction : uint8_t {UP, DOWN, STOP, NA};
enum Mode : uint8_t {AUTO, MANUAL, OFF, MALFUNC};

typedef bool (*ElevatorEvent)(Elevator* object);
typedef bool (*ElevatorModeEvent)(Elevator* object, const Mode new_mode);
typedef void (*ElevatorFloorEvent)(Elevator* object, const uint8_t _floor);
typedef byte (*DoorControl)(const int target_floor, const char command);

// ----------------------------------------------------------------------

struct ElevatorEvents
{
  ElevatorEvent OnRun, OnEmptyQueue, OnSuspend;
  ElevatorModeEvent OnSwitchMode;
  ElevatorFloorEvent OnStartNext, OnArrive, OnAdd, OnRemove; 
  DoorControl DoorControlFunction; 

  ElevatorEvents(ElevatorEvent _OnRun=0, ElevatorEvent _OnEmptyQueue=0, ElevatorEvent _OnSuspend=0,
                 ElevatorModeEvent _OnSwitchMode=0, ElevatorFloorEvent _OnStartNext=0, ElevatorFloorEvent _OnArrive=0, 
                 ElevatorFloorEvent _OnAdd=0, ElevatorFloorEvent _OnRemove=0,
                 DoorControl _DoorControlFunction=0) :
                 OnRun(_OnRun), OnEmptyQueue(_OnEmptyQueue), OnSuspend(_OnSuspend), 
                 OnSwitchMode(_OnSwitchMode), OnStartNext(_OnStartNext), OnArrive(_OnArrive), OnAdd(_OnAdd), 
                 OnRemove(_OnRemove), DoorControlFunction(_DoorControlFunction) { }
};

// ----------------------------------------------------------------------

class Elevator 
{
public:  
	static const uint8_t MAX_NUM_FLOORS = 100;
  static const uint8_t STEP_DELAY = 5;
  static const long MAX_DOOR_DELAY = 10000;
  static const long MAX_FLOOR_DELAY = 30000;

  static const char* Direction_2str(const Direction& _dir)
  {
    switch(_dir) {
      case Direction::UP: return "UP";
      case Direction::DOWN: return "DOWN";
      case Direction::STOP: return "STOP";
    }
    return "N/A";
  }

  static const char* Mode_2str(const Mode& _mode)
  {
    switch(_mode) {
      case Mode::AUTO: return "AUTO";
      case Mode::MANUAL: return "MANUAL";
      case Mode::OFF: return "OFF";
      case Mode::MALFUNC: return "MALFUNCTION";
    }
    return "N/A";  
  }

  static const char* Error_2str(const ErrorCode& _error)
  {
    switch(_error) {
      case ErrorCode::E_OK: return "SUCCESS";
      case ErrorCode::E_OUT_OF_RANGE: return "Parameter (e.g. floor) out of range (floor must be in range [1..FLOOR_COUNT])";
      case ErrorCode::E_Q_FULL: return "Elevator queue is full";
      case ErrorCode::E_Q_HAS_DUPLICATE: return "Floor already in Elevator queue";
      case ErrorCode::E_Q_NEXTTARGET_EMPTY: return "No next floor to move to (_next_floor pointer empty)";
      case ErrorCode::E_NO_FLOORS: return "Number of floors is zero";
      case ErrorCode::E_NO_DRIVER: return "Stepper motor driver is absent (_motor pointer empty)";
      case ErrorCode::E_WRONG_MODE: return "Wrong elevator operation mode";
      case ErrorCode::E_NO_EVENT_ASSIGNED: return "This operation is not associated with corresponding event handler (function pointer empty)";
      case ErrorCode::E_FLOOR_POS_UNASSIGNED: return "Floor position(s) not assigned";
    }
    return "Unknown error";  
  }

public:
	Elevator(uint8_t floors, AccelStepper* elev_motor_driver, const ElevatorEvents& events=ElevatorEvents());
	~Elevator() { freemem(); }

  ErrorCode move_to();                             // move to next floor in queue
	ErrorCode move_to(uint8_t target_floor);			    // move to specified floor
	ErrorCode move_to(float abs_position, ElevatorEvent on_run, ElevatorEvent on_stop);			   // move to absolute position in manual mode (see abs_position()) -- BLOCKING!
  ErrorCode move_up(ElevatorEvent on_run, ElevatorEvent on_stop);            // move upwards slowly in manual mode -- BLOCKING!
  ErrorCode move_down(ElevatorEvent on_run, ElevatorEvent on_stop);          // move downwards slowly in manual mode -- BLOCKING!  
	void suspend();								              // suspend operation (queue remains filled)
  void resume();                              // resume operation (after suspension)
	ErrorCode run();		                        // run current queue (in AUTO mode)
	ErrorCode add_target(uint8_t target_floor);	// add target floor to queue (honoring elev logic)
  void remove_target(uint8_t target_floor);
	void clear_queue();							            // clear queue and stop after current job
	void move_to_closest();						          // stop at closest floor as to current position (if between floors)
	ErrorCode open_door(bool immediate=false);							        // open door at current floor
	ErrorCode open_door(uint8_t target_floor, bool immediate=false);	// open door at specified floor
	ErrorCode close_door(bool immediate=false);							        // close door at current floor
	ErrorCode close_door(uint8_t target_floor, bool immediate=false);	// close door at specified floor
	ErrorCode open_all_dooors();						    // open all floor doors
	ErrorCode close_all_dooors();					      // close all floor doors
  ErrorCode get_door_state(uint8_t target_floor, byte& door_state); // get state of a specific door (open==1/closed==0) 
  ErrorCode update_current_floor();           // update current floor and absolute position in steps
  ErrorCode update_current_directon();        // update current direction
  ErrorCode update_current_values();          // update all current values (floor, position and direction)
  	
  //--- properties ---

    // number of floors
  const uint8_t floor_count() { return _num_floors; }
  void set_floor_count(uint8_t num_floors);  
    // elevator mode
  const Mode current_mode() { return _mode; }
  void set_current_mode(const Mode elev_mode=Mode::AUTO);          
    // elevator status / direction
  const Direction current_direction() { return _dir; }
    // current floor
  const uint8_t current_floor() { return _curr_floor; }
    // current position (in steps)
  const long current_pos() { return _floor_position; }
  void set_current_pos(const long pos);
    // current position (as fraction of floor in range [1;max_floor], e.g. 2.5 = between 2 and 3 (middle))
  const float abs_pos(bool update_position=true);              
    // door open/close delay (in msec)
  const long door_delay() { return _door_delay; }
  void set_door_delay(const long _delay=1000) { _door_delay = (_delay > 0 && _delay < MAX_DOOR_DELAY)? _delay : 1000; }
    // floor delay - for passengers enter/exit (in msec)
  const long floor_delay() { return _floor_delay; }
  void set_floor_delay(const long _delay=5000) { _floor_delay = (_delay > 0 && _delay < MAX_FLOOR_DELAY)? _delay : 5000; }
    // next floor
  const uint8_t next_floor() { return _next_floor? *_next_floor : 0; }
    // any floor position (in steps)
  const long floor_position(uint8_t target_floor) { return _floor_positions[target_floor-1]; }  
  void set_floor_position(uint8_t target_floor, const long position);  // sets reference position for specified floor (e.g. in motor steps)
	  // number of floors still to visit before idle
  const uint8_t floors_to_visit();
    // check idle status
  const bool has_work() { return floors_to_visit() > 0; }
  const bool is_running() { return _is_running; } 
	
protected:
	AccelStepper* _motor;
	ElevatorEvents _events;

  volatile bool _is_running;
	uint8_t _num_floors;                        // number of floors
	volatile uint8_t _curr_floor;               // floor where elevator is at
	uint8_t* _floorq;                           // floors visiting queue
	volatile uint8_t* _next_floor;              // pointer to next target floor in queue
	volatile Mode _mode;                        // elevator function mode
	volatile Direction _dir;
  volatile long _floor_position;
	long _door_delay;
  long _floor_delay;
	long* _floor_positions;                     // reference floor positions (e.g. in motor steps)
	long _floor_position_tolerance;             // floor position tolerance (e.g. in motor steps)
  volatile bool _suspended;
  void optimize_queue();                      // optimize floor queue to honor elevator logic
  ErrorCode move(const long steps, ElevatorEvent on_run, ElevatorEvent on_stop);

  inline void freemem() 
  { 
    if(_floorq) { free(_floorq); _floorq = 0; } 
    if(_floor_positions) { free(_floor_positions); _floor_positions = 0; } 
    _next_floor = 0;
  }

private:
  void queue_advance();                       // mark reached floor (=0) and move _next_floor ptr to next in queue
  uint8_t* get_floor_in_q(const uint8_t _floor);
  int8_t get_floor_index_in_q(const uint8_t _floor);
  const bool q_empty();

  inline bool Success(const ErrorCode& result) { return result == ErrorCode::E_OK; } 
};

// ----------------------------------------------------------------------
} // Elevatorns

#endif
