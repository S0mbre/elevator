#include "elevator.h"

// ----------------------------------------------------------------------

using namespace Elevatorns;

// ----------------------------------------------------------------------

Elevator::Elevator(uint8_t floors, AccelStepper* elev_motor_driver, const ElevatorEvents& events) :
	_floorq(0), _floor_positions(0), _num_floors(0), _floor_position(0),
	_motor(elev_motor_driver), _curr_floor(0), _mode(Mode::AUTO),  
	_door_delay(1000), _suspended(false), _events(events), _is_running(false)
{
	set_floor_count(floors);
}

// ----------------------------------------------------------------------

void Elevator::set_floor_count(uint8_t num_floors)
{
  if(_motor) {
    if(_motor->isRunning()) {
      _suspended = true;
      _motor->stop();
    }
  }
	if(num_floors > MAX_NUM_FLOORS) num_floors = MAX_NUM_FLOORS;
	// initialize queue size
	freemem();
	_floorq = &Elevq(num_floors);
  
	// floor positions
	_floor_positions = (long*) malloc(num_floors * sizeof(long));
	long* last_el = _floor_positions + num_floors - 1;
	for(long* pt=_floor_positions; pt<=last_el; ++pt) *pt = -1L;
  
	_num_floors = num_floors;
}

// ----------------------------------------------------------------------

void Elevator::set_floor_position(uint8_t target_floor, const long position)
{
  if(target_floor < 1 || target_floor > _num_floors) return;
  _floor_positions[target_floor - 1] = position;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::add_target(uint8_t target_floor)
{
  /*
   * Adds a next target floor to the elevator queue, resorting
   * the queue if necessary to achieve the efficient elevator logic.
   */
   if(!_floorq) return ErrorCode::E_UNKNOWN;
   
   if(target_floor < 1 || target_floor > _num_floors)
    return ErrorCode::E_OUT_OF_RANGE;
   
   uint8_t* pFloor = get_floor_in_q(target_floor);
   if(pFloor) // floor already in queue
    return ErrorCode::E_Q_HAS_DUPLICATE;
      
   // find first vacant pos in queue
   pFloor = get_floor_in_q(0);
   if(!pFloor) // queue full, cannot add anything
    return ErrorCode::E_Q_FULL;
   
   // fill that vacant cell
   *pFloor = target_floor;

   // resort
   optimize_queue();

   if(_events.OnAdd) _events.OnAdd(this, target_floor);

   // run
   if(!_is_running) run();
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move_to()
{
  if(_next_floor) return move_to(*_next_floor);
  return ErrorCode::E_Q_NEXTTARGET_EMPTY;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move_to(uint8_t target_floor)
{
	if(!_motor) return ErrorCode::E_NO_DRIVER;
	if(target_floor < 1 || target_floor > _num_floors) return ErrorCode::E_OUT_OF_RANGE;
  if((uint8_t)_mode > 1) return ErrorCode::E_WRONG_MODE; 
  if(_floor_positions[target_floor - 1] == -1) return ErrorCode::E_FLOOR_POS_UNASSIGNED;
	_motor->moveTo(_floor_positions[target_floor - 1]);
  return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move_to(float abs_position, ElevatorEvent on_run, ElevatorEvent on_stop)
{
	if((uint8_t)_mode > 1) return ErrorCode::E_WRONG_MODE;
  
	if(abs_position > float(_num_floors)) return move_to(_num_floors);		
	if(abs_position < 1.0) return move_to(1);
	
	uint8_t fl = uint8_t(abs_position);
	float fr = abs_position - float(fl);
	long pos_this = _floor_positions[fl - 1];
	long pos_next = _floor_positions[fl];

  if(_mode != Mode::MANUAL) set_current_mode(Mode::MANUAL);
		
	_motor->moveTo(pos_this + long(float(labs(pos_this - pos_next)) * fr));
  while(_motor->runSpeed() && !_suspended) {
    update_current_floor();
    if(on_run) {
      bool result = on_run(this);
      if(!result) {
        _motor->stop(); 
        if(on_stop) on_stop(this);
        return ErrorCode::E_OK;
      }
    }
    delay(Elevator::STEP_DELAY);
  }
  if(on_stop) on_stop(this);
  return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move(const long steps, ElevatorEvent on_run, ElevatorEvent on_stop)
{
  if(!steps) return ErrorCode::E_OK;
  if((uint8_t)_mode > 1) return ErrorCode::E_WRONG_MODE;  
  if(_mode != Mode::MANUAL) set_current_mode(Mode::MANUAL);
  
  _motor->move(steps);
  _dir = steps<0? Direction::UP : Direction::DOWN;
  
  while(_motor->runSpeed() && !_suspended) {
    _floor_position = _motor->currentPosition();
    if(on_run) {      
      bool result = on_run(this);
      if(!result) {
        _motor->stop(); 
        _dir = Direction::STOP;
        if(on_stop) on_stop(this);
        return ErrorCode::E_OK;
      }
    }
    //delay(Elevator::STEP_DELAY);
  }
  _dir = Direction::STOP;
  if(on_stop) on_stop(this);
  return ErrorCode::E_OK;  
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move_up(ElevatorEvent on_run, ElevatorEvent on_stop)
{  
  return move(-1, on_run, on_stop);  
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move_down(ElevatorEvent on_run, ElevatorEvent on_stop)
{
  return move(1, on_run, on_stop);
}

// ----------------------------------------------------------------------

void Elevator::set_current_pos(const long pos)
{
  if(_motor) {
    _motor->setCurrentPosition(pos);  
    _floor_position = pos;
  }
}

// ----------------------------------------------------------------------

void Elevator::suspend()
{
  _suspended = true;
}

// ----------------------------------------------------------------------

void Elevator::resume()
{
  _suspended = false;
  set_current_mode(Mode::AUTO);
}

// ----------------------------------------------------------------------

ErrorCode Elevator::run()
{
  if(!_motor) return ErrorCode::E_NO_DRIVER;
  if(_mode != Mode::AUTO) return ErrorCode::E_WRONG_MODE;
  bool running = false;
  ErrorCode move_result;
  optimize_queue();

  _is_running = true;
  
  while(_mode == Mode::AUTO && _next_floor && !_suspended) {
   
    move_result = move_to();

    if(move_result==ErrorCode::E_OUT_OF_RANGE) {
      _next_floor = 0;
      break;  
    }
    else if(!Success(move_result)) {
      return move_result;
    }

    if(_events.OnStartNext) _events.OnStartNext(this, *_next_floor);

    while(1) {    
      if(_mode != Mode::AUTO) {
         if(_events.OnSwitchMode) _events.OnSwitchMode(this, _mode);
         break;    
      }
      running = _motor->run();    
      update_current_values();
  
      if(!running) {
        // motor has stopped
        _dir = Direction::STOP;
        if(_curr_floor == *_next_floor) {
          // landed at next target_pos
          if(_events.OnArrive) _events.OnArrive(this, _curr_floor);
          queue_advance();
        }
        break; // exit inner loop on motor stop
      }
      else {
        // running to next target
        if(_events.OnRun) _suspended = !_events.OnRun(this);      // update suspended status from callback
      }
      
      // motor step delay
      delay(Elevator::STEP_DELAY);      
    }
    
    optimize_queue();
  }

  if(!_next_floor && _events.OnEmptyQueue) _events.OnEmptyQueue(this);
  if(_suspended && _events.OnSuspend) _events.OnSuspend(this);

  _is_running = false;
  return ErrorCode::E_OK;
}


// ----------------------------------------------------------------------

void Elevator::queue_advance()
{
  *_next_floor = 0;
  uint8_t _index = (_next_floor - _floorq)/sizeof(uint8_t*);
  if(_index < (_num_floors-1)) _next_floor++;
  else _next_floor = _floorq;
  optimize_queue();
}

// ----------------------------------------------------------------------

void Elevator::remove_target(uint8_t target_floor)
{
  uint8_t* pInq = get_floor_in_q(target_floor);
  if(pInq) *pInq = 0;
  optimize_queue();  
}

// ----------------------------------------------------------------------

void Elevator::clear_queue()
{
  for(uint8_t* p=_floorq; p<=(_floorq+_num_floors-1); ++p) *p = 0;  
  _next_floor = 0;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::open_door(uint8_t target_floor, bool immediate)
{
  if(!_events.DoorControlFunction) return ErrorCode::E_NO_EVENT_ASSIGNED;
  if(target_floor < 1 || target_floor > _num_floors) return ErrorCode::E_OUT_OF_RANGE;
  if(!immediate) delay(_door_delay);
  _events.DoorControlFunction(target_floor, 'o');
  return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::open_door(bool immediate)
{
  return open_door(_curr_floor, immediate);
}

// ----------------------------------------------------------------------

ErrorCode Elevator::close_door(uint8_t target_floor, bool immediate)
{
  if(!_events.DoorControlFunction) return ErrorCode::E_NO_EVENT_ASSIGNED;
  if(target_floor < 1 || target_floor > _num_floors) return ErrorCode::E_OUT_OF_RANGE;
  _events.DoorControlFunction(target_floor, 'c');
  if(!immediate) delay(_door_delay);
  return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::close_door(bool immediate)
{
  return close_door(_curr_floor, immediate);
}

// ----------------------------------------------------------------------

ErrorCode Elevator::open_all_dooors()
{
  ErrorCode c, cc=ErrorCode::E_OK;
  for(uint8_t i=0; i<_num_floors; i++) {
    c = open_door(i+1, true);  
    if(c!=ErrorCode::E_OK) cc = c;
  }
  return cc;
}


// ----------------------------------------------------------------------

ErrorCode Elevator::close_all_dooors()
{
  ErrorCode c, cc=ErrorCode::E_OK;
  for(uint8_t i=0; i<_num_floors; i++) {
    c = close_door(i+1, true);  
    if(c!=ErrorCode::E_OK) cc = c;
  }
  return cc;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::get_door_state(uint8_t target_floor, byte& door_state)
{
  if(!_events.DoorControlFunction) return ErrorCode::E_NO_EVENT_ASSIGNED;
  if(target_floor < 1 || target_floor > _num_floors) return ErrorCode::E_OUT_OF_RANGE;
  door_state = _events.DoorControlFunction(target_floor, 'g');
  return ErrorCode::E_OK;  
}

// ----------------------------------------------------------------------

ErrorCode Elevator::update_current_floor()
{
  _curr_floor = 0;
  if(!_motor) return ErrorCode::E_NO_DRIVER;
  _floor_position = _motor->currentPosition();
  for(uint8_t i=_num_floors; i>0; --i) {
    if(_floor_position <= _floor_positions[i-1]) {
      _curr_floor = i;
      break;
    }
  }
  return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::update_current_directon()
{
  _dir = Direction::NA;
  if(!_motor) return ErrorCode::E_NO_DRIVER;
  if(!_motor->isRunning()) _dir = Direction::STOP;
  else _dir = _motor->distanceToGo() > 0? Direction::DOWN : Direction::UP;
  return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::update_current_values()
{
  return max(update_current_floor(), update_current_directon());
}

// ----------------------------------------------------------------------

const float Elevator::abs_pos(bool update_position)
{
  if(!_motor) return 0.0;
  if(update_position) update_current_floor();
  if(_floor_position > 0 && _floor_position <= _floor_positions[_num_floors-1]) {
    for(uint8_t i=_num_floors; i>0; --i) {
      if(_floor_position == _floor_positions[i-1]) 
        return float(i);
      if(_floor_position < _floor_positions[i-1]) 
        return float(i) + float(_floor_positions[i-1] - _floor_position) / float(labs(_floor_positions[i-1] - _floor_positions[i]));
    }
  }
  return 0.0;
}

// ----------------------------------------------------------------------

void Elevator::set_current_mode(const Mode elev_mode)
{  
  _mode = elev_mode;
  
  switch(elev_mode) {
    case Mode::AUTO:
      run();
      break;
    case Mode::MANUAL:
      _motor->stop();
      break;
    case Mode::OFF:
      _motor->stop();
      break;
    case Mode::MALFUNC:
      _motor->stop();
      open_all_dooors();
      break;
  }
  
}

// ----------------------------------------------------------------------

uint8_t* Elevator::get_floor_in_q(const uint8_t _floor)
{
  if(_floorq && _floor > 0 && _floor <= _num_floors)
    for(uint8_t* p=_floorq; p<=(_floorq+_num_floors-1); ++p) {
      if(*p==_floor) return p;
    }
  return 0;
}

// ----------------------------------------------------------------------

int8_t Elevator::get_floor_index_in_q(const uint8_t _floor)
{
  if(_floorq && _floor > 0 && _floor <= _num_floors)
    for(size_t i=0; i<_num_floors; i++) {
      if(_floorq[i] == _floor) return (int8_t)i;
    }
  return -1;
}

// ----------------------------------------------------------------------

const bool Elevator::q_empty()
{
  for(uint8_t* p=_floorq; p<=(_floorq+_num_floors-1); ++p)
    if(*p) return false;
  return true;
}

// ----------------------------------------------------------------------

void Elevator::optimize_queue()
{
  /*
   * Reshuffles internal floor queue (_floorq) and updates next floor pointer (_next_floor)
   * to reflect the elevator logic.
   */
   
   if(!_next_floor) { // case 1 - next target is not set
    
    ; 
   }
   
   else { // case 2 - next target is set
    
    // determine preferable movement direction (where's next target with reference to current floor)
    uint8_t fl_offset = *_next_floor - _curr_floor;
    if(fl_offset > 0) { // going up
      
    }
    else if(fl_offset < 0) { // going down
      
    }
    else { // next = current
      if(_events.OnArrive) _events.OnArrive(this, _curr_floor);
      queue_advance(); // could this lead to infinite recursion?  
    }
    
   }
}

// ----------------------------------------------------------------------

const uint8_t Elevator::floors_to_visit()
{
  uint8_t counter = 0;
  for(uint8_t* p=_floorq; p<=(_floorq+_num_floors-1); ++p)
    if(*p) counter++;
  return counter;
}
