#include "elevator.h"

// ----------------------------------------------------------------------

using namespace Elevatorns;

// ----------------------------------------------------------------------

Elevator::Elevator(uint8_t floors, AccelStepper* elev_motor_driver, const ElevatorEvents& events) :
	_floorq(0), _floor_positions(0), _num_floors(0), _floor_position(0),
	_motor(elev_motor_driver), _mode(Mode::AUTO),  
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

void Elevator::set_floor_positions(const long* const positions)
{
  if(sizeof(positions) < (sizeof(long*) * _num_floors))
    return;
    
  for(size_t i=0; i<_num_floors; i++)
    _floor_positions[i] = positions[i];    
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
   
   if(!_floorq->add(target_floor)) return ErrorCode::E_Q_FULL;

   if(_events.OnAdd) _events.OnAdd(this, target_floor);

   // run
   if(!_is_running || _floorq->getDirection() == Direction::STOP) run();

   return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::remove_target(uint8_t target_floor)
{
  if(!_floorq) return ErrorCode::E_UNKNOWN;
  if(!_floorq->remove(target_floor)) return ErrorCode::E_FLOOR_NOT_FOUND;
  if(_events.OnRemove) _events.OnRemove(this, target_floor);
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move_to()
{
  const uint8_t* _next_floor = next_floor_ptr();
  if(_next_floor) return move_to(*_next_floor);
  return ErrorCode::E_Q_NEXTTARGET_EMPTY;
}

// ----------------------------------------------------------------------

ErrorCode Elevator::move_to(uint8_t target_floor)
{
	if(!_motor) return ErrorCode::E_NO_DRIVER;
	if(target_floor < 1 || target_floor > _num_floors) return ErrorCode::E_OUT_OF_RANGE;
  if((uint8_t)_mode > 1) return ErrorCode::E_WRONG_MODE; 
  if(_floor_positions[target_floor - 1] < 0) return ErrorCode::E_FLOOR_POS_UNASSIGNED; // todo: check that all floor positions > 0!
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
  if(!_floorq) return ErrorCode::E_UNKNOWN;
  if(!steps) return ErrorCode::E_OK;
  if((uint8_t)_mode > 1) return ErrorCode::E_WRONG_MODE;  
  if(_mode != Mode::MANUAL) set_current_mode(Mode::MANUAL);
  
  _motor->move(steps);
  _floorq->setDirection(steps<0? Direction::UP : Direction::DOWN);
  
  while(_motor->runSpeed() && !_suspended) {
    _floor_position = _motor->currentPosition();
    if(on_run) {      
      if(!on_run(this)) break;
    }
    //delay(Elevator::STEP_DELAY);
  }
  
  _motor->stop();
  _floorq->setDirection(Direction::STOP);
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
  if(!_floorq) return ErrorCode::E_UNKNOWN;
  
  bool running = false;
  ErrorCode move_result;
  _floorq->resort(); // todo: check if needed!

  _is_running = true;
  uint8_t* _next_floor = next_floor_ptr();
  
  while(_mode == Mode::AUTO && _next_floor && !_suspended) {
   
    move_result = move_to();

    if(move_result==ErrorCode::E_OUT_OF_RANGE) {
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
      update_current_floor();
  
      if(!running) {
        // motor has stopped
        _floorq->setDirection(Direction::STOP);  // todo: check if needed
        if(_floorq->current() == *_next_floor) {
          // landed at next target_pos
          if(_events.OnArrive) _events.OnArrive(this, _floorq->current());          
          if(_events.OnRemove) _events.OnRemove(this, *_next_floor);
          _floorq->advance();
        }
        break; // exit inner loop on motor stop
      }
      
      // running to next target
      if(_events.OnRun) _suspended = !_events.OnRun(this);      // update suspended status from callback      
      
      // motor step delay
      delay(Elevator::STEP_DELAY);      
    }
    
    _next_floor = next_floor_ptr();
  }

  if(!_next_floor && _events.OnEmptyQueue) _events.OnEmptyQueue(this);
  if(_suspended && _events.OnSuspend) _events.OnSuspend(this);

  _is_running = false;
  return ErrorCode::E_OK;
}

// ----------------------------------------------------------------------

void Elevator::clear_queue()
{
  if(_floorq) _floorq->clear();
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
  return open_door(current_floor(), immediate);
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
  return close_door(current_floor(), immediate);
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
  uint8_t _curr_floor = 0;
  if(!_motor) return ErrorCode::E_NO_DRIVER;
  _floor_position = _motor->currentPosition();
  for(uint8_t i=_num_floors; i>0; --i) {
    if(_floor_position <= _floor_positions[i-1]) {
      _curr_floor = i;
      break;
    }
  }
  if(_floorq) _floorq->current() = _curr_floor;
  return ErrorCode::E_OK;
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
