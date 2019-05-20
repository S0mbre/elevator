#ifndef ELEVQ_H
#define ELEVQ_H

#include <Arduino.h>

#define ELTYPE uint8_t
#define CHECKRANGE(i) (i >= 0 && i < buffsz)
#define ZEROVAL 0
#ifndef max
  #define max(a,b) (a>=b? a : b)
#endif
#ifndef min
  #define min(a,b) (a<=b? a : b)
#endif

// ----------------------------------------------------------------------
namespace Elevatorns {
// ----------------------------------------------------------------------

enum Direction : uint8_t {UP, DOWN, STOP, NA};

// ----------------------------------------------------------------------

class Elevq
{
public:
	Elevq(const size_t Qsize=0, const ELTYPE& fill=ZEROVAL);
	~Elevq() { if(buff) free(buff); }
	
public:  
	ELTYPE* get(const size_t index) { return CHECKRANGE(index)? &buff[index] : 0; }
	bool set(const ELTYPE& el, const ELTYPE& value=ZEROVAL);
	ELTYPE* first() { return get(0); }
	ELTYPE* last() { return get(buffsz-1); }
  ELTYPE& current() { return current_el; } 
	//bool setCurrent(const ELTYPE& el) { return CHECKRANGE(el)? (current_el = el, true) : false; }
	const ELTYPE* next() { return next_el; }
	bool setNext(const ELTYPE& el);
	const size_t size() { return buffsz; }
	bool empty() { return buffsz == 0; }
  bool all_marked(const ELTYPE& value=ZEROVAL);
	void resize(const size_t Qsize, const ELTYPE& fill=ZEROVAL);
	ELTYPE* add(const ELTYPE& el);
  bool remove(const ELTYPE& el) { return set(el); }
	bool advance();
	bool has(const ELTYPE& el) { return find(el); }
	ELTYPE* find(const ELTYPE& el);
  const Direction getDirection() { return last_direction; }
  void setDirection(const Direction& _dir) { last_direction = _dir; }       // use at own discretion!
  void update_direction();
  void clear(const ELTYPE& value=ZEROVAL); 
  void resort();
	
protected:
	bool indexof(const ELTYPE& el, size_t& index);	
	ELTYPE* find_empty();
  	
protected:
	size_t buffsz;
	ELTYPE* buff;	
  ELTYPE current_el;
	ELTYPE* next_el;
  Direction last_direction;
	
private:
	void create_buff(ELTYPE* _buff, const size_t Qsize, const ELTYPE& fill=ZEROVAL);
  void qs(ELTYPE* _buff, size_t _from, size_t _to, const char _mode='^');
};

// ----------------------------------------------------------------------
} // Elevatorns

#endif
