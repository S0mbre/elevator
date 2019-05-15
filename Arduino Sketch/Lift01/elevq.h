#ifndef ELEVQ_H
#define ELEVQ_H

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define ELTYPE uint8_t
#define CHECKRANGE(i) (i >= 0 && i < buffsz)
#define ZEROVAL 0

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
	bool setCurrent(const ELTYPE& el);
	const ELTYPE* next() { return next_el; }
	bool setNext(const ELTYPE& el);
	const size_t size() { return buffsz; }
	bool empty() { return buffsz == 0; }
	void resize(const size_t Qsize, const ELTYPE& fill=ZEROVAL);
	ELTYPE* add(const ELTYPE& el);
	bool advance();
	bool has(const ELTYPE& el) { return find(el); }
	ELTYPE* find(const ELTYPE& el);

public:
  volatile ELTYPE current_el;
	
protected:
	ELTYPE* add_after(const ELTYPE& el, const ELTYPE& el_after);
	ELTYPE* add_afteri(const ELTYPE& el, const size_t index_after);
	bool indexof(const ELTYPE& el, size_t& index);
	void resort();
	ELTYPE* find_empty();
  void update_direction();
	
protected:
	size_t buffsz;
	ELTYPE* buff;	
	ELTYPE* next_el;
  char last_direction;  // 's' = 'stop' / not assigned; 'u' = 'up'; 'd' = 'down'
	
private:
	void create_buff(ELTYPE* _buff, const size_t Qsize, const ELTYPE& fill=ZEROVAL);
};

// ----------------------------------------------------------------------

#endif