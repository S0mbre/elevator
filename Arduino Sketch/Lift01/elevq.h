#ifndef ELEVQ_H
#define ELEVQ_H

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define ELTYPE uint8_t
#define CHECKRANGE(i) (i >= 0 && i < buffsz)
#define ZEROVAL 0
#define max(a,b) (a>=b? a : b)
#define min(a,b) (a<=b? a : b)

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
  bool all_marked(const ELTYPE& value=ZEROVAL);
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
  void shift_buffer(ELTYPE* _buff, ELTYPE* _from, const char _dir, 
                    const size_t bufsize, const size_t _times=1, const ELTYPE& fill=ZEROVAL);
  void move_buffer(ELTYPE* _buff, ELTYPE* _from, ELTYPE* _to);
  void move_buffer(ELTYPE* _buff, const size_t index_from, const size_t index_to);
  ELTYPE* insert_buffer(ELTYPE* _buff, const size_t bufsize, const ELTYPE& _new, ELTYPE* _existing, const char _mode='a');
};

// ----------------------------------------------------------------------

#endif
