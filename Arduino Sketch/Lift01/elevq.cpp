#include "elevq.h"

// ----------------------------------------------------------------------

Elevq::Elevq(const size_t Qsize, const ELTYPE& fill) :
	buffsz(Qsize), buff(0), current_el(0), next_el(0), last_direction('s')
{
	create_buff(buff, Qsize, fill);
}

// ----------------------------------------------------------------------

void Elevq::create_buff(ELTYPE* _buff, const size_t Qsize, const ELTYPE& fill)
{
	if(_buff) free(_buff);
	_buff = (ELTYPE*) malloc(Qsize * sizeof(ELTYPE));
	for(ELTYPE* p=_buff; p<=(_buff+Qsize-1); ++p) *p = fill;
}

// ----------------------------------------------------------------------

void Elevq::resize(const size_t Qsize, const ELTYPE& fill)
{
	ELTYPE* _buff;
	create_buff(_buff, Qsize, fill);
	if(buff) {
		size_t min_num = (Qsize <= buffsz)? Qsize : buffsz;
		for(size_t i=0; i<min_num; i++) _buff[i] = buff[i];
		free(buff);
	}
	buff = _buff;
	buffsz = Qsize;
}

// ----------------------------------------------------------------------

bool Elevq::setNext(const ELTYPE& el)
{
	ELTYPE* found_el = find(el);
	if(!found_el) return false;
	next_el = found_el;
	return true
}

// ----------------------------------------------------------------------

void Elevq::update_direction()
{
  last_direction = 's'; 
  if(!current_el || !next_el) return;
  
  long d = current_el - *next_el;  
  if(d>0) last_direction = 'd';
  else if(d<0) last_direction = 'u';
}

// ----------------------------------------------------------------------

ELTYPE* Elevq::add(const ELTYPE& el)
{
	if(el==ZEROVAL) return 0;
 
	if(empty()) {
    buff[0] = el;
    next_el = &buff[0];
    update_direction();
    return next_el;
	}
 
	ELTYPE* found_el = find(el);
	if(found_el) return found_el;
 
	found_el = find_empty();
	if(!found_el) return 0;
 
	*found_el = el;
	resort();
	return find(el);
}

// ----------------------------------------------------------------------

bool Elevq::set(const ELTYPE& el, const ELTYPE& value)
{
	ELTYPE* found_el = find(el);
	if(!found_el) return false;
	*found_el = value;
	resort();
	return true;
}

// ----------------------------------------------------------------------

bool Elevq::advance()
{
	if(!buff) return false;
	if(!current_el) current_el = &buff[0];
	*current_el = ZEROVAL;
	resort();
	next_el = &buff[0];
	return *next_el != ZEROVAL;
}

// ----------------------------------------------------------------------

ELTYPE* Elevq::find(const ELTYPE& el)
{
	for(ELTYPE* p=buff; p<(buff+buffsz); ++p)
		if(*p == el) return p;
	return 0;
}

// ----------------------------------------------------------------------

void Elevq::shift_buffer(ELTYPE* _buff, ELTYPE* _from, const char _dir, 
                  const size_t bufsize, const size_t _times, const ELTYPE& fill)
{
  if(_dir == 'u') {     // shift up
    size_t _maxtimes = min( (bufsize*sizeof(ELTYPE) - _from)/sizeof(ELTYPE), _times);
    for(size_t i=0; i<_maxtimes; i++) {
      for(ELTYPE* p=(_buff+bufsize-1); p!=_from; --p) *p = *(p-1);
      *_from = fill;
    }
  }
  else if(_dir == 'd') {     // shift up
    size_t _maxtimes = min( _from/sizeof(ELTYPE), _times);
    for(size_t i=0; i<_maxtimes; i++) {
      for(ELTYPE* p=_buff; p!=_from; ++p) *p = *(p+1);
      *_from = fill;
    }
  }
}

// ----------------------------------------------------------------------

void Elevq::move_buffer(ELTYPE* _buff, ELTYPE* _from, ELTYPE* _to)
{
  size_t diff = _from - _to;
  if(!diff) return;
  ELTYPE fr = *_from;
  if(diff < 0) for(ELTYPE* p=_from, p!=_to; ++p) *p = *(p+1);
  else for(ELTYPE* p=_from, p!=_to; --p) *p = *(p-1);    
  *_to = fr;
}

// ----------------------------------------------------------------------

void Elevq::move_buffer(ELTYPE* _buff, const size_t index_from, const size_t index_to)
{
  // !!! No range check !!!
  move_buffer(_buff, &_buff[index_from], &_buff[index_to]);
}

// ----------------------------------------------------------------------

ELTYPE* Elevq::insert_buffer(ELTYPE* _buff, const size_t bufsize, const ELTYPE& _new, 
                             ELTYPE* _existing, const char _mode)
{
  if(_mode == 'a' && _existing>=(_buff+bufsize-1))
    return 0; // can't insert after last

  if(_mode == 'a') {
    move_buffer(_buff, _buff + bufsize - 1, _existing + 1);
    *(_existing + 1) = _new;
  }
  else if(_mode == 'b') {
    move_buffer(_buff, _existing, _existing + 1);
    *_existing = _new;
  }
}

// ----------------------------------------------------------------------

ELTYPE* Elevq::add_after(const ELTYPE& el, const ELTYPE& el_after)
{
	ELTYPE* found_el_after = find(el_after);
	if(!found_el_after) return 0;
	
	ELTYPE* found_el = find(el);
	if(found_el) {
		*found_el = ZEROVAL;
		resort();
		return add_after(el, el_after);
	}
	
	if(buffsz < 2 || *last() != ZEROVAL) return 0;
	
	for(ELTYPE* p=(buff+buffsz-1); p!=found_el_after; --p) 
		*p = *(p-1);
	*(found_el_after + 1) = el;
	resort();
	return find(el);
}

// ----------------------------------------------------------------------

ELTYPE* Elevq::add_afteri(const ELTYPE& el, const size_t index_after)
{
	if(index_after >= (buffsz - 1)) return 0;
	return add_after(buff[index_after]);
}

// ----------------------------------------------------------------------

bool Elevq::indexof(const ELTYPE& el, size_t& index)
{
	for(size_t i=0; i<buffsz; i++)
		if(buff[i]==el) {
			index = i;
			return true;
		}
	return false;
}

// ----------------------------------------------------------------------

ELTYPE* Elevq::find_empty()
{
	for(ELTYPE* p=buff; p<(buff+buffsz); ++p)
		if(*p == ZEROVAL) return p;
	return 0;
}

// ----------------------------------------------------------------------

bool Elevq::all_marked(const ELTYPE& value)
{
  for(ELTYPE* p=buff; p<(buff+buffsz); ++p)  
    if(*p != value) return false;
  return true;
}

// ----------------------------------------------------------------------

void Elevq::resort()
{
	/* Resort elements in buff so that:
  *     1) all unassigned (=0) elements are at the end: [1, 2, 0, 3, 0, 5] >>> [1, 2, 3, 5, 0, 0]
  *     2) elements are sorted relative to current_el and next_el: [2, (5), 0, 3, 6, 1] >>> [6, (5), 3, 2, 1, 0]
	 */
  if(!buff || empty() || all_marked()) return;

  update_direction();
  ELTYPE* _buff;
  create_buff(_buff, buffsz);

  if(last_direction == 'u') {
    
    for(size_t i=0, j=0; i<buffsz; i++) {  
      if(buff[i] < current_el) continue;  
      if(_buff[j] == ZEROVAL) _buff[j] = buff[i];
      else if(_buff[j] < buff[i]) 
      
      
      
    }

  }
 
}
