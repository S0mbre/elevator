#include "elevq.h"

using namespace Elevatorns;
// ----------------------------------------------------------------------

Elevq::Elevq(const size_t Qsize, const ELTYPE& fill) :
	buffsz(Qsize), buff(0), current_el(0), next_el(0), last_direction(Direction::NA)
{
	if(Qsize) create_buff(buff, Qsize, fill);
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
	return true;
}

// ----------------------------------------------------------------------

void Elevq::update_direction()
{
  last_direction = Direction::STOP; 
  if(!current_el || !next_el || *next_el == ZEROVAL) return;
  
  long d = current_el - *next_el;  
  if(d>0) last_direction = Direction::DOWN;
  else if(d<0) last_direction = Direction::UP;
}

// ----------------------------------------------------------------------

ELTYPE* Elevq::add(const ELTYPE& el)
{
	if(el==ZEROVAL) return 0;
 
	if(empty()) {
    buff[0] = el;
    next_el = buff;
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

void Elevq::clear(const ELTYPE& value)
{
  if(buff && buffsz) {
    for(ELTYPE* p=buff; p<(buff+buffsz); ++p) *p = value;
    next_el = 0;
    last_direction = Direction::STOP;
  }
}

// ----------------------------------------------------------------------

bool Elevq::advance()
{
	if(!buff) return false;
	if(!current_el) current_el = buff[0];
  if(!next_el) next_el = buff;
  
	*next_el = ZEROVAL;
	resort();
  next_el = buff;
  	
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

  size_t j = 0;
  for(size_t i=0; i<buffsz; i++) {
    if(buff[i] == current_el) {
      _buff[j] = buff[i];
      j++;
      break;
    }
  }  

  if(last_direction == 'u') {

    // copy floors > current floor and sort them ascending    
    for(size_t i=0; i<buffsz; i++) {  
      if(buff[i] > current_el) {
        _buff[j] = buff[i]; 
        j++;
      }  
    }
    qs(_buff, 0, j-1, '^');
    
    // copy floors < current floor and sort them descending
    size_t k = j - 1;
    for(size_t i=0; i<buffsz; i++) {  
      if(buff[i] < current_el) {
        _buff[j] = buff[i]; 
        j++;
      }  
    }
    qs(_buff, k, j-1, 'v');

  }

  else if(last_direction == 'd') {

    // copy floors < current floor and sort them descending    
    for(size_t i=0; i<buffsz; i++) {  
      if(buff[i] < current_el) {
        _buff[j] = buff[i]; 
        j++;
      }  
    }
    qs(_buff, 0, j-1, 'v');

    // copy floors > current floor and sort them ascending
    size_t k = j - 1;
    for(size_t i=0; i<buffsz; i++) {  
      if(buff[i] > current_el) {
        _buff[j] = buff[i]; 
        j++;
      }  
    }
    qs(_buff, k, j-1, '^');
    
  }

  // replace buff
  free(buff);
  buff = _buff;

  next_el = buff;

  update_direction(); 
}

// ----------------------------------------------------------------------

void Elevq::qs(ELTYPE* _buff, size_t _from, size_t _to, const char _mode)
{
    // Quick sort. No range check!!!
    
    if (_from >= _to) return;
    
    ELTYPE left = _from, right = _to, middle = _buff[(left + right) / 2];
    do
    {
        while (_buff[left] < middle) left++;
        while (_buff[right] > middle) right--;
        if( (_mode=='^' && left <= right) || (_mode=='v' && left >= right) )
        {
            ELTYPE tmp = _buff[left];
            _buff[left] = _buff[right];
            _buff[right] = tmp;
            left++;
            right--;
        }
    } while (left <= right);
    qs(_buff, _from, right, _mode);
    qs(_buff, left, _to, _mode);    
}
