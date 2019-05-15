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

void Elevq::resort()
{
	/* Resort elements in buff so that:
  *     1) all unassigned (=0) elements are at the end: [1, 2, 0, 3, 0, 5] >>> [1, 2, 3, 5, 0, 0]
  *     2) elements are sorted relative to current_el and next_el: [2, (5), 0, 3, 6, 1] >>> [6, (5), 3, 2, 1, 0]
	 */
 
}
