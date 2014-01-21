#include "meas_queue.h"

void perls::Meas_queue::push (Meas_ptr m)
{
    meas_it it = _queue.begin ();
    while (it != _queue.end () && (*it)->utime () > m->utime ()) ++it;
    if (it == _queue.begin ())
        _utime = m->utime ();
    _queue.insert (it, m);
}

void perls::Meas_queue::push_pop (Meas_ptr m)
{
    push (m);

    meas_rit rit = _queue.rbegin ();
    while (rit != _queue.rend () && (*rit)->utime () <= (_utime - _lag)) {
        _cb_ptr->handle (*rit);
        _queue.pop_back ();
        rit = _queue.rbegin ();
    }
}

std::vector<perls::Meas_ptr> perls::Meas_queue::remaining_queue () 
{
    std::vector<Meas_ptr> v (_queue.size ());
    if (!_queue.size ()) return v;

    unsigned int i = 0;
    meas_rit rit = _queue.rbegin ();
    while (rit != _queue.rend ()) {
        v[i] = *rit;
        ++rit;
        ++i;
    }
    return v;
}
