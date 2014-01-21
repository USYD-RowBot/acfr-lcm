#ifndef __MEAS_QUEUE_H__
#define __MEAS_QUEUE_H__

#include <list>
#include <vector>

#include "meas.h"

namespace perls
{
    typedef std::list<Meas_ptr>::iterator meas_it;
    typedef std::list<Meas_ptr>::reverse_iterator meas_rit;

    class GenCallbackClass
    {
      public:
        virtual void handle (Meas_ptr m) = 0;
    };

    template <class CallbackType>
    class CallbackClass : public GenCallbackClass
    {
      public:
        CallbackClass (void (CallbackType::*handler_func)(Meas_ptr m), CallbackType *handler_obj)
            : _handler (handler_func), _handler_obj (handler_obj)
        {}
        
        void handle (Meas_ptr m)
        {
            (_handler_obj->*_handler) (m);
        }

      private:
        void (CallbackType::*_handler)(Meas_ptr m);
        CallbackType *_handler_obj;
    };

    class Meas_queue
    {
      public:
        Meas_queue () : _lag (0), _utime (0) {}
        Meas_queue (int64_t lag) : _lag (lag), _utime (0), _cb_ptr (0) {} 
        ~Meas_queue () {if (_cb_ptr) delete _cb_ptr;}

        template <class CallbackType>
        void set_callback (void (CallbackType::*func)(Meas_ptr m), CallbackType *obj)
        {
            _cb_ptr = new CallbackClass<CallbackType> (func, obj);
        }

        int size () const {return static_cast<int>(_queue.size ());}

        bool empty () const {return _queue.empty ();}

        void push (Meas_ptr meas);
        void push_pop (Meas_ptr meas);

        std::vector<Meas_ptr> remaining_queue ();

      private:
        int64_t _lag; // lag in microsecs
        int64_t _utime;

        GenCallbackClass *_cb_ptr;
        
        std::list<Meas_ptr> _queue;
    };
}

#endif // __MEAS_QUEUE_H__
