// list<TYPE> is implemented as a Python list.
template<TYPE>
%MappedType list<TYPE>
{
%TypeHeaderCode
#include <list>
using namespace std;
%End

%ConvertFromTypeCode
  // Create the list.
  PyObject *l;
  if ((l = PyList_New(sipCpp->size())) == NULL)
    return NULL;
  // Set the list elements.
  list<TYPE>::iterator iter;
  int i(0);
  for (iter = sipCpp->begin(); iter != sipCpp->end(); ++iter) {
    TYPE *t = new TYPE(*iter);
    PyObject *tobj;
    if ((tobj = sipConvertFromNewInstance(t, sipClass_TYPE, sipTransferObj)) == NULL) {
      Py_DECREF(l);
      delete t;
      return NULL;
    }
    PyList_SET_ITEM(l, i++, tobj);
  }
  return l;
%End

%ConvertToTypeCode
  // Check the type if that is all that is required.
  if (sipIsErr == NULL) {
    if (!PyList_Check(sipPy))
      return 0;
    for (int i = 0; i < PyList_GET_SIZE(sipPy); ++i)
      if (!sipCanConvertToInstance(PyList_GET_ITEM(sipPy, i), sipClass_TYPE, SIP_NOT_NONE))
        return 0;
    return 1;
  }
  list<TYPE> *qv = new list<TYPE>;
  for (int i = 0; i < PyList_GET_SIZE(sipPy); ++i) {
    int state;
    TYPE *t = reinterpret_cast<TYPE *>(sipConvertToInstance(PyList_GET_ITEM(sipPy, i), sipClass_TYPE, sipTransferObj, SIP_NOT_NONE, &state, sipIsErr));
    if (*sipIsErr) {
      sipReleaseInstance(t, sipClass_TYPE, state);
      delete qv;
      return 0;
    }
    qv->push_back(*t);
    sipReleaseInstance(t, sipClass_TYPE, state);
  }
  *sipCppPtr = qv;
  return sipGetState(sipTransferObj);
%End
};
