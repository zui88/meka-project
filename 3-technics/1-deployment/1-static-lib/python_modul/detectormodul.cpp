// it must be declared first and then c headers
#define PY_SSIZE_T_CLEAN   
#include <Python.h>

#include "detectFunction.h"

static PyObject *
detector_detect(PyObject *self)
{
    double x_, y_, width_, height_;
    float  score_;
    detectFunction(&x_, &y_, &width_, &height_, &score_);
    // giving out a tuple
    return Py_BuildValue(x_, y_, width_, height_, score_);
}