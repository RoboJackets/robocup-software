#include "PythonFunctionWrapper.hpp"
#include <iostream>

float cpp_function_cb(Geometry2d::Point p, PyObject* pyfunc) {
    if (pyfunc == nullptr) {
        std::cerr << "Pyfunction is null. Does the PythonFunctionWrapper " <<
                     "have the same lifetime as the NelderMead object?" << std::endl;

        return -1;
    }

    PyObject* pyresult =
        PyObject_CallObject(pyfunc, Py_BuildValue("ff", p.x(), p.y()));

    if (pyresult == NULL) {
        std::cerr << "Python callback function returned a bad value with args ";
        std::cerr << p << std::endl;

        return -1;
    }

    return PyFloat_AsDouble(pyresult);
}

PythonFunctionWrapper::PythonFunctionWrapper(PyObject* pf) {
    pyFunc = pf;

    Py_INCREF(pyFunc);

    f = std::bind(&cpp_function_cb,
                  std::placeholders::_1,
                  pyFunc);
}

PythonFunctionWrapper::~PythonFunctionWrapper() {
    Py_DECREF(pyFunc);
    pyFunc = nullptr;
}