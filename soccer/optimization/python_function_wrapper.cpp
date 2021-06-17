#include "python_function_wrapper.hpp"

#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

float cpp_function_cb(rj_geometry::Point p, PyObject* pyfunc) {
    if (pyfunc == nullptr) {
        SPDLOG_ERROR(
            "Pyfunction is null. Does the PythonFunctionWrapper have the same lifetime as the "
            "NelderMead object?");

        return -1;
    }

    PyObject* pyresult = PyObject_CallObject(pyfunc, Py_BuildValue("ff", p.x(), p.y()));

    if (pyresult == NULL) {
        SPDLOG_ERROR("Python callback function returned a bad value with args {}.", p);

        return -1;
    }

    return PyFloat_AsDouble(pyresult);
}

PythonFunctionWrapper::PythonFunctionWrapper(PyObject* pf) {
    py_func = pf;

    Py_INCREF(py_func);

    f = std::bind(&cpp_function_cb, std::placeholders::_1, py_func);
}

PythonFunctionWrapper::~PythonFunctionWrapper() {
    Py_DECREF(py_func);
    py_func = nullptr;
}