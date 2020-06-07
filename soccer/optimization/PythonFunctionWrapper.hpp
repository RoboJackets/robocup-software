#include <Python.h>
#include <geometry2d/point.h>

#include <functional>

float cpp_function_cb(geometry2d::Point p, PyObject* pyfunc);

class PythonFunctionWrapper {
public:
    PyObject* pyFunc;
    std::function<float(geometry2d::Point)> f;

    PythonFunctionWrapper(PyObject* pf);

    ~PythonFunctionWrapper();
};
