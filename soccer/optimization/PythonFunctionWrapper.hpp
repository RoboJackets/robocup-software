#include <functional>
#include <Python.h>
#include <Geometry2d/Point.hpp>


float cpp_function_cb(Geometry2d::Point p, PyObject* pyfunc);

class PythonFunctionWrapper {
public:
    PyObject* pyFunc;
    std::function<float(Geometry2d::Point)> f;

    PythonFunctionWrapper(PyObject* pf);

    ~PythonFunctionWrapper();
};