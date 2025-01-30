#include <functional>
#include <Python.h>
#include <rj_geometry/point.hpp>


float cpp_function_cb(rj_geometry::Point p, PyObject* pyfunc);

class PythonFunctionWrapper {
public:
    PyObject* py_func;
    std::function<float(rj_geometry::Point)> f;

    PythonFunctionWrapper(PyObject* pf);

    ~PythonFunctionWrapper();
};