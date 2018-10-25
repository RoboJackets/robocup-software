#include <functional>
#include <Python.h>
#include <Geometry2d/Point.hpp>


float cpp_function_cb(Geometry2d::Point p, PyObject* pyfunc) {
    if (pyfunc == nullptr) {
        std::cerr << "Pyfunction is null. Does the PythonFunctionWrapper have the same lifetime as the NelderMead object?"

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

class PythonFunctionWrapper {
public:
    PyObject* pyFunc;
    std::function<float(Geometry2d::Point)> f;

    PythonFunctionWrapper(PyObject* pf) {
        pyFunc = pf;

        Py_INCREF(pyFunc);

        f = std::bind(&cpp_function_cb,
                      std::placeholders::_1,
                      pyFunc);
    }

    ~PythonFunctionWrapper() {
        Py_DECREF(pyFunc);
        pyFunc = nullptr;
    }

    std::function<float(Geometry2d::Point)> getFunction() {
        return f;
    }

};