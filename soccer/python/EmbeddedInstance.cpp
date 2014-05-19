#include "EmbeddedInstance.hpp"
#include <boost/python.hpp>
#include <iostream>

using namespace std;
using namespace boost;


void EmbeddedInstance::initialize() {
    try {
        cout << "Initializing embedded python interpreter..." << endl;
        Py_Initialize();
    } catch (python::error_already_set) {
        PyErr_Print();
        throw new runtime_error("Unable to initialize embedded python interpreter");
    }
}
