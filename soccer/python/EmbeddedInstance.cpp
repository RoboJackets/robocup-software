#include "EmbeddedInstance.hpp"
#include <boost/python.hpp>
#include <iostream>

#include "robocup-py.hpp"

using namespace std;
using namespace boost::python;


void EmbeddedInstance::initialize() {
    try {
        cout << "Initializing embedded python interpreter..." << endl;
        
        //  this tells python how to load the robocup module
        //  it has to be done before Py_Initialize()
        PyImport_AppendInittab("robocup", &PyInit_robocup);


        Py_Initialize();


        object main_module((handle<>(borrowed(PyImport_AddModule("__main__")))));
        object main_namespace = main_module.attr("__dict__");

        object robocup_module((handle<>(PyImport_ImportModule("robocup"))));
        main_namespace["robocup"] = robocup_module;

        handle<>ignored((PyRun_String("print(\"Hello world\")\np = robocup.Point(1, 2)\nprint(p)",
            Py_file_input,
            main_namespace.ptr(),
            main_namespace.ptr())));

    } catch (error_already_set) {
        PyErr_Print();
        throw new runtime_error("Unable to initialize embedded python interpreter");
    }
}
