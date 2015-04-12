//  main.cpp

//  the iostream header contains cout among other things
#include <iostream>

//  this declaration lets us leave of the "std::" when using things in the standard libraries
//  without it, we'd have to type "std::cout", not we can just type "cout"
using namespace std;


int main(int argc, char **argv) {
    cout << "Hello World" << endl;

    //  return 0 to indicate success
    return 0;
}