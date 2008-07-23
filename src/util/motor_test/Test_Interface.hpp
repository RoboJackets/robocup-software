#ifndef _TEST_INTERFACE_HPP_
#define _TEST_INTERFACE_HPP_

#include "Labjack_U3.hpp"

class Test_Interface
{
public:
    typedef enum
    {
        Z = -1,
        L,
        H
    } Output;
    
    Test_Interface();
    
    void hall(int value);
    void motors_raw(float *value);
    void motors(Output *value, float *raw = 0);

protected:
    Labjack::U3 u3;
};

#endif // _TEST_INTERFACE_HPP_
