// EIO 0,1,2 is hall 1,2,3
// EIO 3,4,5 is motor C,B,A

#include "Test_Interface.hpp"

Test_Interface::Test_Interface()
{
    u3.open();
    
    // Set EIO3-4 as analog inputs
    uint8_t cmd[] =
    {
        0, 0xf8, 0x03, 0x0b, 0, 0,
        8, 0, 0, 0, 0x0f, 0x78
    };
    uint8_t resp[12];
    u3.command(cmd, sizeof(cmd), resp, sizeof(resp));
}

void Test_Interface::hall(int value)
{
    uint8_t cmd[] =
    {
        0, 0xf8, 4, 0, 0, 0, 0,
        27, 0, 7, 0, 0, value, 0
    };
    uint8_t resp[10];
    u3.command(cmd, sizeof(cmd), resp, sizeof(resp));
}

void Test_Interface::motors_raw(float *value)
{
    uint8_t cmd[] =
    {
        0, 0xf8, 5, 0, 0, 0, 0,
        1, 11, 31, 1, 12, 31, 1, 13, 31
    };
    uint8_t resp[16];
    u3.command(cmd, sizeof(cmd), resp, sizeof(resp));
    
    value[2] = (resp[10] * 256 + resp[9]) / 65535.0 * 2.44;
    value[1] = (resp[12] * 256 + resp[11]) / 65535.0 * 2.44;
    value[0] = (resp[14] * 256 + resp[13]) / 65535.0 * 2.44;
}

void Test_Interface::motors(Output *value, float *raw)
{
    float v[3];
    motors_raw(v);
    
    for (int i = 0; i < 3; ++i)
    {
        if (v[i] < 0.95)
        {
            value[i] = L;
        } else if (v[i] > 2.0)
        {
            value[i] = H;
        } else {
            value[i] = Z;
        }
    }
    
    if (raw)
    {
        for (int i = 0; i < 3; ++i)
        {
            raw[i] = v[i];
        }
    }
}
