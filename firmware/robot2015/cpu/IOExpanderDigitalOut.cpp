#include "IOExpanderDigitalOut.h"

IOExpanderDigitalOut::IOExpanderDigitalOut(MCP23017 *exp, IOExpanderPin pin)
{
    m_expander = exp;
    m_pin = pin;
}

int IOExpanderDigitalOut::read()
{
    return m_expander->read_bit(m_pin);
}

void IOExpanderDigitalOut::write(int const val)
{
    m_expander->write_bit(val, m_pin);
}