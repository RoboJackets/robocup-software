class IOExpanderDigitalOut
{
private:
    IOExpander m_exp;
public:
    IOExpanderDigitalOut(IOExpander exp)
    {
        m_exp = exp;
    }

    IOExpanderDigitalOut& operator= (int pin) {
        write(pin);
        return *this;
    }

    void write(int pin) {
        m_exp.set(pin);
    }

    void read(int pin) {
        m_exp.get
    }
};