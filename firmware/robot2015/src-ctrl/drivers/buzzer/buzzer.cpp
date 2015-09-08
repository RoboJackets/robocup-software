#include "buzzer.hpp"

#include <array>

#include <helper-funcs.hpp>

// constexpr math functions based on https://github.com/pkeir/ctfft
// computation tolerance threshold
constexpr float tol = 0.001;

constexpr
float cube_const(const float x)
{
    return x * x * x;
}

// Based on the triple-angle formula: sin 3x = 3 sin x - 4 sin ^3 x
constexpr
float sin_helper(const float x)
{
    return x < tol ? x : 3 * (sin_helper(x / 3.0)) - 4 * cube_const(sin_helper(x / 3.0));
}

constexpr
float sin_const(const float x)
{
    return sin_helper(x < 0 ? -x + 3.141592653589793 : x);
}

// by Xeo, from http://stackoverflow.com/a/13294458/420683
template<std::size_t... Is> struct seq {};
template<std::size_t N, std::size_t... Is>
struct gen_seq : gen_seq < N - 1, N - 1, Is... > {};
template<std::size_t... Is>
struct gen_seq<0, Is...> : seq<Is...> {};

template<class Generator, std::size_t... Is>
constexpr auto generate_array_helper(Generator g, seq<Is...>)
-> std::array < decltype(g(std::size_t{}, sizeof...(Is))), sizeof...(Is) > {
    return {{g(Is, sizeof...(Is))...}};
}

template<std::size_t tcount, class Generator>
constexpr auto generate_array(Generator g)
-> decltype( generate_array_helper(g, gen_seq<tcount> {}) )
{
    return generate_array_helper(g, gen_seq<tcount> {});
}

// number of points in the sine wave
const int numPts = 32;

// the generator for computing the sin wave elements
constexpr float my_generator(std::size_t curr, std::size_t total)
{
    return ((1.0 + sin_const(static_cast<float>(curr) / numPts * 6.28318530717959)) / 2.0);
}

// compile time lookup table of sine wave elements - ranges from 0.0 to 1.0
const auto analog_data = generate_array<numPts>(my_generator);
auto analog_data_scaled = generate_array<numPts>(my_generator);

// used to output next analog sample whenever a timer interrupt occurs
void analogUpdate(void const* args)
{
    Buzzer* buzz = (Buzzer*)args;

    // increment pointer and wrap around back to 0 at 128
    int j = buzz->getIndex();

    // send next analog sample out to D to A
    buzz->write(analog_data.at(j));

    j = ( (j == (numPts - 1)) ? 0 : (j + 1) );

    buzz->setIndex(j);
}

// class method to play a note based on AnalogOut class
void Buzzer::play(float freq, int dur, float vol)
{
    // scale samples using current volume level arg
    for (int i = 0; i < numPts; i++) {
        analog_data_scaled.at(i) = vol * analog_data.at(i);
    }

    // reset to start of sample array
    j = 0;

    RtosTimer sin_wave(analogUpdate, osTimerPeriodic, (void*)this);
    sin_wave.start(1.0 / (freq * numPts));

    // play note for specified time
    Thread::wait(dur);
    sin_wave.stop();

    // sets output to mid range - analog zero
    write_u16(32768);
    //buzzer.write(0);
}

int Buzzer::getIndex(void)
{
    return j;
}

void Buzzer::setIndex(int newIndex)
{
    j = newIndex;
}
