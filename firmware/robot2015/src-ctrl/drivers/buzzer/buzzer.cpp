#include "buzzer.hpp"

#include <array>

#include <helper-funcs.hpp>
#include <const-math.hpp>

/*
 * These must be extern here since conexpr is taken advantage
 * of to compute lookup tables at compile time. I feel like this
 * isn't the right way to do this though...
 */
extern constexpr double sin_const(const double);

// by Xeo, from http://stackoverflow.com/a/13294458/420683
template <std::size_t... Is>
struct seq {};

template <std::size_t N, std::size_t... Is>
struct gen_seq : gen_seq<N - 1, N - 1, Is...> {};

template <std::size_t... Is>
struct gen_seq<0, Is...> : seq<Is...> {};

template <class Generator, std::size_t... Is>
constexpr auto lut_generator_helper(Generator g, seq<Is...>)
    -> std::array<decltype(g(std::size_t{}, sizeof...(Is))), sizeof...(Is)> {
    return {{g(Is, sizeof...(Is))...}};
}

template <std::size_t tcount, class Generator>
constexpr auto lut_generator(Generator g)
    -> decltype(lut_generator_helper(g, gen_seq<tcount>{})) {
    return lut_generator_helper(g, gen_seq<tcount>{});
}

// number of points in the sine wave
const size_t numPts = 32;

// Definition for how the indexes of our lookup table are calculated.
//
// Many different sounds can be created by combining multiple singals
// and computing an array length that is the least common multiple of
// all of individual periods. Then change the indexing technique for playing
// different sounds from the single lookup table. For now...we have
// a basic sin wave.
//
// Check out /firmware/common2015/util/const-math.hpp for the math operations
// that could be used to generate a lookup table at compile time.
constexpr float wave_lut(std::size_t curr, std::size_t total) {
    return ((1.0 + sin_const(static_cast<float>(curr) / numPts * 2 * M_PI)) /
            2.0);
}

// lookup table for a sine wave - ranges from 0.0 to 1.0
const auto analog_data = lut_generator<numPts>(wave_lut);
auto analog_data_scaled = lut_generator<numPts>(wave_lut);

// used to output next analog sample whenever a timer interrupt occurs
void analogUpdate(void const* arg) {
    // grab a pointer to the buzzer object
    Buzzer* buzz = const_cast<Buzzer*>(reinterpret_cast<const Buzzer*>(arg));
    // cycle through the index value for the lookup table
    buzz->j = (buzz->j + 1) % numPts;
    // send next analog sample out to DAC
    buzz->write(analog_data_scaled.at(buzz->j));
}

// class method to play a note based on AnalogOut class
void Buzzer::play(float freq, int dur, float vol) {
    // we create a new array here where each value is scaled by some
    // value. This changes the buzzer's volume.
    for (size_t i = 0; i < numPts; i++)
        analog_data_scaled.at(i) = vol * analog_data.at(i);

    // reset our lut index
    j = 0;

    // setup an interrupt timer that updates the currenly set
    // value at the given duration. This depends on our playing
    // frequency's period and the number of elements in our lookup table.
    RtosTimer sin_wave(analogUpdate, osTimerPeriodic, (void*)this);
    sin_wave.start(1.0 / (freq * numPts));

    // keep the timer interrupt active for however long we
    // need to play this note. Disabling the interrupt once
    // we've waiting long enough
    Thread::wait(dur);
    sin_wave.stop();

    // sets output to mid range - analog zero
    write_u16(32768);
}
