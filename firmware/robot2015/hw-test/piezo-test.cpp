#include <mbed.h>
#include <rtos.h>

#include <array>

#include <helper-funcs.hpp>

#include "robot-devices.hpp"

DigitalOut good(LED1, 0);
DigitalOut bad1(LED2, 0);
DigitalOut bad2(LED3, 0);
DigitalOut pwr(LED4, 1);
Serial pc(RJ_SERIAL_RXTX);

AnalogOut buzzer(RJ_SPEAKER);

bool testPass = false;
bool batchedResult = false;

// number of points in the sine wave
const int numPts = 32;

// constexpr math functions based on https://github.com/pkeir/ctfft
// computation tolerance threshold
constexpr float tol = 0.001;

constexpr float cube_const(const float x) { return x * x * x; }

// Based on the triple-angle formula: sin 3x = 3 sin x - 4 sin ^3 x
constexpr float sin_helper(const float x) {
    return x < tol ? x : 3 * (sin_helper(x / 3.0)) -
                             4 * cube_const(sin_helper(x / 3.0));
}

constexpr float sin_const(const float x) {
    return sin_helper(x < 0 ? -x + 3.141592653589793 : x);
}

// by Xeo, from http://stackoverflow.com/a/13294458/420683
template <std::size_t... Is>
struct seq {};
template <std::size_t N, std::size_t... Is>
struct gen_seq : gen_seq<N - 1, N - 1, Is...> {};
template <std::size_t... Is>
struct gen_seq<0, Is...> : seq<Is...> {};

template <class Generator, std::size_t... Is>
constexpr auto generate_array_helper(Generator g, seq<Is...>)
    -> std::array<decltype(g(std::size_t{}, sizeof...(Is))), sizeof...(Is)> {
    return {{g(Is, sizeof...(Is))...}};
}

template <std::size_t tcount, class Generator>
constexpr auto generate_array(Generator g)
    -> decltype(generate_array_helper(g, gen_seq<tcount>{})) {
    return generate_array_helper(g, gen_seq<tcount>{});
}

// the generator for computing the sin wave elements
constexpr float my_generator(std::size_t curr, std::size_t total) {
    return ((1.0 +
             sin_const(static_cast<float>(curr) / numPts * 6.28318530717959)) /
            2.0);
}

int j = 0;

// compile time lookup table of sine wave elements - ranges from 0.0 to 1.0
const auto analog_data = generate_array<numPts>(my_generator);
auto analog_data_scaled = generate_array<numPts>(my_generator);

// used to output next analog sample whenever a timer interrupt occurs
void analog_out_interrupt(void const* args) {
    // send next analog sample out to D to A
    buzzer = analog_data.at(j);
    // increment pointer and wrap around back to 0 at 128
    j = (j + 1) & 0x07F;
}

// class method to play a note based on AnalogOut class
void play_note(float freq, int dur, float vol = 1.0) {
    // scale samples using current volume level arg
    for (int i = 0; i < numPts; i++) {
        analog_data_scaled.at(i) = vol * analog_data.at(i);
    }

    // reset to start of sample array
    j = 0;

    RtosTimer sin_wave(analog_out_interrupt, osTimerPeriodic);
    sin_wave.start(1.0 / (freq * numPts));

    // turn on timer interrupts to start sine wave output
    // Sample_Period.attach(this, &Speaker::Sample_timer_interrupt, 1.0 / (freq
    // * 32.0));

    // play note for specified time
    Thread::wait(dur);
    // turns off timer interrupts
    // Sample_Period.detach();

    sin_wave.stop();

    // sets output to mid range - analog zero
    // buzzer.write_u16(32768);
    buzzer.write(0);
}

int main() {
    pc.baud(57600);
    pc.printf("START========= STARTING TEST =========\r\n\r\n");

    pwr = 0;
    RtosTimer live_light(imAlive, osTimerPeriodic, (void*)&pwr);
    live_light.start(250);

    // Show the lookup table elements
    pc.printf("--  showing sin lookup table:\r\n");

    for (std::size_t i = 0; i != analog_data.size(); ++i) {
        pc.printf("%01.5f\t\t", analog_data.at(i));

        if (i % 4 == 3) pc.printf("\r\n");
    }

    play_note(969.0, 300, 0.01);
    Thread::wait(100);
    play_note(800.0, 300, 0.06);
    Thread::wait(100);
    play_note(920.0, 300, 0.075);
    play_note(0, 100, 0);

    // turn on timer interrupts to start sine wave output
    // sample rate is 500Hz
    // RtosTimer sine_wave(Sample_timer_interrupt, osTimerPeriodic);
    // sine_wave.start(1.0 / (500.0 * numPts));

    // Thread::wait(300);

    // now we stop the piezo from playing
    // sine_wave.stop();

    // Final results of the test
    testPass = analog_data.size() == numPts;

    pc.printf("\r\n=================================\r\n");
    pc.printf("========== TEST %s ==========\r\n",
              testPass ? "PASSED" : "FAILED");
    pc.printf("=================================DONE");

    // Turn on the corresponding LED(s)
    good = testPass;
    bad1 = !testPass;
    bad2 = !testPass;
    live_light.stop();
    pwr = testPass;

    while (true) {
        // Never return
        wait(2.0);
    }
}
