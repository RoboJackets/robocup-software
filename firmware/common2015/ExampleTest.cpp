#include <gtest/gtest.h>

// Example unit test
TEST(ExampleTest, example) { EXPECT_EQ(true, !false); }

#include "../robot2015/src-ctrl/drivers/rotarySelector/RotarySelector.hpp"

class FakeDigitalIn {
public:
    FakeDigitalIn(int value) : _value(value){

    }


    int read() const {
        return _value;
    }

private:
    int _value;
};

TEST(RotarySelector, read) {
    RotarySelector<FakeDigitalIn> selector({
        FakeDigitalIn(1),
        FakeDigitalIn(1),
        FakeDigitalIn(0),
        FakeDigitalIn(1),
    });
    EXPECT_EQ(11, selector.read());
}
