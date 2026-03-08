uint8 SHOOT_MODE_KICK = 0
uint8 SHOOT_MODE_CHIP = 1

uint8 TRIGGER_MODE_STAND_DOWN = 0
uint8 TRIGGER_MODE_IMMEDIATE = 1
uint8 TRIGGER_MODE_ON_BREAK_BEAM = 2

uint8 shoot_mode
uint8 trigger_mode

# In range [0, 1]
float32 kick_strength

# In range [-1, 1] -1 being away from us and 1 being towards us
float32 dribbler_speed
