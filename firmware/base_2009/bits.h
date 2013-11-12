#pragma once

#define set_bit(var, bit)   {(var) |= 1 << (bit);}
#define clear_bit(var, bit) {(var) &= ~(1 << (bit));}
