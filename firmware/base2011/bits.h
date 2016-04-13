#ifndef _BITS_H_
#define _BITS_H_

#define set_bit(var, bit) \
    { (var) |= 1 << (bit); }
#define clear_bit(var, bit) \
    { (var) &= ~(1 << (bit)); }

#endif  // _BITS_H_
