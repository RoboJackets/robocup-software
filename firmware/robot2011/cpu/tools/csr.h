#pragma once

/// Bitmap for all status bits in CSR.
#define REG_NO_EFFECT_1_ALL                                               \
    AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1 | AT91C_UDP_STALLSENT | \
        AT91C_UDP_RXSETUP | AT91C_UDP_TXCOMP

/// Clears the specified bit(s) in the UDP_CSR register.
/// \param endpoint The endpoint number of the CSR to process.
/// \param flags The bitmap to set to 1.
#define SET_CSR(endpoint, flags)                                         \
    {                                                                    \
        volatile unsigned int reg;                                       \
        reg = AT91C_BASE_UDP->UDP_CSR[endpoint];                         \
        reg |= REG_NO_EFFECT_1_ALL;                                      \
        reg |= (flags);                                                  \
        AT91C_BASE_UDP->UDP_CSR[endpoint] = reg;                         \
        while ((AT91C_BASE_UDP->UDP_CSR[endpoint] & (flags)) != (flags)) \
            ;                                                            \
    }

/// Sets the specified bit(s) in the UDP_CSR register.
/// \param endpoint The endpoint number of the CSR to process.
/// \param flags The bitmap to clear to 0.
#define CLEAR_CSR(endpoint, flags)                                       \
    {                                                                    \
        volatile unsigned int reg;                                       \
        reg = AT91C_BASE_UDP->UDP_CSR[endpoint];                         \
        reg |= REG_NO_EFFECT_1_ALL;                                      \
        reg &= ~(flags);                                                 \
        AT91C_BASE_UDP->UDP_CSR[endpoint] = reg;                         \
        while ((AT91C_BASE_UDP->UDP_CSR[endpoint] & (flags)) == (flags)) \
            ;                                                            \
    }
