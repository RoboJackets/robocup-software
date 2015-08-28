/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// Implementation of several stdio.h methods, such as printf(), sprintf() and
/// so on. This reduces the memory footprint of the binary when using those
/// methods, compared to the libc implementation.
///
/// !Usage
///
/// Adds stdio.c to the list of file to compile for the project. This will
/// automatically replace libc methods by the custom ones.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <stdio.h>
#include <stdarg.h>

//------------------------------------------------------------------------------
//         Local Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Writes a string inside the given string.
// Returns the size of the written
// string.
// \param pStr  Storage string.
// \param pSource  Source string.
//------------------------------------------------------------------------------
static void PutString(const char* pSource) {
    while (*pSource != 0) {
        putchar(*pSource++);
    }
}

//------------------------------------------------------------------------------
// Writes an unsigned int inside the given string, using the provided fill &
// width parameters.
// Returns the size in characters of the written integer.
// \param pStr  Storage string.
// \param fill  Fill character.
// \param width  Minimum integer width.
// \param value  Integer value.
//------------------------------------------------------------------------------
void PutUnsignedInt(char fill, int width, unsigned int value) {
    // Take current digit into account when calculating width
    width--;

    // Recursively write upper digits
    if ((value / 10) > 0) {
        PutUnsignedInt(fill, width, value / 10);
    }
    // Write filler characters
    else {
        while (width > 0) {
            putchar(fill);
            width--;
        }
    }

    // Write lower digit
    putchar((value % 10) + '0');
}

//------------------------------------------------------------------------------
// Writes a int inside the given string, using the provided fill & width
// parameters.
// Returns the size of the written integer.
// \param pStr  Storage string.
// \param fill  Fill character.
// \param width  Minimum integer width.
// \param value  Signed integer value.
//------------------------------------------------------------------------------
void PutSignedInt(char fill, int width, int value) {
    unsigned int absolute;

    // Compute absolute value
    if (value < 0) {
        absolute = -value;
    } else {
        absolute = value;
    }

    // Take current digit into account when calculating width
    width--;

    // Recursively write upper digits
    if ((absolute / 10) > 0) {
        if (value < 0) {
            PutSignedInt(fill, width, -(absolute / 10));
        } else {
            PutSignedInt(fill, width, absolute / 10);
        }
    } else {
        // Reserve space for sign
        if (value < 0) {
            width--;
        }

        // Write filler characters
        while (width > 0) {
            putchar(fill);
            width--;
        }

        // Write sign
        if (value < 0) {
            putchar('-');
        }
    }

    // Write lower digit
    putchar((absolute % 10) + '0');
}

//------------------------------------------------------------------------------
// Writes an hexadecimal value into a string, using the given fill, width &
// capital parameters.
// Returns the number of char written.
// \param pStr  Storage string.
// \param fill  Fill character.
// \param width  Minimum integer width.
// \param maj  Indicates if the letters must be printed in lower- or upper-case.
// \param value  Hexadecimal value.
//------------------------------------------------------------------------------
static void PutHexa(char fill, int width, unsigned char maj,
                    unsigned int value) {
    // Decrement width
    width--;

    // Recursively output upper digits
    if ((value >> 4) > 0) {
        PutHexa(fill, width, maj, value >> 4);
    }
    // Write filler chars
    else {
        while (width > 0) {
            putchar(fill);
            width--;
        }
    }

    // Write current digit
    if ((value & 0xF) < 10) {
        putchar((value & 0xF) + '0');
    } else if (maj) {
        putchar((value & 0xF) - 10 + 'A');
    } else {
        putchar((value & 0xF) - 10 + 'a');
    }
}

//------------------------------------------------------------------------------
//         Global Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Outputs a formatted string on the DBGU stream, using a variable number of
/// arguments.
/// \param pFormat  Format string.
//------------------------------------------------------------------------------
int printf(const char* pFormat, ...) {
    va_list ap;
    char fill;
    unsigned char width;

    // Forward call to vprintf
    va_start(ap, pFormat);

    // Phase string
    while (*pFormat != 0) {
        // Normal character
        if (*pFormat != '%') {
            putchar(*pFormat++);
        }
        // Escaped '%'
        else if (*(pFormat + 1) == '%') {
            putchar('%');
            pFormat += 2;
        }
        // Token delimiter
        else {
            fill = ' ';
            width = 0;
            pFormat++;

            // Parse filler
            if (*pFormat == '0') {
                fill = '0';
                pFormat++;
            }

            // Parse width
            while ((*pFormat >= '0') && (*pFormat <= '9')) {
                width = (width * 10) + *pFormat - '0';
                pFormat++;
            }

            // Parse type
            switch (*pFormat) {
                // FIXME - Division is broken
                //             case 'i':
                case 'd':
                    PutSignedInt(fill, width, va_arg(ap, int));
                    break;
                //             case 'u': PutUnsignedInt(fill, width, va_arg(ap,
                //             unsigned int)); break;
                case 'x':
                    PutHexa(fill, width, 0, va_arg(ap, unsigned int));
                    break;
                case 'X':
                    PutHexa(fill, width, 1, va_arg(ap, unsigned int));
                    break;
                case 's':
                    PutString(va_arg(ap, char*));
                    break;
                case 'c':
                    putchar(va_arg(ap, unsigned int));
                    break;
                case 'p':
                    putchar('0');
                    putchar('x');
                    PutHexa('0', 8, 0, va_arg(ap, unsigned int));
                    break;
                default:
                    break;
            }

            pFormat++;
        }
    }
    va_end(ap);

    return 1;
}
