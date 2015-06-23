#pragma once

#include "CC1201.hpp"
#include "CC1201Defines.hpp"
#include "CC1201Config.hpp"

#include "logger.hpp"

/*
 * default configuration include.
 * 		-should be an RF studio header export
 * 		-*ALL* registers must be exported
 */
//#include "CC1201-920-2GFSK-100ksps.hpp"
//#include "CC1201-ARIBStd-920-4GFSK-500ksps.hpp"
 #include "smartrf_CC1201.h"
 
#if not defined(SMARTRF_RADIO_CC1201)
#error "A valid Smart RF register configuration file was not loaded."
#endif
