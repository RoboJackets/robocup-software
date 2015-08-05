#pragma once

#include "CC1201.hpp"
#include "CC1201Defines.hpp"
#include "CC1201Config.hpp"

/*
 * default configuration include.
 * 		-should be an RF studio header export
 * 		-*ALL* registers must be exported
 */
#include "cc1201_rj_config_920mhz.hpp"
 
#ifndef SMARTRF_RADIO_CC1201
#error "A valid Smart RF register configuration file was not loaded."
#endif
