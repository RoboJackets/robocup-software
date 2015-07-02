/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "FunctionPointerRJ.hpp"

FunctionPointerRJ::FunctionPointerRJ(void (*function)(void)) :
    _function(),
    _object(),
    _membercaller()
{
    attach(function);
}

void FunctionPointerRJ::attach(void(*function)(void))
{
    _function = function;
    _object = 0;
}

void FunctionPointerRJ::attach(void(*function)(RTP_t*))
{
    _functionRTP = function;
    _object = 0;
}

void FunctionPointerRJ::attach(void(*function)(void const*))
{
    _functionConst = function;
    _object = 0;
}

void FunctionPointerRJ::call(void)
{
    if (_function) {
        _function();
    } else if (_object) {
        _membercaller(_object, _member);
    }
}

void FunctionPointerRJ::call(RTP_t* p)
{
    if (_functionRTP) {
        _functionRTP(p);
    } else if (_object) {
        _membercallerRTP(_object, _member, p);
    }
}

void FunctionPointerRJ::call(void const *arg)
{
    if (_functionConst) {
        _functionConst(arg);
    } else if (_object) {
        _membercaller(_object, _member);
    }
}

#ifdef MBED_OPERATORS
void FunctionPointerRJ::operator ()(void)
{
    call();
}
#endif

