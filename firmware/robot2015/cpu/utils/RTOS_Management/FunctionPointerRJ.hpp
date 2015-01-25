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
#ifndef FUNCTION_POINTER_RJ_H
#define FUNCTION_POINTER_RJ_H

#include <string.h>
#include "RTP.hpp"

typedef void (*pvoidf_t)(void);

/** A class for storing and calling a pointer to a static or member void function
 */
class FunctionPointerRJ
{
public:

    /** Create a FunctionPointer, attaching a static function
     *
     *  @param function The void static function to attach (default is none)
     */
    FunctionPointerRJ(void (*function)(void) = 0);

    /** Create a FunctionPointer, attaching a member function
     *
     *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
     *  @param function The address of the void member function to attach
     */
    template<typename T>
    FunctionPointerRJ(T *object, void (T::*member)(void)) {
        attach(object, member);
    }

    /** Attach a static function
     *
     *  @param function The void static function to attach (default is none)
     */
    void attach(void(*function)(void) = 0);
    void attach(void(*function)(RTP_t*) = 0);
    void attach(void(*function)(void const*));

    /** Attach a member function
     *
     *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
     *  @param function The address of the void member function to attach
     */
    template<typename T>
    void attach(T *object, void (T::*member)(void)) {
        _object = static_cast<void*>(object);
        memcpy(_member, (char*)&member, sizeof(member));
        _membercaller = &FunctionPointerRJ::membercaller<T>;
        _function = 0;
    }

    /** Attach a member function
    *
    *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
    *  @param function The address of the void member function to attach
    */
    template<typename T>
    void attach(T *object, void (T::*member)(RTP_t*)) {
        _object = static_cast<void*>(object);
        memcpy(_member, (char*)&member, sizeof(member));
        _membercallerRTP = &FunctionPointerRJ::membercallerRTP<T>;
        _functionRTP = 0;
    }

    /** Call the attached static or member function
     */
    void call();
    void call(RTP_t*);
    void call(void const*);

    pvoidf_t get_function() const {
        return (pvoidf_t)_function;
    }

#ifdef MBED_OPERATORS
    void operator ()(void);
#endif

private:
    template<typename T>
    static void membercaller(void *object, char *member) {
        T* o = static_cast<T*>(object);
        void (T::*m)(void);
        memcpy((char*)&m, member, sizeof(m));
        (o->*m)();
    }

    template<typename T>
    static void membercallerRTP(void *object, char *member, RTP_t *p) {
        T* o = static_cast<T*>(object);
        void(T::*m)(RTP_t*);
        memcpy((char*)&m, member, sizeof(m));
        (o->*m)(p);

    }

    void (*_functionConst)(void const*);
    void (*_functionRTP)(RTP_t*);
    void (*_function)(void);             // static function pointer - 0 if none attached
    void *_object;                       // object this pointer - 0 if none attached
    char _member[16];                    // raw member function pointer storage - converted back by registered _membercaller
    void (*_membercaller)(void*, char*); // registered membercaller function to convert back and call _member on _object
    void (*_membercallerRTP)(void*, char*, RTP_t*);
};

#endif  // FUNCTIONPOINTER_RJ_H
