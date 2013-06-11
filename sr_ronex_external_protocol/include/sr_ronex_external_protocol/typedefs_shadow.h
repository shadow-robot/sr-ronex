//
// © 2010 Shadow Robot Company Limited.
//
// FileName:        typedefs_shadow.h
// Dependencies:    
// Processor:       PSoC3, PSoC5
// Compiler:        C51
//
//  +------------------------------------------------------------------------+
//  | This file is part of The Shadow Robot PIC32 firmware code base.        |
//  |                                                                        |
//  | It is free software: you can redistribute it and/or modify             |
//  | it under the terms of the GNU General Public License as published by   |
//  | the Free Software Foundation, either version 3 of the License, or      |
//  | (at your option) any later version.                                    |
//  |                                                                        |
//  | It is distributed in the hope that it will be useful,                  |
//  | but WITHOUT ANY WARRANTY; without even the implied warranty of         |
//  | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          |
//  | GNU General Public License for more details.                           |
//  |                                                                        |
//  | You should have received a copy of the GNU General Public License      |
//  | along with this code repository. The text of the license can be found  |
//  | in Pic32/License/gpl.txt. If not, see <http://www.gnu.org/licenses/>.  |
//  +------------------------------------------------------------------------+
//
//
//
//
//  Doxygen
//  -------
//
//! @file
//
//! @group

#ifndef TYPEDEFS_SHADOW_H_INCLUDED
#define TYPEDEFS_SHADOW_H_INCLUDED

#include <limits.h>

/*
typedef enum _BOOL { FALSE = 0, TRUE } BOOL;  // Undefined size

#ifndef NULL
    #define NULL    ((void *)0)
#endif
*/
/*
#define	PUBLIC                                  // Function attributes
#define PROTECTED
#define PRIVATE   static
*/

// Processor & Compiler independent, size specific definitions
// To Do:  We need to verify the sizes on each compiler.  These
//         may be compiler specific, we should either move them
//         to "compiler.h" or #ifdef them for compiler type.
typedef signed char         int8s;
typedef signed short int    int16s;
typedef signed long int     int32s;
//typedef signed long long    int64s;

typedef unsigned char       int8u;
typedef unsigned short int  int16u;
typedef unsigned long int   int32u;  // other name for 32-bit integer
//typedef unsigned long long  int64u;


// Check that an int8 is 8 bits
//
/*
#if UCHAR_MAX !=  0xFF
    #error UCHAR_MAX wrong
#endif

#if UCHAR_MIN != 0
    #error CHAR_MIN wrong
#endif


// Check that an int16 is 16 bits
//
#if USHRT_MAX !=  0xFFFF
    #error UINT_MAX wrong
#endif

#if USHRT_MIN != 0
    #error UINT_MIN wrong
#endif


// Check that an int32 is 32 bits
//
#if UINT_MAX !=  0xFFFFFFFF
    #error UINT_MAX wrong
#endif

#if UINT_MIN != 0
    #error UINT_MIN wrong
#endif
*/

/// This is a macro which lets you declare function parameters as unused.
/// The benefits are:
///    It's nicer to type UNUSED(x) rather than __attribute__((unused))
///    It mangles the name, so you *can't* use it in the function
#ifdef UNUSED
#elif defined(__GNUC__)
    #define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
    #define UNUSED(x) /*@unused@*/ x
#else
    #define UNUSED(x) x
#endif


typedef union
{
    int16u U16;
    int16s S16;
    int8u  U8[2];
    int8s  S8[2];
}union16;

typedef union
{
    int32u U32;
    int32s S32;
    int16u U16[2];
    int16s S16[2];
    int8u  U8[4];
    int8s  S8[4];
}union32;

//#define SWAP_BYTES(u)   {u.U8[0]^=u.U8[1]; u.U8[1]^=u.U8[0]; u.U8[0]^=u.U8[1];}
#define SWAP_BYTES(u)   {int8u t; t=u.U8[0]; u.U8[0]=u.U8[1]; u.U8[1]=t;}
#define SWAP_ENDIANNESS(x)  (((x&0x000000FF)<<24) | ((x&0x0000FF00)<<8) | ((x&0xFF000000)>>24) | ((x&0x00FF0000)>>8))


int32u  bit_count_32(int32u n);
void    safe_string_copy(char* source, char* dest, int max_chars);
int32u  atoiu_safe(char *s, int32u max_chars);
void    swap_bytes(int16u *i);
int     typedef_tests(void);

//#define max(a, b) (((a)>(b)) ? (a) : (b))
//#define min(a, b) (((a)<(b)) ? (a) : (b))

#endif
