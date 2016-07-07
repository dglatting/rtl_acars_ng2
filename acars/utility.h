/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 *
 * Silly utility routines.
 *
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 *
 * $Log: utility.h,v $
 * Revision 1.1  2016/07/04 19:15:54  dennisg
 * Initial revision
 *
 * Revision 1.1  2016/02/29 04:00:42  dennisg
 * Initial revision
 *
 */

#ifndef __INCLUDE_ACARS_UTILITY_H__
#define __INCLUDE_ACARS_UTILITY_H__

#include <vector>

extern "C" {

#include <stdint.h>
  
}


// Return the value of bit 7 based on the odd parity of the index.

extern const std::vector<uint8_t> parity_bit;


// Convert the 7bit character to odd party, ignoring bit7.

inline uint8_t
_to_odd( uint8_t c ) {
  
  return c | parity_bit[ c & 0x7f ];
}

// Given a byte MSB..LSB, return the byte LSB..MSB

extern const std::vector<uint8_t> reverse_bits;
  

#endif

