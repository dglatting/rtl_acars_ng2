/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 * 
 * CRC interface.
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
 *
 * $Log: crc.h,v $
 * Revision 1.1  2016/07/06 05:26:21  dennisg
 * Initial revision
 *
 * Revision 1.4  2016/03/02 05:29:17  dennisg
 * Working check-in.
 *
 * Revision 1.3  2016/02/29 06:42:26  dennisg
 * Changed CRC code to be table driven, now that I have confidence in
 * that. Added alternative polynomials based on various other texts
 * and code. Added optional padding to generation and checking but it
 * isn't clear if that is necessary or another implementation is
 * buggy; which it is.
 *
 * Revision 1.2  2016/02/29 04:01:30  dennisg
 * Fixed several bugs related to CRC calculation and Tx buffer
 * content.
 *
 * Revision 1.1  2016/02/19 05:39:51  dennisg
 * Initial revision
 *
 */

#ifndef __ACARS_CRC_H__
#define __ACARS_CRC_H__

#include <vector>

extern "C" {

#include <stdint.h>

}


uint16_t
gen_crc( std::vector<uint8_t>::const_iterator s,
	 std::vector<uint8_t>::const_iterator e );

bool
check_crc( std::vector<uint8_t>::const_iterator s,
	   std::vector<uint8_t>::const_iterator e );
  
#endif


//  LocalWords:  CRC
