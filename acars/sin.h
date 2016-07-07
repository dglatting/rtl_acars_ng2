/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 * 
 *
 * A rather badly named file but this file provides the prototypes for
 * functions that build logical bit 0/1s for the 1220Hz and 2400Hz
 * tones.
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
 * $Log: sin.h,v $
 * Revision 1.1  2016/07/04 19:15:54  dennisg
 * Initial revision
 *
 * Revision 1.4  2016/03/06 08:30:18  dennisg
 * Receiver code compiles and runs. Doesn't work yet.
 *
 * Revision 1.3  2016/03/05 20:39:48  dennisg
 * Changed the template for BufferVOLK to be a templated type rather
 * than a fixed type. Of course this caused ripples throughout the
 * code. I also Redefined BufferCPLX as BufferVOLK with the type
 * lv_32fc_t.
 *
 * Revision 1.2  2016/02/26 00:00:02  dennisg
 * Changed usage of BufferVOLK for complex signals to another type.
 *
 * Revision 1.1  2016/02/23 19:44:43  dennisg
 * Initial revision
 *
 */

#ifndef __ACARS_SIN_H__
#define __ACARS_SIN_H__

#include <complex>
#include <vector>

#include <acars/Buffer.h>


namespace gr {
  namespace acars {

    // Return the sin wave at the indicated length (one time slice)
    // for the logical zero and one at the high and low tones.
    
    BufferVOLK<lv_32fc_t>
      L0( size_t len ), L1( size_t len ),
      H0( size_t len ), H1( size_t len );

  }
}

#endif


//  LocalWords:  BufferVOLK templated
