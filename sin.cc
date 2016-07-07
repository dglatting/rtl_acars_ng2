/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
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
 * $Log: sin.cc,v $
 * Revision 1.1  2016/07/04 19:15:32  dennisg
 * Initial revision
 *
 * Revision 1.12  2016/06/28 23:10:53  dennisg
 * Transitioned back from I/Q phase of 0 to pi/2.
 *
 * Revision 1.11  2016/06/28 03:54:25  dennisg
 * In an AM system the phase difference between I and Q is zero. It
 * would have been nice to have known that a long time ago.
 *
 * Revision 1.10  2016/06/26 23:50:16  dennisg
 * Fixed some comments.
 *
 * Revision 1.9 2016/06/26 23:44:13 dennisg 
 * Changed from implicit use of cos()/sin() for phase coherence of 90
 * degrees to sin()/sin() with an explicit phase coherence of 45
 * degrees. Changed the "c" and "s" structures to I/Q for better
 * visual description of what's going on. Finally, changed lots of
 * doubles to floats since the end math is float, and hopefully
 * eliminating unnecessary run-time conversions.
 *
 * Revision 1.8  2016/06/26 21:28:50  dennisg
 * Why did I touch this file? Why?
 *
 * Revision 1.7  2016/06/22 19:46:06  dennisg
 * Fixed comments.
 *
 * Revision 1.6  2016/03/14 23:04:45  dennisg
 * Added a RCS tag.
 *
 * Revision 1.5  2016/03/05 20:39:48  dennisg
 * Changed the template for BufferVOLK to be a templated type rather
 * than a fixed type. Of course this caused ripples throughout the
 * code. I also Redefined BufferCPLX as BufferVOLK with the type
 * lv_32fc_t.
 *
 * Revision 1.4  2016/02/28 23:38:19  dennisg
 * Dummy. Took out the last edits.
 *
 * Revision 1.3  2016/02/28 23:25:30  dennisg
 * I was calculating too many points for L 0/1 thereby forming a whole
 * wave rather than a half wave.
 *
 * Revision 1.2  2016/02/26 00:00:02  dennisg
 * Changed usage of BufferVOLK for complex signals to another type.
 *
 * Revision 1.1  2016/02/23 19:43:44  dennisg
 * Initial revision
 *
 */

#include <cmath>
#include <iostream>
#include <string>

#include <acars/sin.h>


static const std::string my_ident = "$Id: sin.cc,v 1.1 2016/07/04 19:15:32 dennisg Exp $";


namespace gr {
  namespace acars {

    static constexpr float twoPI  = ( 2.0 * M_PI );
    static constexpr float halfPI = ( M_PI / 2.0 );
    static constexpr float quarterPI = ( M_PI / 4.0 );
    
    BufferVOLK<lv_32fc_t> L0( size_t len ) {

      static constexpr float L = 1.0;
      
      BufferVOLK<lv_32fc_t> m( len );
      BufferVOLK<float>     I( len );

      const float T = float(len);
      
      for( size_t t = 0; t < len; ++t )
	I[t] = (  M_PI * float(t) / T );

      volk_32f_sin_32f( I.get(), I.get(), len );

      for( size_t i = 0; i < len; ++i )
	m[i] = lv_32fc_t( I[i] * L, I[i] * L );
      
      return m;
    }

    BufferVOLK<lv_32fc_t> L1( size_t len ) {

      static constexpr float L = -1.0;
      
      BufferVOLK<lv_32fc_t> m( len );
      BufferVOLK<float>     I( len );

      const float T = float(len);
      
      for( size_t t = 0; t < len; ++t )
	I[t] =  ( M_PI * float(t) / T );

      volk_32f_sin_32f( I.get(), I.get(), len );

      for( size_t i = 0; i < len; ++i )
	m[i] = lv_32fc_t( I[i] * L, I[i] * L );
      
      return m;
    }
    
    BufferVOLK<lv_32fc_t> H0( size_t len ) {

      static constexpr float H = -1.0;
      
      BufferVOLK<lv_32fc_t> m( len );
      BufferVOLK<float>     I( len );

      const float T = float(len);
      
      for( size_t t = 0; t < len; ++t ) 
	I[t] =  ( twoPI * float(t) / T );

      volk_32f_sin_32f( I.get(), I.get(), len );

      for( size_t i = 0; i < len; ++i )
	m[i] = lv_32fc_t( I[i] * H, I[i] * H );
      
      return m;
    }

    BufferVOLK<lv_32fc_t> H1( size_t len ) {

      static constexpr float H = 1.0;
      
      BufferVOLK<lv_32fc_t> m( len );
      BufferVOLK<float>     I( len );

      const float T = float(len);

      for( size_t t = 0; t < len; ++t )
	I[t] =  ( twoPI * float(t) / T );

      volk_32f_sin_32f( I.get(), I.get(), len );
      
      for( size_t i = 0; i < len; ++i )
	m[i] = lv_32fc_t( I[i] * H, I[i] * H );
      
      return m;
    }
    
  }
}


//  LocalWords:  BufferVOLK BufferCPLX
