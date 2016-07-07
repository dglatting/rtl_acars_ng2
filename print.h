/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 * 
 * Silly utility print routines.
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
 * $Log: print.h,v $
 * Revision 1.1  2016/07/04 19:15:54  dennisg
 * Initial revision
 *
 * Revision 1.5  2016/06/22 21:33:20  dennisg
 * Moved the x_array() functions from Receive to print.
 *
 * Revision 1.4  2016/03/14 22:06:49  dennisg
 * Moved the _LOG/_log functions into common code outside blocks.
 *
 * Revision 1.3  2016/03/05 20:39:48  dennisg
 * Changed the template for BufferVOLK to be a templated type rather
 * than a fixed type. Of course this caused ripples throughout the
 * code. I also Redefined BufferCPLX as BufferVOLK with the type
 * lv_32fc_t.
 *
 * Revision 1.2  2016/03/05 05:15:50  dennisg
 * Added printing for a complex array.
 *
 * Revision 1.1  2016/02/09 16:21:49  dennisg
 * Initial revision
 *
 */

#ifndef __ACARS_PRINT_H__
#define __ACARS_PRINT_H__

#include <complex>
#include <iostream>
#include <string>
#include <vector>

#include <boost/format.hpp>
#include <volk/volk.h>

#include <acars/Buffer.h>


void w_array( const boost::format&, const std::vector<float>& );
void w_array( const boost::format&, const gr::acars::BufferVOLK<float>& );
void w_array( const boost::format&, const gr::acars::BufferVOLK<lv_32fc_t>& );
void w_array( const boost::format&, const gr::acars::Buffer<float>& );
void w_array( const boost::format&, const std::vector<std::complex<float>>& );
void w_array( const boost::format&, const gr::acars::BufferFFT& );
  

// Stupid wrappers.

inline void
x_array( const char* fmt,
	 const std::vector<float>& v ) {
  
  w_array( boost::format( fmt ), v );
}

inline void
x_array( const char* fmt, uint tag,
	 const std::vector<float>& v ) {
  
  w_array( boost::format( fmt ) % tag, v );
}

inline void
x_array( const char* fmt,
	 const std::vector<std::complex<float>>& v ) {
  
  w_array( boost::format( fmt ), v );
}

inline void
x_array( const char* fmt, uint tag,
	 const std::vector<std::complex<float>>& v ) {
  
  w_array( boost::format( fmt ) % tag, v );
}

inline void
x_array( const char* fmt,
	 const gr::acars::BufferVOLK<float>& v ) {
  
  w_array( boost::format( fmt ), v );
}

inline void
x_array( const char* fmt, uint tag,
	 const gr::acars::BufferVOLK<float>& v ) {
  
  w_array( boost::format( fmt ) % tag, v );
}

inline void
x_array( const char* fmt,
	 const gr::acars::BufferVOLK<lv_32fc_t>& v ) {
  
  w_array( boost::format( fmt ), v );
}

inline void
x_array( const char* fmt, uint tag,
	 const gr::acars::BufferVOLK<lv_32fc_t>& v ) {
  
  w_array( boost::format( fmt ) % tag, v );
}

inline void
x_array( const char* fmt, uint tag,
	 const gr::acars::BufferFFT& v ) {
  
  w_array( boost::format( fmt ) % tag, v );
}


// This is a convenience macro wrapping the _log() function for
// reducing the about of garbage that has to be typed when calling
// _log().

#define _LOG(x) _log( name(), __FUNCTION__, x );

// Another convenience macro, built on the preceding macro, for
// sending log messages to stdout.

#define _SLOG(x) { std::cout << _LOG(x);}

// This is a common logging function. Its purpose is to form a log
// message suitable for sending to stdout the name of the block, the
// method in which the log was called, then followed by a formatted
// message.
//
// The usage of this function looks like:
//   std::cout << _log(foo)

const std::string
_log( const std::string& n, const char* f, const boost::format& m );


#endif


//  LocalWords:  stdout BufferVOLK BufferCPLX
