/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 * 
 * Utility print routines.
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
 * $Log: print.cc,v $
 * Revision 1.1  2016/07/04 19:15:32  dennisg
 * Initial revision
 *
 * Revision 1.8  2016/06/22 20:00:39  dennisg
 * Was missing print for BufferVOLK<complex>.
 *
 * Revision 1.7  2016/06/22 19:46:06  dennisg
 * Fixed comments.
 *
 * Revision 1.6  2016/03/14 23:04:45  dennisg
 * Added a RCS tag.
 *
 * Revision 1.5  2016/03/14 22:06:49  dennisg
 * Moved the _LOG/_log functions into common code outside blocks.
 *
 * Revision 1.4  2016/03/05 20:39:48  dennisg
 * Changed the template for BufferVOLK to be a templated type rather
 * than a fixed type. Of course this caused ripples throughout the
 * code. I also Redefined BufferCPLX as BufferVOLK with the type
 * lv_32fc_t.
 *
 * Revision 1.3  2016/03/05 05:15:50  dennisg
 * Added printing for a complex array.
 *
 * Revision 1.2  2016/02/19 22:25:05  dennisg
 * Added prefix to file names to reduce redundancy in other files.
 *
 * Revision 1.1  2016/02/09 16:27:52  dennisg
 * Initial revision
 *
 */

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <acars/print.h>


// All files are written here.
#define PREFIX "/tmp/"


static const std::string my_ident = "$Id: print.cc,v 1.1 2016/07/04 19:15:32 dennisg Exp $";


void
w_array( const boost::format& fname, const gr::acars::BufferFFT& x ) {
  
  std::stringstream s;
  
  for( size_t i = 0; i < x.size(); ++i ) {
    
    s << x.get()[i][0];
    if( x.get()[i][1] >= 0 )
      s << "+";
    s << x.get()[i][1] << "i,";
    
  }
  
  std::string str( s.str());
  if( str.empty() == false )
    str.erase( str.end() - 1 );
  
  std::ofstream f( PREFIX + fname.str());
  f << str << std::endl;
  
}

void
w_array( const boost::format& fname, const std::vector<float>& x ) {
  
  std::stringstream s;
  
  for( size_t i = 0; i < x.size(); ++i )
    s << x[i] << ",";
  
  std::string str( s.str());
  if( str.empty() == false )
    str.erase( str.end() - 1 );
  
  std::ofstream f( PREFIX + fname.str());
  f << str << std::endl;
  
}

void
w_array( const boost::format& fname, const gr::acars::BufferVOLK<float>& x ) {
  
  std::stringstream s;
  
  for( size_t i = 0; i < x.size(); ++i )
    s << x[i] << ",";
  
  std::string str( s.str());
  if( str.empty() == false )
    str.erase( str.end() - 1 );
  
  std::ofstream f( PREFIX + fname.str());
  f << str << std::endl;
  
}

void
w_array( const boost::format& fname,
	 const gr::acars::BufferVOLK<lv_32fc_t>& x ) {

  std::stringstream s;

  for( size_t i = 0; i < x.size(); ++i ) {
    s << lv_creal( x[i]);
    if( lv_cimag( x[i]) >= 0 )
      s << "+";
    s << lv_cimag( x[i]) << "i,";
  }

  std::string str( s.str());
  if( str.empty() == false )
    str.erase( str.end() - 1 );

  std::ofstream f( PREFIX + fname.str());
  f << str << std::endl;
    
}
  
void
w_array( const boost::format& fname, const gr::acars::Buffer<float>& x ) {
  
  std::stringstream s;
  
  for( size_t i = 0; i < x.size(); ++i )
    s << x[i] << ",";
  
  std::string str( s.str());
  if( str.empty() == false )
    str.erase( str.end() - 1 );
  
  std::ofstream f( PREFIX + fname.str());
  f << str << std::endl;
  
}

void
w_array( const boost::format& fname,
	 const std::vector<std::complex<float>>& x ) {

  std::stringstream s;

  for( size_t i = 0; i < x.size(); ++i ) {
    s << x[i].real();
    if( x[i].imag() >= 0 )
      s << "+";
    s << x[i].imag() << "i,";
  }

  std::string str( s.str());
  if( str.empty() == false )
    str.erase( str.end() - 1 );

  std::ofstream f( PREFIX + fname.str());
  f << str << std::endl;
    
}

const std::string
_log( const std::string& n, const char* f, const boost::format& m ) {

  std::stringstream s;

  s << n << "::" << f << "(): " << m;

  return s.str();
}

