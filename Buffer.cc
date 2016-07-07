/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 *
 * Implementation of parts of the Buffer classes that are too nasty to
 * inline.
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
 * $Log: Buffer.cc,v $
 * Revision 1.2  2016/07/06 19:31:31  dennisg
 * Check in before porting/updating getmesg() back from GNURadio.
 *
 * Revision 1.1  2016/07/04 19:15:32  dennisg
 * Initial revision
 *
 * Revision 1.4  2016/03/14 23:04:45  dennisg
 * Added a RCS tag.
 *
 * Revision 1.3  2016/03/06 08:30:18  dennisg
 * Receiver code compiles and runs. Doesn't work yet.
 *
 * Revision 1.2  2016/03/05 20:39:48  dennisg
 * Changed the template for BufferVOLK to be a templated type rather
 * than a fixed type. Of course this caused ripples throughout the
 * code. I also Redefined BufferCPLX as BufferVOLK with the type
 * lv_32fc_t.
 *
 * Revision 1.1  2016/02/23 19:26:12  dennisg
 * Initial revision
 *
 */

#include <string>

#include <acars/Buffer.h>


static const std::string my_ident = "$Id: Buffer.cc,v 1.2 2016/07/06 19:31:31 dennisg Exp $";


namespace gr {
  namespace acars {


    BufferFFT::BufferFFT( void )
      : Buffer<fftw_complex>( 0, fftw_malloc, fftw_free ) {
      
    }
    
    BufferFFT::BufferFFT( size_t the_size )
      : Buffer<fftw_complex>( the_size, fftw_malloc, fftw_free ) {
      
    }
    
    BufferFFT::BufferFFT( const BufferFFT& b )
      : Buffer<fftw_complex>( b ) {
      
    }
    
    BufferFFT::BufferFFT( BufferFFT&& b )
      : Buffer<fftw_complex>( b ) {
      
    }
    


    // My Buffer class memory allocator only handles one parameter and
    // VOLK's malloc() requires two: a boundary alignment
    // function. Fortunately, I always use the default VOLK alignment
    // function.

    void*
    volk_malloc_wrapper( size_t the_size ) {
      
      return volk_malloc( the_size, volk_get_alignment());
    }

    template <class T>
    BufferVOLK<T>::BufferVOLK( void )
      : Buffer<T>( 0, volk_malloc_wrapper, volk_free ) {

    }

    template class BufferVOLK<float>;
    template class BufferVOLK<lv_32fc_t>;

  }
}


//  LocalWords:  BufferVOLK templated BufferCPLX
