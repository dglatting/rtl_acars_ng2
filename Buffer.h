/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 * 
 *
 * Templates to provide buffers for VOLK and FFT libraries.
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
 * $Log: Buffer.h,v $
 * Revision 1.1  2016/07/04 19:15:54  dennisg
 * Initial revision
 *
 * Revision 1.5  2016/03/06 08:30:18  dennisg
 * Receiver code compiles and runs. Doesn't work yet.
 *
 * Revision 1.4  2016/03/05 20:39:48  dennisg
 * Changed the template for BufferVOLK to be a templated type rather
 * than a fixed type. Of course this caused ripples throughout the
 * code. I also Redefined BufferCPLX as BufferVOLK with the type
 * lv_32fc_t.
 *
 * Revision 1.3  2016/02/24 06:04:56  dennisg
 * Saving changes.
 *
 * Revision 1.2  2016/02/23 19:25:59  dennisg
 * Now compiles.
 *
 * Revision 1.1  2016/02/23 19:09:55  dennisg
 * Initial revision
 *
 */

#ifndef __ACARS_BUFFER_H__
#define __ACARS_BUFFER_H__

#include <cassert>

extern "C" {

#include <stdlib.h>
#include <string.h>
  
#include <fftw3.h>

}

#include <volk/volk.h>


namespace gr {
  namespace acars {

    // The purpose of this buffer template is:
    //  1) To put some buffer overflow checking into the application's
    //     use of buffers;
    //  2) Provide assured allocation/deallocation;
    //  3) Better buffer size management; and,
    //  4) Eventual array bounds check.
    //
    // Generally a buffer is used as an array.

    template<typename T>
    class Buffer {

    private:

      // The size (in number of items, NOT bytes) of my buffer plus
      // the buffer itself.

      size_t  my_size;
      T*      my_buffer;

      // Pointers to allocate and free functions.

      void *(*mem_alloc  )( size_t );
      void  (*mem_dealloc)( void*  );

      // Private functions that assign memory and insert the check
      // sequence and another function to check the sequence. The
      // point is to look for buffer overruns.

      void _deadbeef_alloc(   void ) noexcept;
      void _deadbeef_dealloc( void ) noexcept;
      void _deadbeef_check(   void ) const noexcept;

    public:

               Buffer( void );
      virtual ~Buffer( void );

      Buffer( size_t the_size,
	      void   *(*the_alloc  )(size_t) = ::malloc,
	      void    (*the_dealloc)(void* ) = ::free );

      Buffer( const Buffer&  b );
      Buffer(       Buffer&& b );

      Buffer& operator=( const Buffer&  b );
      Buffer& operator=(       Buffer&& b );

      // Set a new buffer size and allocation/deallocation
      // pointers. If the pointers are not set then use existing.

      void set( size_t the_size,
		void   *(*the_alloc  )(size_t) = nullptr,
		void    (*the_dealloc)(void* ) = nullptr );

      // Return things about the buffer.

      size_t size( void ) const noexcept;
      void*  (*alloc(    void ))( size_t ) const noexcept;
      void   (*dealloc(  void ))( void*  ) const noexcept;

      // Return the buffer pointer.

      T*     get( void  ) const noexcept;

      // Return a reference to the value at X.

      const T& operator[]( int x ) const noexcept;
      T& operator[]( int x ) noexcept;

      // Check the buffer for overflow.

      void check( void ) const noexcept;

    };

    template<typename T>
    Buffer<T>::Buffer( void )
      : my_size( 0 ), my_buffer( nullptr ),
	mem_alloc( ::malloc ), mem_dealloc( ::free ) {

    }

    template<typename T>
    Buffer<T>::~Buffer( void ) {

      _deadbeef_dealloc();

    }

    template<typename T>
    Buffer<T>::Buffer( size_t the_sz,
		       void   *(*the_alloc  )(size_t),
		       void    (*the_dealloc)(void* ))
      : my_size( the_sz ), my_buffer( nullptr ),
	mem_alloc( the_alloc ), mem_dealloc( the_dealloc ) {

      _deadbeef_alloc();

    }

    template<typename T>
    Buffer<T>::Buffer( const Buffer& b )
      : my_size( b.my_size ), my_buffer( nullptr ),
	mem_alloc( b.mem_alloc ), mem_dealloc( b.mem_dealloc ) {

      _deadbeef_alloc();

      ::memcpy( my_buffer, b.get(), my_size * sizeof( T ));

    }

    template<typename T>
    Buffer<T>::Buffer( Buffer&& b )
      : my_size( b.my_size ), my_buffer( b.my_buffer ),
	mem_alloc( b.mem_alloc ), mem_dealloc( b.mem_dealloc ) {

      b.my_size   = 0;
      b.my_buffer = nullptr;

    }

    template<typename T>
    Buffer<T>&
    Buffer<T>::operator=( const Buffer<T>& b ) {

      _deadbeef_dealloc();

      my_size     = b.my_size;
      mem_alloc   = b.mem_alloc;
      mem_dealloc = b.mem_dealloc;

      _deadbeef_alloc();

      ::memcpy( my_buffer, b.get(), my_size * sizeof( T ));

      return *this;
    }

    template<typename T>
    Buffer<T>&
    Buffer<T>::operator=( Buffer&& b ) {

      _deadbeef_dealloc();

      my_size     = b.my_size;
      my_buffer   = b.my_buffer;
      mem_alloc   = b.mem_alloc;
      mem_dealloc = b.mem_dealloc;

      b.my_size   = 0;
      b.my_buffer = nullptr;

      return *this;
    }

    template<typename T>
    void
    Buffer<T>::_deadbeef_alloc( void ) noexcept {

      const size_t sz = ( sizeof( T ) * my_size );

      if( my_buffer )
	_deadbeef_dealloc();

      assert( mem_alloc );

      if( my_size ) {

	my_buffer = (T*)(*mem_alloc)( sz + sizeof( uint64_t ) + 2 );
	assert( my_buffer );

	uint8_t* p = (uint8_t*)my_buffer;

	p[ sz + 0 ] = 'd';
	p[ sz + 1 ] = 'e';
	p[ sz + 2 ] = 'a';
	p[ sz + 3 ] = 'd';
	p[ sz + 4 ] = 'b';
	p[ sz + 5 ] = 'e';
	p[ sz + 6 ] = 'e';
	p[ sz + 7 ] = 'f';

      }
    }

    template<typename T>
    void
    Buffer<T>::_deadbeef_dealloc( void ) noexcept {

      _deadbeef_check();

      if( my_buffer ) {

	assert( my_size  );
	assert( mem_dealloc );

	_deadbeef_check();

	(*mem_dealloc)( my_buffer ), my_buffer = nullptr;

      }
    }

    template<typename T>
    inline void
    Buffer<T>::_deadbeef_check( void ) const noexcept {

      if( my_buffer ) {

	const size_t   sz = ( sizeof( T ) * my_size );
	const uint8_t* p  = (uint8_t*)my_buffer;

	assert(( p[ sz + 0 ] == 'd' ) &&
	       ( p[ sz + 1 ] == 'e' ) &&
	       ( p[ sz + 2 ] == 'a' ) &&
	       ( p[ sz + 3 ] == 'd' ) &&
	       ( p[ sz + 4 ] == 'b' ) &&
	       ( p[ sz + 5 ] == 'e' ) &&
	       ( p[ sz + 6 ] == 'e' ) &&
	       ( p[ sz + 7 ] == 'f' ));

      }
    }

    template<typename T>
    inline size_t
    Buffer<T>::size( void ) const noexcept {

      return my_size;
    }

    template<typename T>
    inline void* ( *Buffer<T>::alloc( void ))( size_t ) const noexcept {

      return mem_alloc;
    }

    template<typename T>
    inline void ( *Buffer<T>::dealloc( void ))( void* ) const noexcept {

      return mem_dealloc;
    }

    template<typename T>
    inline T*
    Buffer<T>::get( void  ) const noexcept {

      return my_buffer;
    }

    template<typename T>
    inline const T&
    Buffer<T>::operator[]( int x ) const noexcept {

      assert((size_t)x <= size());

      return this->get()[x];
    }

    template<typename T>
    inline T&
    Buffer<T>::operator[]( int x ) noexcept {

      assert((size_t)x <= size());

      return this->get()[x];
    }

    template<typename T>
    void
    Buffer<T>::set( size_t the_size,
		    void   *(*the_alloc  )(size_t),
		    void    (*the_dealloc)(void* )) {

      _deadbeef_dealloc();

      my_size = the_size;

      if( the_alloc )
	mem_alloc = the_alloc;
      if( the_dealloc )
	mem_dealloc = the_dealloc;

      _deadbeef_alloc();

    }

    template<typename T>
    inline void
    Buffer<T>::check( void ) const noexcept {

      if( my_size )
	assert( my_buffer );

      _deadbeef_check();

    }




    // Specialization of the Buffer class for FFTs against the FFTW
    // library.

    class BufferFFT : public Buffer<fftw_complex> {

    public:

               BufferFFT( void );
      virtual ~BufferFFT( void ) {};

      BufferFFT( size_t the_size );

      BufferFFT( const BufferFFT&  b );
      BufferFFT(       BufferFFT&& b );

      BufferFFT& operator=( const BufferFFT&  b );
      BufferFFT& operator=(       BufferFFT&& b );

      void set( size_t the_size );

    };

    inline BufferFFT&
    BufferFFT::operator=( const BufferFFT&  b ) {
      
      Buffer<fftw_complex>::operator=( b );
      
      return *this;
    }
    
    inline BufferFFT&
    BufferFFT::operator=( BufferFFT&&  b ) {
      
      Buffer<fftw_complex>::operator=( b );
      
      return *this;
    }
    
    inline void
    BufferFFT::set( size_t the_size ) {
      
      Buffer<fftw_complex>::set( the_size, fftw_malloc, fftw_free );
      
    }


    void* volk_malloc_wrapper( size_t );
    
    // Specialization of the Buffer class against the VOLK library.

    template<typename T>
    class BufferVOLK : public Buffer<T> {

    public:

               BufferVOLK( void );
      virtual ~BufferVOLK( void ) {};

      BufferVOLK( size_t the_size );
	
      BufferVOLK( const BufferVOLK&  b );
      BufferVOLK(       BufferVOLK&& b );

      BufferVOLK& operator=( const BufferVOLK&  b );
      BufferVOLK& operator=(       BufferVOLK&& b );

      void set( size_t the_size );

    };

    template <class T>
    BufferVOLK<T>::BufferVOLK( size_t the_size )
      : Buffer<T>( the_size, volk_malloc_wrapper, ::volk_free ) {

    }

    template <class T>
    BufferVOLK<T>::BufferVOLK( const BufferVOLK&  b )
      : Buffer<T>( b ) {

    }

    template <class T>
    BufferVOLK<T>::BufferVOLK( BufferVOLK&& b )
      : Buffer<T>( b ) {

    }
    
    template<typename T>
    inline void
    BufferVOLK<T>::set( size_t the_size ) {

      Buffer<T>::set( the_size, volk_malloc_wrapper, volk_free );

    }

    template<typename T>
    inline BufferVOLK<T>&
    BufferVOLK<T>::operator=( const BufferVOLK<T>&  b ) {

      Buffer<T>::operator=( b );

      return *this;
    }

    template<typename T>
    inline BufferVOLK<T>&
    BufferVOLK<T>::operator=( BufferVOLK<T>&&  b ) {
      
      Buffer<T>::operator=( b );
      
      return *this;
    }
    
  }
}


#endif


//  LocalWords:  BufferVOLK templated BufferCPLX
