/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 * 
 * Simple, inlined math routines taken from my ADS-B project.
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
 * $Log: math.h,v $
 * Revision 1.1  2016/07/04 19:15:54  dennisg
 * Initial revision
 *
 * Revision 1.1  2016/02/08 18:24:28  dennisg
 * Initial revision
 *
 */

#ifndef __ACARS_MATH_H__
#define __ACARS_MATH_H__

#include <cmath>
#include <complex>


static constexpr float sqrt2 = std::sqrt(2.0);
static constexpr float input_impedance = 600.0;
static const     float Vrms_dbM = std::sqrt( 0.001 * input_impedance );
static const     float P0_R     = ( Vrms_dbM * Vrms_dbM );

static inline float
mag( const std::complex<float>& t ) {
  
  return std::sqrt( t.real()*t.real() + t.imag()*t.imag());
}

static inline float
sqr( float i ) noexcept {
  
  return i*i;
}

static inline float
rms2dbm( float rms ) noexcept {
  
  return 10.0 * std::log10( sqr(rms) / P0_R );
}

static inline float
peak2dbm( float peak ) noexcept {
  
  return rms2dbm( peak / sqrt2 );
}


#endif

