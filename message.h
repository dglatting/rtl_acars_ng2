/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 *
 * Various constants and types regarding an ACARS message.
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
 * $Log: message.h,v $
 * Revision 1.2  2016/07/06 23:51:24  dennisg
 * Working. Check-in before loading on github.
 *
 * Revision 1.1  2016/07/04 19:15:54  dennisg
 * Initial revision
 *
 * Revision 1.3  2016/03/06 08:30:18  dennisg
 * Receiver code compiles and runs. Doesn't work yet.
 *
 * Revision 1.2  2016/02/24 06:04:56  dennisg
 * Saving changes.
 *
 * Revision 1.1  2016/02/22 22:45:17  dennisg
 * Initial revision
 *
 */

#ifndef INCLUDED_ACARS_MESSAGE_H
#define INCLUDED_ACARS_MESSAGE_H


extern "C" {

#include <stdint.h>

}


namespace gr {
  namespace acars {

    // The two MSK frequency components, which I arbitrarily called a
    // "tone" because that is how it is defined in ARINC 618-7. There
    // really isn't a 1200Hz tone, rather a half 1200Hz cycle indicates
    // the logical change in bit value from the previous bit.
    
#define TONE_2400 2400.0
#define TONE_1200 1200.0

    // This is the ACARS bit rate in bits-per-second followed by the
    // duration (in time) of one bit.
    
#define BIT_RATE 2400.0
#define deltaT   ( 1.0 / BIT_RATE )
    
    // The size of ACARS message components in bits and bytes. The
    // definition is in bits however bit fields are always a multiple of
    // eight (bytes). Characters are encoded 7-bit with odd parity
    // (Section 4.4.2.1 of 618-7). The exception is the pre-key and Block
    // Check Sequence (BCE).
    //
    // This information was obtained from:
    //
    // * http://www.scancat.com/Code-30_html_Source/acars.html
    // * ARINC 618-7
    //
    // Scancat defines the maximum Poe-key length 128bits but 618-7
    // defines the maximum pre-key length as 85mS with 35mS settling time
    // (AGC, etc) resulting in 53-75mS key length, or 127 bits
    // (.053mS). Further, Appendix B of 618-7 defines the pre-key as 16
    // bytes, or 128 bits. Therefore, to accommodate worst, worst case
    // when searching through recorded data I use 85mS as the pre-key
    // length.

#define PREKEY_BITS           208
#define BIT_SYNC_BITS          16
#define CHAR_SYNC_BITS         16
#define SOH_BITS                8
#define MODE_BITS               8
#define ADDRESS_BITS           56
#define ACK_NAK_BITS            8
#define LABEL_BITS             16
#define BLOCK_ID_BITS           8
#define STX_BITS                8
#define SEQ_NUM_BITS           32
#define FLIGHT_NUM_BITS        48
#define MAX_TEXT_BITS ( 220 * 8 )
#define ETX_BITS                8
#define CRC_BITS               16
#define BCS_BITS                8
    
#define PREKEY_BYTES          ( PREKEY_BITS / 8 )
#define BIT_SYNC_BYTES      ( BIT_SYNC_BITS / 8 )
#define CHAR_SYNC_BYTES    ( CHAR_SYNC_BITS / 8 )
#define SOH_BYTES                ( SOH_BITS / 8 )
#define MODE_BYTES              ( MODE_BITS / 8 )
#define ADDRESS_BYTES        ( ADDRESS_BITS / 8 )
#define ACK_NAK_BYTES        ( ACK_NAK_BITS / 8 )
#define LABEL_BYTES            ( LABEL_BITS / 8 )
#define BLOCK_ID_BYTES       (BLOCK_ID_BITS / 8 )
#define STX_BYTES                ( STX_BITS / 8 )
#define SEQ_NUM_BYTES        ( SEQ_NUM_BITS / 8 )
#define FLIGHT_NUM_BYTES  ( FLIGHT_NUM_BITS / 8 )
#define MAX_TEXT_BYTES      ( MAX_TEXT_BITS / 8 )
#define ETX_BYTES                ( ETX_BITS / 8 )
#define CRC_BYTES                ( CRC_BITS / 8 )
#define BCS_BYTES                ( BCS_BITS / 8 )

// The maximum size of the message with and without the preamble.

#define MAX_MBLOCK_BITS    ( BIT_SYNC_BITS + CHAR_SYNC_BITS + SOH_BITS + \
			     MODE_BITS + ADDRESS_BITS + ACK_NAK_BITS +\
			     LABEL_BITS + BLOCK_ID_BITS + STX_BITS +  \
			     SEQ_NUM_BITS + FLIGHT_NUM_BITS +		\
			     MAX_TEXT_BITS + ETX_BITS + CRC_BITS + BCS_BITS )
#define MIN_MBLOCK_BITS    ( MAX_MBLOCK_BITS - MAX_TEXT_BITS )
#define MAX_MBLOCK_BYTES   ( BIT_SYNC_BYTES + CHAR_SYNC_BYTES +SOH_BYTES + \
			     MODE_BYTES + ADDRESS_BYTES + ACK_NAK_BYTES + \
			     LABEL_BYTES + BLOCK_ID_BYTES + STX_BYTES + \
			     SEQ_NUM_BYTES + FLIGHT_NUM_BYTES +		\
			     MAX_TEXT_BYTES + ETX_BYTES + CRC_BYTES +	\
			     BCS_BYTES )
#define MIN_MBLOCK_BYTES   ( MAX_MBLOCK_BYTES - MAX_TEXT_BYTES )
    
#define MAX_wPREKEY_BITS    ( PREKEY_BITS + MAX_MBLOCK_BITS )
#define MAX_wPREKEY_BYTES   ( MAX_wPREKEY_BITS / 8 )

#define MIN_wPREKEY_BITS    ( PREKEY_BITS + MIN_MBLOCK_BITS )
#define MIN_wPREKEY_BYTES   ( MIN_wPREKEY_BITS / 8 )

    // ACARS message constants (from ARINC 618)

#define SOH uint8_t(0x01)
#define STX uint8_t(0x02)
#define ETX uint8_t(0x03)
#define ACK uint8_t(0x06)
#define  LF uint8_t(0x0a)
#define  CR uint8_t(0x0d)
#define NAK uint8_t(0x15)
#define SYN uint8_t(0x16)
#define ETB uint8_t(0x17)
#define DEL uint8_t(0x7f)

#define PRE_KEY_CHAR uint8_t(0xff)
#define BIT_SYNC_1   uint8_t('+')
#define BIT_SYNC_2   uint8_t('*')
#define CHAR_SYNC_1  uint8_t(SYN)
#define CHAR_SYNC_2  uint8_t(SYN)
    
  } // namespace acars
} // namespace gr

#endif /* INCLUDED_ACARS_ACARS_IMPL_H */


//  LocalWords:  DFT VOLK FFTs FFTW ACARS ARINC
