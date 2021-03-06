/* -*- c++ -*- */

/* 
 * Copyright 2016 Dennis Glatting
 *
 * This file holds code implementing the CRC algorithm used in ACARS.
 *
 *
 * References:
 *
 * RFC-2119, Key words for use in RFCs to Indicate Requirement Levels
 *
 * This polynomial is identified as: CRC-16-CCITT
 *  https://en.wikipedia.org/wiki/Polynomial_representations_of_cyclic_redundancy_checks
 *
 * Koopman DOES NOT identify this polynomial as having error
 * corrective properties. That's sad.
 *  https://users.ece.cmu.edu/~koopman/crc/
 *
 * Other reference material:
 *  http://srecord.sourceforge.net/crc16-ccitt.html
 *  https://github.com/zqad/crc16-ccitt/blob/master/crc16-ccitt-generate-tab
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
 * $Log: crc.cc,v $
 * Revision 1.1  2016/07/06 05:26:21  dennisg
 * Initial revision
 *
 * Revision 1.6  2016/03/14 23:04:45  dennisg
 * Added a RCS tag.
 *
 * Revision 1.5  2016/03/02 04:53:20  dennisg
 * transmission of CRC now works against rtl_acars_ng.
 *
 * Revision 1.4  2016/02/29 06:42:26  dennisg
 * Changed CRC code to be table driven, now that I have confidence in
 * that. Added alternative polynomials based on various other texts
 * and code. Added optional padding to generation and checking but it
 * isn't clear if that is necessary or another implementation is
 * buggy; which it is.
 *
 * Revision 1.3  2016/02/29 04:01:30  dennisg
 * Fixed several bugs related to CRC calculation and Tx buffer
 * content.
 *
 * Revision 1.2  2016/02/19 05:44:18  dennisg
 * Added/fixed text.
 *
 * Revision 1.1  2016/02/19 05:41:09  dennisg
 * Initial revision
 *
 */

#undef CRC_DEBUG

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

extern "C" {

#include <stddef.h>
#include <stdint.h>
  
}

#include <acars/crc.h>
#include <acars/utility.h>


static const std::string my_ident = "$Id: crc.cc,v 1.1 2016/07/06 05:26:21 dennisg Exp $";


static const
std::vector<uint16_t> crc_ccitt = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

// This CRC table is derived from the application "acarsdec," of which
// there are many versions. Where the polynomial comes from is unknown
// to me and I found no other reference to the polynomial. Also, the
// table used in that implementation is wrong and therefore this table
// won't work against that implementation. Sucks.
//
// The polynomial seems to be:
//
//  P(x) = x^16 + x^12 + x^8 + x^7 + x^3 + 1
//
// If you know where this polynomial comes from, drop me a line.

static const
std::vector<uint16_t> crc_ccitt_alt1 = {
  0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
  0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
  0x0919, 0x1890, 0x2A0B, 0x3B82, 0x4F3D, 0x5EB4, 0x6C2F, 0x7DA6,
  0x8551, 0x94D8, 0xA643, 0xB7CA, 0xC375, 0xD2FC, 0xE067, 0xF1EE,
  0x1232, 0x03BB, 0x3120, 0x20A9, 0x5416, 0x459F, 0x7704, 0x668D,
  0x9E7A, 0x8FF3, 0xBD68, 0xACE1, 0xD85E, 0xC9D7, 0xFB4C, 0xEAC5,
  0x1B2B, 0x0AA2, 0x3839, 0x29B0, 0x5D0F, 0x4C86, 0x7E1D, 0x6F94,
  0x9763, 0x86EA, 0xB471, 0xA5F8, 0xD147, 0xC0CE, 0xF255, 0xE3DC,
  0x2464, 0x35ED, 0x0776, 0x16FF, 0x6240, 0x73C9, 0x4152, 0x50DB,
  0xA82C, 0xB9A5, 0x8B3E, 0x9AB7, 0xEE08, 0xFF81, 0xCD1A, 0xDC93,
  0x2D7D, 0x3CF4, 0x0E6F, 0x1FE6, 0x6B59, 0x7AD0, 0x484B, 0x59C2,
  0xA135, 0xB0BC, 0x8227, 0x93AE, 0xE711, 0xF698, 0xC403, 0xD58A,
  0x3656, 0x27DF, 0x1544, 0x04CD, 0x7072, 0x61FB, 0x5360, 0x42E9,
  0xBA1E, 0xAB97, 0x990C, 0x8885, 0xFC3A, 0xEDB3, 0xDF28, 0xCEA1,
  0x3F4F, 0x2EC6, 0x1C5D, 0x0DD4, 0x796B, 0x68E2, 0x5A79, 0x4BF0,
  0xB307, 0xA28E, 0x9015, 0x819C, 0xF523, 0xE4AA, 0xD631, 0xC7B8,
  0x48C8, 0x5941, 0x6BDA, 0x7A53, 0x0EEC, 0x1F65, 0x2DFE, 0x3C77,
  0xC480, 0xD509, 0xE792, 0xF61B, 0x82A4, 0x932D, 0xA1B6, 0xB03F,
  0x41D1, 0x5058, 0x62C3, 0x734A, 0x07F5, 0x167C, 0x24E7, 0x356E,
  0xCD99, 0xDC10, 0xEE8B, 0xFF02, 0x8BBD, 0x9A34, 0xA8AF, 0xB926,
  0x5AFA, 0x4B73, 0x79E8, 0x6861, 0x1CDE, 0x0D57, 0x3FCC, 0x2E45,
  0xD6B2, 0xC73B, 0xF5A0, 0xE429, 0x9096, 0x811F, 0xB384, 0xA20D,
  0x53E3, 0x426A, 0x70F1, 0x6178, 0x15C7, 0x044E, 0x36D5, 0x275C,
  0xDFAB, 0xCE22, 0xFCB9, 0xED30, 0x998F, 0x8806, 0xBA9D, 0xAB14,
  0x6CAC, 0x7D25, 0x4FBE, 0x5E37, 0x2A88, 0x3B01, 0x099A, 0x1813,
  0xE0E4, 0xF16D, 0xC3F6, 0xD27F, 0xA6C0, 0xB749, 0x85D2, 0x945B,
  0x65B5, 0x743C, 0x46A7, 0x572E, 0x2391, 0x3218, 0x0083, 0x110A,
  0xE9FD, 0xF874, 0xCAEF, 0xDB66, 0xAFD9, 0xBE50, 0x8CCB, 0x9D42,
  0x7E9E, 0x6F17, 0x5D8C, 0x4C05, 0x38BA, 0x2933, 0x1BA8, 0x0A21,
  0xF2D6, 0xE35F, 0xD1C4, 0xC04D, 0xB4F2, 0xA57B, 0x97E0, 0x8669,
  0x7787, 0x660E, 0x5495, 0x451C, 0x31A3, 0x202A, 0x12B1, 0x0338,
  0xFBCF, 0xEA46, 0xD8DD, 0xC954, 0xBDEB, 0xAC62, 0x9EF9, 0x8F70
};

// This weird polynomial is found in the document:
//
// AIR GROUND DATA LINK VHF AIRLINE COMMUNICATIONS AND REPORTING
// SYSTEM (ACARS) PRELIMINARY TEST REPORT.
//  Albert Rehmann, 1995
//
//  P(x) = x^16 + x^15 + x^13 + x^5 + x^3 + x^1 + 1 (aka: 0xA02B)
//
// I'm not clear on the purpose of that polynomial but I found other
// texts that describe it as "ARINC." I haven't (yet) found an
// implementation.

static const
std::vector<uint16_t> crc_ccitt_alt2 = {
  0x0000, 0xA02B, 0xE07D, 0x4056, 0x60D1, 0xC0FA, 0x80AC, 0x2087,
  0xC1A2, 0x6189, 0x21DF, 0x81F4, 0xA173, 0x0158, 0x410E, 0xE125,
  0x236F, 0x8344, 0xC312, 0x6339, 0x43BE, 0xE395, 0xA3C3, 0x03E8,
  0xE2CD, 0x42E6, 0x02B0, 0xA29B, 0x821C, 0x2237, 0x6261, 0xC24A,
  0x46DE, 0xE6F5, 0xA6A3, 0x0688, 0x260F, 0x8624, 0xC672, 0x6659,
  0x877C, 0x2757, 0x6701, 0xC72A, 0xE7AD, 0x4786, 0x07D0, 0xA7FB,
  0x65B1, 0xC59A, 0x85CC, 0x25E7, 0x0560, 0xA54B, 0xE51D, 0x4536,
  0xA413, 0x0438, 0x446E, 0xE445, 0xC4C2, 0x64E9, 0x24BF, 0x8494,
  0x8DBC, 0x2D97, 0x6DC1, 0xCDEA, 0xED6D, 0x4D46, 0x0D10, 0xAD3B,
  0x4C1E, 0xEC35, 0xAC63, 0x0C48, 0x2CCF, 0x8CE4, 0xCCB2, 0x6C99,
  0xAED3, 0x0EF8, 0x4EAE, 0xEE85, 0xCE02, 0x6E29, 0x2E7F, 0x8E54,
  0x6F71, 0xCF5A, 0x8F0C, 0x2F27, 0x0FA0, 0xAF8B, 0xEFDD, 0x4FF6,
  0xCB62, 0x6B49, 0x2B1F, 0x8B34, 0xABB3, 0x0B98, 0x4BCE, 0xEBE5,
  0x0AC0, 0xAAEB, 0xEABD, 0x4A96, 0x6A11, 0xCA3A, 0x8A6C, 0x2A47,
  0xE80D, 0x4826, 0x0870, 0xA85B, 0x88DC, 0x28F7, 0x68A1, 0xC88A,
  0x29AF, 0x8984, 0xC9D2, 0x69F9, 0x497E, 0xE955, 0xA903, 0x0928,
  0xBB53, 0x1B78, 0x5B2E, 0xFB05, 0xDB82, 0x7BA9, 0x3BFF, 0x9BD4,
  0x7AF1, 0xDADA, 0x9A8C, 0x3AA7, 0x1A20, 0xBA0B, 0xFA5D, 0x5A76,
  0x983C, 0x3817, 0x7841, 0xD86A, 0xF8ED, 0x58C6, 0x1890, 0xB8BB,
  0x599E, 0xF9B5, 0xB9E3, 0x19C8, 0x394F, 0x9964, 0xD932, 0x7919,
  0xFD8D, 0x5DA6, 0x1DF0, 0xBDDB, 0x9D5C, 0x3D77, 0x7D21, 0xDD0A,
  0x3C2F, 0x9C04, 0xDC52, 0x7C79, 0x5CFE, 0xFCD5, 0xBC83, 0x1CA8,
  0xDEE2, 0x7EC9, 0x3E9F, 0x9EB4, 0xBE33, 0x1E18, 0x5E4E, 0xFE65,
  0x1F40, 0xBF6B, 0xFF3D, 0x5F16, 0x7F91, 0xDFBA, 0x9FEC, 0x3FC7,
  0x36EF, 0x96C4, 0xD692, 0x76B9, 0x563E, 0xF615, 0xB643, 0x1668,
  0xF74D, 0x5766, 0x1730, 0xB71B, 0x979C, 0x37B7, 0x77E1, 0xD7CA,
  0x1580, 0xB5AB, 0xF5FD, 0x55D6, 0x7551, 0xD57A, 0x952C, 0x3507,
  0xD422, 0x7409, 0x345F, 0x9474, 0xB4F3, 0x14D8, 0x548E, 0xF4A5,
  0x7031, 0xD01A, 0x904C, 0x3067, 0x10E0, 0xB0CB, 0xF09D, 0x50B6,
  0xB193, 0x11B8, 0x51EE, 0xF1C5, 0xD142, 0x7169, 0x313F, 0x9114,
  0x535E, 0xF375, 0xB323, 0x1308, 0x338F, 0x93A4, 0xD3F2, 0x73D9,
  0x92FC, 0x32D7, 0x7281, 0xD2AA, 0xF22D, 0x5206, 0x1250, 0xB27B
};

// Compute the CRC over the contents of the buffer. The ACARS CRC
// polynomial is defined as:
//
//  P(x) = x^16 + x^12 + x^5 + 1 (aka: 0x1021)
//
// There are multiple implementations of CRC16-CCITT, many wrong.


inline uint16_t
fold_crc_rev( uint16_t crc, uint8_t c ) {

  // Reflect input byte.
  const uint8_t the_byte = reverse_bits[ c ];
  const uint8_t pos = ((( crc >> 8 ) ^ the_byte ) & 0xff );
  
  return ( crc << 8 ) ^ crc_ccitt[ pos ];

}

inline uint16_t
fold_crc( uint16_t crc, uint8_t c ) {

  const uint8_t the_byte = c;
  const uint8_t pos = ((( crc >> 8 ) ^ the_byte ) & 0xff );

  return ( crc << 8 ) ^ crc_ccitt[ pos ];

}

uint16_t
gen_crc( std::vector<uint8_t>::const_iterator s,
	 std::vector<uint8_t>::const_iterator e ) {

  uint16_t wCRC = 0;

  if( s >= e )
    wCRC = 0;
  else {
    
    do {

#ifdef CRC_DEBUG
      std::cout << "0x"
                << std::hex << std::setfill('0') << std::setw(2)
                << unsigned(*s) << std::dec << ",";
#endif
      
      wCRC = fold_crc_rev( wCRC, *s );
	
    } while( ++s < e );

#if 0
    // Add 16bits of pad.
    //
    // From ARINC 618-7, Section 2.2.10:
    //
    // The initial multiplication of the message polynomial by x^16
    // effectively adds sixteen zero bits to the message. These bits
    // drive the remainder out of the shift register often used in a
    // hardware implementation of the algorithm.
          
    wCRC = fold_crc_rev( wCRC, 0 );
    wCRC = fold_crc_rev( wCRC, 0 );
#endif
    
    // What ARINC does not say is the CRC output MUST be reflected.

#ifdef CRC_DEBUG
    std::cout << "CRC=0x" << std::hex << wCRC;
#endif
    
    wCRC =
      ( reverse_bits[( wCRC >> 0 ) & 0x00ff ] << 8 ) |
      ( reverse_bits[( wCRC >> 8 ) & 0x00ff ] << 0 );

#ifdef CRC_DEBUG
    std::cout << "=>" << std::hex << wCRC << std::dec << std::endl;
#endif
    
  }
  
  return wCRC;
}


bool
check_crc( std::vector<uint8_t>::const_iterator s,
	   std::vector<uint8_t>::const_iterator e ) {

  uint16_t wCRC = 0;

  if( s >= e )
    wCRC = 0;
  else {

    do {

#ifdef CRC_DEBUG
      std::cout << "0x"
		<< std::hex << std::setfill('0') << std::setw(2)
		<< unsigned(*s) << std::dec << ",";
#endif
      
      wCRC = fold_crc( wCRC, *s );

    } while( ++s < e );

#ifdef CRC_DEBUG
        std::cout << "=0x" << std::hex << wCRC << std::dec << std::endl; 
#endif
  }

  return wCRC == 0;
}



//  LocalWords:  CRC ACARS ARINC
