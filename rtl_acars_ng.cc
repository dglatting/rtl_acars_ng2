/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 * Copyright (C) 2012 by Kyle Keen <keenerd@gmail.com>
 * Copyright (C) 2013 by Elias Oenal <EliasOenal@gmail.com>
 * Copyright (C) 2016 by Dennis Glatting <dg@pki2.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $Log: rtl_acars_ng.cc,v $
 * Revision 1.7  2016/07/06 23:51:24  dennisg
 * Working. Check-in before loading on github.
 *
 * Revision 1.6  2016/07/06 19:31:31  dennisg
 * Check in before porting/updating getmesg() from GNURadio.
 *
 * Revision 1.5  2016/07/06 17:37:59  dennisg
 * Many changes related to the incorporation of getmesg() changes and
 * CRC routines and changes related to writing the README.
 *
 * Revision 1.4  2016/07/06 06:44:40  dennisg
 * Ran the code through GCC 6.1.0 to check for further compiler
 * warnings and C++14 from C++11. No additional errors were emitted by
 * the compiler and the code ran without error.
 *
 * Revision 1.3  2016/07/06 06:40:48  dennisg
 * Replaced the CRC code with the CRC algorithm from my GNURadio
 * code. I also removed (temporarily?) the two bit error correction
 * logic because in failure cases it consumed a lot of CPU, as
 * expected. I may put the two bit back, which would be easy to do.
 *
 * Revision 1.2  2016/07/06 05:31:00  dennisg
 * About to insert CRC routine and related changes.
 *
 * Revision 1.1  2016/07/05 22:17:19  dennisg
 * Initial revision
 * 
 */


#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#include "getopt/getopt.h"
#define usleep(x) Sleep(x/1000)
#ifdef _MSC_VER
#define round(x) (x > 0.0 ? floor(x + 0.5): ceil(x - 0.5))
#endif
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <omp.h>
#include <pthread.h>
#include <libusb.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <acars/Buffer.h>
#include <acars/crc.h>
#include <acars/message.h>

using namespace gr::acars;


#include "rtl-sdr.h"


#define DEFAULT_SAMPLE_RATE	   24000
#define DEFAULT_ASYNC_BUF_NUMBER      32
#define DEFAULT_BUF_LENGTH   (1 * 16384)
#define MAXIMUM_OVERSAMPLE	      16
#define MAXIMUM_BUF_LENGTH	(MAXIMUM_OVERSAMPLE * DEFAULT_BUF_LENGTH)
#define AUTO_GAIN		    -100
#define BUFFER_DUMP		    4096

#define FREQUENCIES_LIMIT  	    1000


// After down sampling (i.e., applying a low pass filter and
// decimating), the sample rate into the bit former is 48k/bps.

#define Fe       48000.0
#define Freqh    (4800.0/Fe*twoPI)
#define Freql    (2400.0/Fe*twoPI)
#define BITLEN   int(Fe/1200.0)

/* ACARS defines */

#define VFOPLL 0.7e-3
#define BITPLL 0.2


// Bit state machine.

static struct bstat_s {

  BufferVOLK<float>
    hsample = BufferVOLK<float>( BITLEN ),
    lsample = BufferVOLK<float>( BITLEN ),
    isample = BufferVOLK<float>( BITLEN ),
    qsample = BufferVOLK<float>( BITLEN ),
    csample = BufferVOLK<float>( BITLEN );

  int is;
  int clock;
  float lin;
  float phih,phil;
  float dfh,dfl;
  float pC,ppC;
  int sgI, sgQ;
  float ea;

} bstat;


// Message state machine state.

enum class STATE {
  HEADL, HEADF,    // Header (PRE-KEY) lost, Header found. The
		   // transition from lost to found is at least eight
		   // bits are found to be a logical 1. When "found,"
		   // the state machine will consume PRE-KEY bits
		   // until at least two bits in eight are not a
		   // logical 1, at which time the state machine
		   // starts to look for BIT-SYNC. If there are more
		   // that two bits that are not a logical one then
		   // the state machine transitions back to lost.
  SYNC,            // Looking for the bit pattern matching the
		   // BIT-SYNC, CHAR-SYNC, and SOH.
  TXT,             // Collecting the text segment of a message by
		   // passing the SOH1 state and terminated by a ETX
		   // or ETB (or buffer overflow).
  CRC1, CRC2,      // Looking for the first and second CRC byte.
  END              // Process the message, if any.
};
std::ostream& operator<<( std::ostream&, const STATE& ) ;

struct {

  // The state machine's state.
  
  STATE state;
  
  // After PRE-KEY characters have been found then the search is on
  // for the SYNC words and SOH. We collect those bits and at a
  // certain point (i.e., when enough bits has been collected for the
  // words) the check is on to declare sync.
  //
  // There has to be a permissible number of errors in the words and
  // there has to be a stopping point to declare no-SYNC.

  uint64_t syncForming;
  int      syncBitsHave;

  const int errLim = 3;
  const int syncBitsLim = ( 40 + 15 ); // 5 words * 8 bits plus extra.

  // The number of consecutive PRE-KEY bits I have seen and the number
  // I want to see before advancing the state machine.
  //
  //  (10ms represents 24 bits of PRE-KEY. PRE_KEY is 208 bits, which
  //   is 86ms.)

        int consecutivePreKey;
  const int consecutivePreKeyLim = ( 0.010 * 2400 ); /* 10ms */
  
  // The raw message bytes with parity and framing bytes.

  std::vector<uint8_t> rawText;
  
  // This was previously used for CRC but doubled as a flag. Now it is
  // a flag where non-zero indicates uncorrected CRC errors.
  
  uint16_t crc;

} m_state;


struct acars_flight {
  char *flightid;
  char *from;
  char *to;
  char *airline;
};
struct acars_flight acars_flights[160000];


struct acars_aircraft {
  char *registration;
  char *modes;
  char *manufacturer;
  char *model;
};
struct acars_aircraft acars_aircrafts[500000];


struct acars_airport {
  char *name;
  char *city;
  char *country;
  char *code;
};
struct acars_airport acars_airports[10000];


struct acars_ml {
  char *ml_code;
  char *ml_label;
};

struct acars_ml acars_mls[16000];

/*
  struct acars_airlines {
  char *al_code;
  char *al_label;
  };

  struct acars_airlines acars_airliness[16000];
*/


// This vector is indexed with a u_char and returns the number of bits
// set to 1.

static const
std::vector<int> bits_set = {
  0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

// This vector and routine is used to set a character's parity.

static const
std::vector<uint8_t> parity_bit = {
  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00,
  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,
  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,
  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00,

  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,
  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00,
  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00,
  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,

  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,
  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00,
  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00,
  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,

  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00,
  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,
  0x00, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80,
  0x80, 0x00, 0x00, 0x80, 0x00, 0x80, 0x80, 0x00
};

inline uint8_t
_to_odd( uint8_t c ) {

  return c | parity_bit[ c & 0x7f ];
}

// Given a index, which bit is set/clear?

static const std::vector<uint8_t> mask = {
  0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
};

// I could do a bunch of casting or simply provide inline wrappers
// that do the same thing. I prefer this way.

inline int
strlen( const uint8_t* c ) {

  return ::strlen(( const char*)c );
}

inline int
strcmp( const uint8_t* s1, const char* s2 ) {

  return ::strcmp(( const char*)s1, s2 );
}

inline char*
strcpy( uint8_t* s1, const char* s2 ) {

  return ::strcpy(( char* )s1, s2 );
}

inline char*
strcpy( char* s1, const uint8_t* s2 ) {

  return ::strcpy( s1, ( const char* )s2 );
}

inline char*
strncpy( char* dest, const uint8_t* src, size_t n ) {

  return ::strncpy( dest, (const char*)src, n );
}

// The maximum number of threads I will use for OpenMP operations is
// 5/8 of the available cores/hyper-threads so NOT to over consume
// available resources and potentially starving other threads, such as
// the SDR reception thread. However, the minimum number of computed
// threads is two thereby potentially impacting single and dual core
// CPUs.

static int n_omp = 2;


static pthread_t demod_thread;
static pthread_cond_t data_ready;   /* shared buffer filled */
static pthread_rwlock_t data_rw;    /* lock for shared buffer */
static pthread_mutex_t data_mutex;  /* because conds are dumb */

static pthread_mutex_t dataset_mutex;

static volatile int do_exit = 0;
static rtlsdr_dev_t *dev = NULL;
static int lcm_post[17] = {1,1,1,3,1,5,3,7,1,9,5,11,3,13,7,15,1};
static int ACTUAL_BUF_LENGTH;

static BufferVOLK<float> h = BufferVOLK<float>( BITLEN );

static int *atan_lut = NULL;
static int atan_lut_size = 131072; /* 512 KB */
static int atan_lut_coef = 8;

static int debug_hop=0;
static int current_freq = 0;

static int verbose = 1;

struct fm_state
{
  int      now_r, now_j;
  int      pre_r, pre_j;
  int      prev_index;
  int      downsample;    /* min 1, max 256 */
  int      post_downsample;
  int      output_scale;
  int      squelch_level, conseq_squelch, squelch_hits, terminate_on_squelch;
  int      exit_flag;
  uint8_t  buf[MAXIMUM_BUF_LENGTH];
  uint32_t buf_len;
  int      signal[MAXIMUM_BUF_LENGTH];  /* 16 bit signed i/q pairs */
  int16_t  signal2[MAXIMUM_BUF_LENGTH]; /* signal has lowpass, signal2 has demod */
  int      signal_len;
  int      signal2_len;
  FILE     *file;
  int      edge;
  uint32_t freqs[FREQUENCIES_LIMIT];
  int      freq_len;
  int      freq_now;
  uint32_t sample_rate;
  int      output_rate;
  int      fir_enable;
  int      fir[256];  /* fir_len == downsample */
  int      fir_sum;
  int      custom_atan;
  int      deemph, deemph_a;
  int      now_lpr;
  int      prev_lpr_index;
  int      dc_block, dc_avg;
};

typedef struct {
  unsigned char mode;
  unsigned char addr[8];
  unsigned char ack;
  unsigned char label[3];
  unsigned char bid;
  unsigned char no[5];
  unsigned char fid[7];
  char txt[256];
  int crc;
} msg_t;

// ACARS decoder variables
static long rx_idx;
int c;
uint8_t rl;
int nbitl = 0;


// Constants.

static constexpr double halfPI = ( M_PI / 2.0 );
static constexpr double twoPI  = ( M_PI * 2.0 );
static constexpr double fourPI = ( M_PI * 4.0 );


static const std::string my_ident = "$Id: rtl_acars_ng.cc,v 1.7 2016/07/06 23:51:24 dennisg Exp dennisg $";


// Two routines to count the number of bit errors (i.e., the number of
// bits set to a 1 formed by an XOR or two words).

inline int
_count_bit_errors( uint8_t c1, uint8_t c2 ) {

  return bits_set[ c1 ^ c2 ];
}

inline int
_count_bit_errors( uint64_t c1, uint64_t c2 ) {

  const uint64_t x = c1 ^ c2;

    return
      bits_set[ uint8_t( x >> 56 ) & uint8_t( 0xff )] +
      bits_set[ uint8_t( x >> 48 ) & uint8_t( 0xff )] +
      bits_set[ uint8_t( x >> 40 ) & uint8_t( 0xff )] +
      bits_set[ uint8_t( x >> 32 ) & uint8_t( 0xff )] +
      bits_set[ uint8_t( x >> 24 ) & uint8_t( 0xff )] +
      bits_set[ uint8_t( x >> 16 ) & uint8_t( 0xff )] +
      bits_set[ uint8_t( x >>  8 ) & uint8_t( 0xff )] +
      bits_set[ uint8_t( x >>  0 ) & uint8_t( 0xff )];
}

std::ostream&
operator<<( std::ostream& o, const STATE& s ) {

  if( s == STATE::HEADL )
    o << "HEADL";
  else
    if( s == STATE::HEADF )
      o << "HEADF";
    else
      if( s == STATE::SYNC )
	o << "SYNC";
      else
	if( s == STATE::TXT )
	  o << "TXT";
	else
	  if( s == STATE::CRC1 )
	    o << "CRC1";
	  else
	    if( s == STATE::CRC2 )
	      o << "CRC2";
	    else
	      if( s == STATE::END )
		o << "END";
	      else
		o << "dunno";
  return o;
}


void
_dump_sync( const uint64_t checkPhrase,
	    const uint64_t checkWord ) {

  const uint64_t
    w1 = (  checkWord & 0xffffffffff ),
    w2 = ( ~checkWord & 0xffffffffff );
  const int
    err1 = _count_bit_errors( w1, checkPhrase ),
    err2 = _count_bit_errors( w2, checkPhrase );

  std::streamsize         width = std::cout.width();
  std::ios_base::fmtflags flags = std::cout.flags();
  char                    fill  = std::cout.fill();

  std::cout << "Check: "
	    << std::hex << std::showbase
	    << std::setw(14) << checkPhrase
	    << " ";
  std::cout << std::hex << std::showbase
	    << std::setw(14) << w1
	    << std::dec
	    << " "
	    << std::setw(2) << err1;
  if( err1 < 5 )
    std::cout << " *** ";
  else
    std::cout << "     ";
  std::cout << std::hex << std::showbase
	    << std::setw(14) << w2
	    << std::dec
	    << " "
	    << std::setw(2) << err2;
  if( err2 < 5 )
    std::cout << " *** ";
  else
    std::cout << "     ";
  std::cout << std::endl;

  std::cout.width( width );
  std::cout.setf( flags );
  std::cout.fill( fill );

}
   

void
_dump_bit_state_machine( void ) {

  std::cout << "c: ";
  for( size_t i = 0; i < bstat.csample.size(); ++i )
    std::cout << bstat.csample[i] << " ";
  std::cout << std::endl;

  std::cout << "h: ";
  for( size_t i = 0; i < bstat.hsample.size(); ++i )
    std::cout << bstat.hsample[i] << " ";
  std::cout << std::endl;

  std::cout << "l: ";
  for( size_t i = 0; i < bstat.lsample.size(); ++i )
    std::cout << bstat.lsample[i] << " ";
  std::cout << std::endl;

  std::cout << "i: ";
  for( size_t i = 0; i < bstat.isample.size(); ++i )
    std::cout << bstat.isample[i] << " ";
  std::cout << std::endl;

  std::cout << "q: ";
  for( size_t i = 0; i < bstat.qsample.size(); ++i )
    std::cout << bstat.qsample[i] << " ";
  std::cout << std::endl;

  std::cout << "phih= " << bstat.phih
	    << ", phil= " << bstat.phil << std::endl;
  std::cout << "dfh= " << bstat.dfh
	    << ", dfl= "<< bstat.dfl << std::endl;
  std::cout << "pC= " << bstat.pC
	    << ", ppC= "<< bstat.ppC << std::endl;
  std::cout << "sgI= " << bstat.sgI
	    << ", sgQ= "<< bstat.sgQ << std::endl;

  std::cout << "is: " << bstat.is    << std::endl;
  std::cout << "cl: " << bstat.clock << std::endl;
  std::cout << "ln: " << bstat.lin   << std::endl;
  std::cout << "ea: " << bstat.ea    << std::endl;

  std::cout << std::endl;

}


void usage(void)
{
  fprintf(stderr,
	  "rtl_fm, a simple narrow band FM demodulator for RTL2832 based DVB-T receivers\n\n"
	  "Use:\tnew_rtl_acars -f freq [-options] \n"
	  "\t[-F enables Hamming FIR (default: off/square)]\n"
	  "\t[-r debug hop]\n"
	  "\t[-v verbose]\n"
	  "\t[-h help (usage)]\n"
	  "\t-f frequency_to_tune_to [Hz]\n"
	  "\t (use multiple -f for scanning, requires squelch)\n"
	  "\t (ranges supported, -f 118M:137M:25k)\n"
	  "\t[-d device_index (default: 0)]\n"
	  "\t[-g tuner_gain (default: automatic)]\n"
	  "\t[-l squelch_level (default: 0/off)]\n"
	  "\t[-o oversampling (default: 1, 4 recommended)]\n"
	  "\t[-p ppm_error (default: 0)]\n"
	  "\t[-r squelch debug mode ]\n"
	  "\t[-t squelch_delay (default: 0)]\n"
	  "\t (+values will mute/scan, -values will exit)\n" );
  exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
  if (CTRL_C_EVENT == signum) {
    fprintf(stderr, "Signal caught, exiting!\n");
    do_exit = 1;
    //rtlsdr_cancel_async(dev);
    return TRUE;
  }
  return FALSE;
}
#else
static void sighandler(int signum)
{
  fprintf(stderr, "Signal caught, exiting!\n");
  do_exit = 1;
  //rtlsdr_cancel_async(dev);
}
#endif


void load_flights(void)
{
  FILE *f = fopen("datasets/flightroute2.txt", "r");
  if (!f) {
    fprintf(stderr, "Warning: datasets/flightroute2.txt data source not found\n");
    acars_flights[0].flightid = NULL;
    return;
  }

  int i = 0;

  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, f) != -1) {
    char *item = line;
    char *tabpos;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_flights[i].flightid = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_flights[i].from = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_flights[i].to = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    tabpos[0] = 0;
    acars_flights[i].airline = strdup(item);

    i++;
  }

  acars_flights[i].flightid = NULL;
  fclose(f);
  printf("Loaded: %i flights from dataset.....\n", i);
}




void load_airports(void)
{
  FILE *f = fopen("datasets/airports.txt", "r");
  if (!f) {
    fprintf(stderr, "Warning: datasets/airports.txt data source not found\n");
    acars_airports[0].code = NULL;
    return;
  }

  int i = 0;

  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, f) != -1) {
    char *item = line;
    char *tabpos;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_airports[i].name = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_airports[i].city = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_airports[i].country = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    tabpos[0] = 0;
    acars_airports[i].code = strdup(item);

    i++;
  }

  acars_airports[i].code = NULL;
  fclose(f);
  printf("Loaded: %i airports from dataset.....\n", i);
}


void load_aircrafts(void)
{
  FILE *f = fopen("datasets/aircrafts.txt", "r");
  if (!f) {
    fprintf(stderr, "Warning: datasets/aircrafts.txt data source not found\n");
    acars_aircrafts[0].registration = NULL;
    return;
  }

  int i = 0;

  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, f) != -1) {
    char *item = line;
    char *tabpos;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_aircrafts[i].registration = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_aircrafts[i].modes = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_aircrafts[i].manufacturer = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    if (!tabpos) {tabpos = (char*)malloc(1);tabpos[0] = 0;}
    acars_aircrafts[i].model = strdup(item);
    i++;
  }

  acars_aircrafts[i].registration = NULL;
  fclose(f);
  printf("Loaded: %i aircrafts from dataset.....\n", i);
}


void load_message_labels(void)
{
  FILE *f = fopen("datasets/acars_mls.txt", "r");
  if (!f) {
    fprintf(stderr, "Warning: datasets/acars_mls.txt data source not found\n");
    acars_mls[0].ml_code = NULL;
    return;
  }

  int i = 0;

  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, f) != -1) {
    char *item = line;
    char *tabpos;

    tabpos = strchr(item, '\t');
    if (!tabpos) { fprintf(stderr, "Parse error on line: %s\n", line); continue; }
    tabpos[0] = 0;
    acars_mls[i].ml_code = strdup(item);
    item = tabpos + 1;

    tabpos = strchr(item, '\t');
    tabpos[0] = 0;
    acars_mls[i].ml_label = strdup(item);

    i++;
  }

  acars_mls[i].ml_code = NULL;
  fclose(f);
  printf("Loaded: %i ACARS message labels from dataset.....\n", i);
}


bool
_getbit( const float sample, uint8_t& outbits ) {

  bool bt = false;
  
  assert( sample >= 0 );

  if( --bstat.is < 0 )
    bstat.is = BITLEN - 1;
  
  bstat.lin = ( 0.003 * std::fabs( sample )) + ( 0.997 * bstat.lin );
  
  /* VFOs */
  
  { float oscl, osch;
    
    const float
      s  = sample / bstat.lin,
      s2 = s * s;

    bstat.phih += Freqh - ( VFOPLL * bstat.dfh );
    if( bstat.phih >= fourPI )
      bstat.phih -= fourPI;
    bstat.dfh = 0.0;
    bstat.hsample[bstat.is] = s2 * std::sin( bstat.phih );
    for( int i = 0; i < ( BITLEN / 2 ); ++i )
      bstat.dfh += bstat.hsample[( bstat.is + i ) % BITLEN];
    osch = std::cos( bstat.phih / 2.0 );
    
    bstat.phil += Freql - ( VFOPLL * bstat.dfl );
    if( bstat.phil >= fourPI )
      bstat.phil -= fourPI;
    bstat.lsample[bstat.is] = s2 * std::sin( bstat.phil );
    bstat.dfl = 0.0;
    for( int i = 0; i < ( BITLEN / 2 ); ++i )
      bstat.dfl += bstat.lsample[( bstat.is + i ) % BITLEN];
    oscl = std::cos( bstat.phil / 2.0 );

    /* mix */
    
    bstat.isample[bstat.is] = s * ( oscl + osch );
    bstat.qsample[bstat.is] = s * ( oscl - osch );
    bstat.csample[bstat.is] = oscl * osch;

  }

  /* bit clock */
  
  if( ++bstat.clock >= ( BITLEN/4 + bstat.ea )) {
    
    bstat.clock = 0;
    
    /*  clock filter  */
    
    float C = 0.0;
    
    for( int i = 0; i < BITLEN; ++i )
      C += h[i] * bstat.csample[( bstat.is + i ) % BITLEN];
    
    if(( bstat.pC < C ) && ( bstat.pC < bstat.ppC )) {
      
      float Q = 0;
      
      /* integrator */
      
      for( int i = 0; i < BITLEN; ++i )
	Q += bstat.qsample[( bstat.is + i ) % BITLEN];
      
      if( bstat.sgQ == 0 ) {
	if( Q < 0 )
	  bstat.sgQ = -1;
	else
	  bstat.sgQ = 1;
      }
      
      outbits =
	( outbits >> 1 ) | uint8_t((( Q * bstat.sgQ ) > 0 ) ? 0x80 : 0 );
      bt = true;
      
      bstat.ea = -BITPLL * ( C - bstat.ppC );
      if( bstat.ea > 2.0 )
	bstat.ea =  2.0;
      if( bstat.ea < -2.0 )
	bstat.ea = -2.0;
      
    }
    
    if(( bstat.pC > C ) && ( bstat.pC > bstat.ppC )) {
      
      float I = 0;
      
      /* integrator */
      
      for( int i = 0; i < BITLEN; ++i )
	I += bstat.isample[( bstat.is + i ) % BITLEN];
      
      if( bstat.sgI == 0 ) {
	if( I < 0 )
	  bstat.sgI = -1;
	else
	  bstat.sgI = 1;
	
      }
      
      outbits =
	( outbits >> 1 ) | uint8_t((( I * bstat.sgI ) > 0 ) ? 0x80 : 0 );
      bt = true;
      
      bstat.ea = BITPLL * ( C - bstat.ppC );
      if( bstat.ea > 2.0 )
	bstat.ea = 2.0;
      if( bstat.ea < -2.0 )
	bstat.ea = -2.0;
    }
    
    bstat.ppC = bstat.pC;
    bstat.pC  = C;
    
  }
  
  return bt;
  
}


void
init_bits(void) {

  for( int i = 0; i < BITLEN; ++i ) 
    h[i] = ( twoPI * float(i) / float(BITLEN));
  volk_32f_sin_32f( h.get(), h.get(), h.size());
  
  for( int i = 0; i < BITLEN; ++i ) {
    bstat.hsample[i] = bstat.lsample[i] =
      bstat.isample[i] = bstat.qsample[i] =
      bstat.csample[i] = 0.0;
  }
  
  bstat.is = bstat.clock = 0;
  bstat.sgI = bstat.sgQ = 0;

  bstat.phih = bstat.phil = 0.0;
  bstat.dfh  = bstat.dfl  = 0.0;
  bstat.pC   = bstat.ppC  = 0.0;
  bstat.ea  = 0.0;
  bstat.lin = 1.0;

}

inline void
_reset_message_state_machine(void) {

  m_state.state             = STATE::HEADL;
  m_state.syncForming       = 0;
  m_state.syncBitsHave      = 0;
  m_state.consecutivePreKey = 0;
  m_state.crc               = 0;
  
  m_state.rawText.clear();
  
}

inline void
_reset_bit_state_machine( void ) {

  // Wouldn't it also make sense to clear the accumulators?
  
  bstat.sgI = bstat.sgQ = 0;

}

ssize_t
getline(char **linep, size_t *np, FILE *stream) {

  char *p = NULL;
  size_t i = 0;

  if (!linep || !np) {
    errno = EINVAL;
    return -1;
  }

  if (!(*linep) || !(*np)) {
    *np = 120;
    *linep = (char *)malloc(*np);
    if (!(*linep)) {
      return -1;
    }
  }

  flockfile(stream);

  p = *linep;
  int ch;
  for (ch = 0; (ch = getc_unlocked(stream)) != EOF;) {
    if (i > *np) {
      /* Grow *linep. */
      size_t m = *np * 2;
      char *s = (char *)realloc(*linep, m);

      if (!s) {
        int error = errno;
        funlockfile(stream);
        errno = error;
        return -1;
      }

      *linep = s;
      *np = m;
    }

    p[i] = ch;
    if ('\n' == ch) break;
    i += 1;
  }
  funlockfile(stream);

  /* Null-terminate the string. */
  if (i > *np) {
    /* Grow *linep. */
    size_t m = *np * 2;
    char *s = (char *)realloc(*linep, m);

    if (!s) {
      return -1;
    }

    *linep = s;
    *np = m;
  }

  p[i + 1] = '\0';
  return ((i > 0)? (ssize_t) i : -1);
}


int
build_mesg( const std::vector<uint8_t>& txt, msg_t* msg) {

  std::vector<char> m;
  int               k = 0;

  assert( msg );
  memset( msg, 0, sizeof( msg_t ));
	  
  // Remove framing and special characters (e.g., the SOH and the two
  // CRC bytes).

  for( size_t i = 1; i < ( txt.size() - 3 ); ++i ) {

    char r = char( txt[i] & 0x7f );
  
    if( r < ' ' && r != CR && r != LF )
      r = '.'; // was 0xa4 AR CHANGE: Set other placeholder
    
    m.push_back( r );
    
  }

  /* fill msg struct */

  msg->mode = m[k++];

  for( int i = 0; i < 7; ++i ) 
    msg->addr[i] = m[k++];
  
  /* ACK/NAK */
  msg->ack = m[k++];

  msg->label[0] = m[k++];
  msg->label[1] = m[k++];

  msg->bid = m[k];
  k++;
  k++;

  for( int i = 0; i < 4; ++i ) 
    msg->no[i] = m[k++];
  
  for( int i = 0; i < 6; ++i ) 
    msg->fid[i] = m[k++];

  for( size_t i = 0; size_t(k) < m.size(); ++i )
    msg->txt[i] = m[k++];

  return 1;
}


int
_getmesg( const uint8_t& r, msg_t* msg ) noexcept {

  assert( msg );
  
  // This is a confusing loop. The point of the loop is to allow a
  // state change to process the word a second time. Specifically,
  // when the state changes from PRE-KEY to BIT SYNC. In this case the
  // code has been processing PRE KEY words but suddenly the word
  // isn't PRE-KEY and it might be a forming BIT SYNC. In that case,
  // loop again until some condition is met.

  if( m_state.state != STATE::HEADL )
    if( verbose > 3 )
      std::cout << m_state.state << ": "
		<< std::hex << unsigned(r) << std::dec
		<< std::endl;
      
  do {

    switch( m_state.state ) {
      
      // PREKEY lost. Looking for PRE-KEY.
      
    case STATE::HEADL:
      
      if( r == PRE_KEY_CHAR ) {
	
	if( ++m_state.consecutivePreKey > m_state.consecutivePreKeyLim )
	  m_state.state = STATE::HEADF;
	
      } else {
	
	_reset_bit_state_machine();
	_reset_message_state_machine();
	
      }
      
      return 1;
      
      // PRE-KEY found. Keep looking for PRE-KEY characters. If the
      // character isn't a PRE-KEY then advance the state machine to
      // sync.
      
    case STATE::HEADF:
      
      // If the character isn't a PRE-KEY then we might be seeing the
      // start of the SYNC characters, so advance the state
      // machine. If the character is a PRE-KEY then we don't want to
      // risk consuming the first bits of the BIT SYNC, which are 11.

      if( r == PRE_KEY_CHAR ) {
	
	return 1;
	
      } else {
	
	m_state.state        = STATE::SYNC;
	m_state.syncForming  = 0;
	m_state.syncBitsHave = 0;
	
      }
      break;
      
      // If the character is the first bitsync then advance the state
      // machine to the second bitsync.
      
    case STATE::SYNC:
      
      if(( verbose > 3 ) && 0 )
	std::cout << "STATE::SYNC" << std::endl;
      
      { static const uint64_t syncCheck =
	  ( uint64_t( _to_odd( BIT_SYNC_1 ))  <<  0 ) |
	  ( uint64_t( _to_odd( BIT_SYNC_2 ))  <<  8 ) |
	  ( uint64_t( _to_odd( CHAR_SYNC_1 )) << 16 ) |
	  ( uint64_t( _to_odd( CHAR_SYNC_2 )) << 24 ) |
	  ( uint64_t( _to_odd( SOH ))         << 32 );
	
	int bitsConsumed = 0;
	
	for( int i = 0; i < 8; ++i ) {
	  
	  static const std::vector<uint8_t> mask = {
	    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
	  };
	  
	  // Add a bit from the passed word to the sync word.
	  
	  m_state.syncForming >>= 1;
	  if( mask[i] & r )
	    m_state.syncForming |= uint64_t(0x00008000000000);
	  ++bitsConsumed;
	  
	  // If we have collected enough bits to check for the SYNC
	  // words and SOH then do so.
	  
	  if( ++m_state.syncBitsHave >= 40  /* 5*8=40 */ ) {

            if( verbose > 3 )
	      _dump_sync( syncCheck, m_state.syncForming );
	    
	    if ( _count_bit_errors( m_state.syncForming, syncCheck ) <=
		 m_state.errLim ) {
	      
	      // The formed word has less than some number of errors
	      // and that is enough to declare SYNC and collect TXT.
	      
	      m_state.state = STATE::TXT;

	      m_state.rawText.clear();
	      m_state.rawText.push_back( _to_odd( SOH ));
	      
	      return bitsConsumed;
	      
	    }
	  }
	  
	  // If the number of bits collected to search for the SYNC
	  // words and SOH has been exhausted then start over.

	  if( m_state.syncBitsHave >= m_state.syncBitsLim ) {
	    
	    m_state.state = STATE::HEADL;
	    break;
	    
	  }
	}
	
	return bitsConsumed;
	
      }
      
      // Collect text characters until either an ETX or ETB character
      // is encountered or until the buffer fills. If the buffer fills
      // then discard the message.
      //
      // Clearly, if there is a bit error in ETX/ETB then it cannot be
      // absorbed. Need to fix that.
      
    case STATE::TXT:
      
      if( verbose > 2 )
	std::cout << "STATE::TXT size= "
		  << m_state.rawText.size()
		  << " + 1"
		  << std::endl;
      
      // Save the character.
      
      m_state.rawText.push_back( r );
      
      // If the buffer is full then reset the state machine.
      
#define BMAX ( MODE_BYTES + ADDRESS_BYTES + ACK_NAK_BYTES +	   \
	       LABEL_BYTES + BLOCK_ID_BYTES + STX_BYTES +	   \
	       SEQ_NUM_BYTES + FLIGHT_NUM_BYTES + MAX_TEXT_BYTES + \
	       ETX_BYTES )
      
      if(  m_state.rawText.size() > BMAX ) {
	
	m_state.state = STATE::HEADL;
	break;
	
      }
      
      // If it's an ETX or ETB then advance the state machine to
      // collect the CRC.
      
      if(( r == _to_odd( ETX )) || ( r == _to_odd( ETB ))) {
	
	m_state.state = STATE::CRC1;
	return 8;
	
      }

      return 8;
      
      // For the states CRC1 and CRC2 simply collect the CRC bytes.
      
    case STATE::CRC1:
      
      if( verbose > 2 )
	std::cout << "STATE::CRC1" << std::endl;

      m_state.rawText.push_back( r );
      m_state.state = STATE::CRC2;
      
      return 8;
      
    case STATE::CRC2:
      
      if( verbose > 2 )
	std::cout << "STATE::CRC1" << std::endl;
      
      m_state.rawText.push_back( r );
      m_state.state = STATE::END;
      
      return 8;
      
      // First, when the end state is reached then reset the state
      // machine.
      
    case STATE::END:
      
      if( verbose > 2 )
	std::cout << "STATE::END" << std::endl;

      // The next state is to start over.
      
      m_state.state = STATE::HEADL;

      // Check the CRC.

      if( gen_crc( m_state.rawText.begin(),
		   m_state.rawText.cend()) == 0x0000 ) {

	m_state.crc = 0;

	build_mesg( m_state.rawText, msg );

	return -1;
	
      } else {

	// Correct one bit error.

	for( size_t i = 0; i < m_state.rawText.size(); ++i ) {
	  for( int j = 0; j < 8; ++j ) {

	    m_state.rawText[i] ^= mask[j];

	    if( gen_crc( m_state.rawText.begin(),
			 m_state.rawText.cend()) == 0x0000 ) {

	      m_state.crc = 0;

	      build_mesg( m_state.rawText, msg );
	      
	      return -1;

	    } else
	      m_state.rawText[i] ^= mask[j];

	  }
	}
      }

      std::cout << std::endl << "CRC check failure" << std::endl;
#ifdef dpgdebug0
      { std::streamsize         width = std::cout.width();
	std::ios_base::fmtflags flags = std::cout.flags();
	char                    fill  = std::cout.fill();
	
	for( size_t i = 0; i < m_state.rawText.size(); ++i )
	  std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2)
		    << unsigned( m_state.rawText[i])
		    << std::dec
		    << "(" << char(m_state.rawText[i]&0x7f ) << ") ";
	std::cout << std::endl << std::endl;
	
	std::cout.width( width );
	std::cout.setf( flags );
	std::cout.fill( fill );
      }
#endif
      
      return 8;
      
    }
  } while( true );
}


void process_qv(char *txt)
{
  switch (txt[0])
    {
    case '1': printf("\nAutotune reject reason: Contrary to airline preference\n");break;
    case '2': printf("\nATN session in progress\n");break;
    case '3': printf("\nAutotune uplink format error\n");break;
    default: printf("\nAutotune reject reason: unknown\n");
    }
}


void process_sa(char *txt)
{
  printf("Version: %c\n",txt[0]);
  if (txt[1]=='E')
    printf("Link state: Established\n");
  else
    if (txt[1]=='L')
      printf("Link state: Lost\n");
    else
      printf("Link state: Unknown\n");
  
  switch (txt[2]) {
    case 'V': printf("Link type: VHF ACARS\n");break;
    case 'S': printf("Link type: Generic SATCOM\n");break;
    case 'H': printf("Link type: HF\n");break;
    case 'G': printf("Link type: GlobalStar SATCOM\n");break;
    case 'C': printf("Link type: ICO SATCOM\n");break;
    case '2': printf("Link type: VDL Mode 2\n");break;
    case 'X': printf("Link type: Inmarsat Aero\n");break;
    case 'I': printf("Link type: Irridium SATCOM\n");break;
    default:  printf("Link type: Unknown\n");
    }
  printf("Event occured at: %c%c:%c%c:%c%c\n",txt[3],txt[4],txt[5],txt[6],txt[7],txt[8]);
}


void
process_5u(char *txt) {

  char airport[4];
  int cur=0;
  int cur2=0;
  int i=0;
    
  printf("Weather report requested from: ");
  while (txt[cur]!=0) {
    
    if ((txt[cur]<='Z')&&(txt[cur]>='A')) {
      
	  airport[cur2]=txt[cur];
	  cur2++;

	  if (cur2==4) {
	    
	      i=0;
	      while(acars_airports[i].code) {
		
		  const char *regtmp = (const char *) airport;

		  while (regtmp[0] == '.')
		    regtmp++;

		  if(!strcmp(acars_airports[i].code, regtmp))
		    printf("%s (%s) ",acars_airports[i].name,acars_airports[i].city);
		  i++;
	      }
	      cur2 = 0;
	  }
    } else
      cur2=0;
    cur++;
  }
  printf("\n");
}

void process_q1(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("OFF event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
  printf("ON event occured at: %c%c:%c%c\n",txt[12],txt[13],txt[14],txt[15]);
  printf("IN event occured at: %c%c:%c%c\n",txt[16],txt[17],txt[18],txt[19]);
  printf("Fuel: %c%c%c%c\n",txt[20],txt[21],txt[22],txt[23]);
  printf("Destination station: %c%c%c%c\n",txt[24],txt[25],txt[26],txt[27]);
}

void process_q2(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("ETA: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Fuel: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qa(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Boarded fuel: %c%c%c%c%c\n",txt[8],txt[9],txt[10],txt[11],txt[12]);
  printf("Fuel quantity: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
}

void process_qb(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("OFF event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
}

void process_qc(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("ON event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
}

void process_qd(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("IN event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Boarded fuel: %c%c%c%c%c\n",txt[8],txt[9],txt[10],txt[11],txt[12]);
  printf("Fuel quantity: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
  printf("Captain/First officer ID: %c\n",txt[17]);
}


void process_qe(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Boarded fuel: %c%c%c%c%c\n",txt[8],txt[9],txt[10],txt[11],txt[12]);
  printf("Fuel quantity: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
  printf("Destination station: %c%c%c\n",txt[17],txt[18],txt[19]);
}

void process_qf(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("OFF event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Destination station: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}


void process_qg(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Return IN event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qh(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("OUT event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
}


void process_qk(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("ON event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Destination station: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_ql(char *txt)
{
  printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("IN event occured at: %c%c:%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Fuel quantity: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
  printf("Captain/First officer ID: %c\n",txt[12]);
  printf("Departure station: %c%c%c%c\n",txt[13],txt[14],txt[15],txt[16]);
}

void process_qm(char *txt)
{
  printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("Fuel quantity: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("Departure station: %c%c%c%c\n",txt[8],txt[9],txt[10],txt[11]);
  printf("Category of landing: %c\n",txt[12]);
}

void process_qn(char *txt)
{
  printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("New destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("ETA at diversion station: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
  printf("Fuel quantity: %c%c%c%c\n",txt[12],txt[13],txt[14],txt[15]);
  printf("Flight segment originating station: %c%c%c%c\n",txt[16],txt[17],txt[18],txt[19]);
}


void process_qp(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("OUT event occured at: %c%c:%c%c\n",txt[7],txt[8],txt[10],txt[11]);
  printf("Boarded fuel: %c%c%c%c\n",txt[12],txt[13],txt[14],txt[15]);
}

void process_qq(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("OFF event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qr(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("ON event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
}

void process_qs(char *txt)
{
  printf("Destination station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("New destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("IN event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
  printf("Fuel quantity: %c%c%c%c\n",txt[12],txt[13],txt[14],txt[15]);
  printf("Flight segment originating station: %c%c%c%c\n",txt[16],txt[17],txt[18],txt[19]);
  printf("Captain/First officer ID: %c\n",txt[20]);
}

void process_qt(char *txt)
{
  printf("Departure station: %c%c%c%c\n",txt[0],txt[1],txt[2],txt[3]);
  printf("Destination station: %c%c%c%c\n",txt[4],txt[5],txt[6],txt[7]);
  printf("OUT event occured at: %c%c:%c%c\n",txt[8],txt[9],txt[10],txt[11]);
  printf("Return IN event occured at: %c%c:%c%c\n",txt[12],txt[13],txt[14],txt[15]);
  printf("Fuel onboard: %c%c%c%c\n",txt[16],txt[17],txt[18],txt[19]);
}

void process_57(char *txt)
{
  printf("Current position: %c%c%c%c%c\n",txt[0],txt[1],txt[2],txt[3],txt[4]);
  printf("Current time: %c%c:%c%c\n",txt[5],txt[6],txt[7],txt[8]);
  printf("Flight level: %c%c%c\n",txt[9],txt[10],txt[11]);
  printf("Next report point: %c%c%c%c%c\n",txt[12],txt[13],txt[14],txt[15],txt[16]);
  printf("Time over: %c%c:%c%c\n",txt[17],txt[18],txt[19],txt[20]);
  printf("Fuel onboard: %c%c%c%c\n",txt[21],txt[22],txt[23],txt[24]);
  printf("Static air temp: %c%c%c\n",txt[25],txt[26],txt[27]);
  printf("Wind direction: %c%c%c deg.\n",txt[28],txt[29],txt[30]);
  printf("Wind speed: %c%c%c knots.\n",txt[31],txt[32],txt[33]);
  printf("Sky condition: %c%c%c%c%c%c%c%c\n",txt[34],txt[35],txt[36],txt[37],txt[38],txt[39],txt[40],txt[41]);
  printf("Turbulence: %c%c%c%c%c%c%c%c\n",txt[42],txt[43],txt[44],txt[45],txt[46],txt[47],txt[48],txt[49]);
  printf("Cruising speed: %c%c%c%c%c\n",txt[50],txt[51],txt[52],txt[54],txt[54]);
}


void process_h1(char *txt)
{
  if (!strncmp(txt,"#DF",3)) printf("Source: Digital Flight Data Acquisition Unit\n");
  if (!strncmp(txt,"#CF",3)) printf("Source: Central Fault Display\n");
  if (!strncmp(txt,"#M1",3)) printf("Source: Flight Management Computer, Left\n");
  if (!strncmp(txt,"#M2",3)) printf("Source: Flight Management Computer, Right\n");
  if (!strncmp(txt,"#M3",3)) printf("Source: Flight Management Computer, Center\n");
  if (!strncmp(txt,"#MD",3)) printf("Source: Flight Management Computer, Selected\n");
  if (!strncmp(txt,"#EC",3)) printf("Source: Engine Display System\n");
  if (!strncmp(txt,"#EI",3)) printf("Source: Engine Indicating System\n");
  if (!strncmp(txt,"#PS",3)) printf("Source: Keyboard/Display Unit\n");
  if (!strncmp(txt,"#S1",3)) printf("Source: SDU, Left\n");
  if (!strncmp(txt,"#S2",3)) printf("Source: SDU, Right\n");
  if (!strncmp(txt,"#SD",3)) printf("Source: SDU, Selected\n");
  if (!strncmp(txt,"#T",2)) printf("Source: Cabin Terminal Message\n");
  if (!strncmp(txt,"#WO",3)) printf("Source: Weather Observation Report\n");
}


void process_54(char *txt)
{
  printf("Frequency (MHZ): %c%c%c.%c%c%c\n",txt[0],txt[1],txt[2],txt[3],txt[4],txt[5]);
}


int is_flight_num(const uint8_t* text)
{
  int a=0;
  int ok=1,dig=0;

  while (a<6) 
    {
      if (!((text[a]=='-') || (text[a]=='.') || ((text[a]<='Z') && (text[a]>='A')) || ((text[a]>='0') && (text[a]<='9')))) ok = 0;
      a++;
    }
  if (!((text[2]>='0') && (text[a]<='9'))) ok = 0;
  for (a=3;a<6;a++) if (((text[2]>='0') && (text[a]<='9'))) dig = 1;
  if (dig==0) ok=0;
  return ok;
}


void print_mesg(msg_t * msg) {

  time_t     t;
  struct tm* tmp;
  long       i = 0;

  printf("\n[BEGIN_MESSAGE]----------------------------------------------------------\n\n");
  printf("RX_IDX: %ld\n", rx_idx);
  if (msg->crc) printf("CRC: Bad, corrected\n");
  else printf("CRC: Correct\n");
  t = time(NULL);
  tmp = localtime(&t);
  printf("Timestamp: %02d/%02d/%04d %02d:%02d\n",	     tmp->tm_mday, tmp->tm_mon + 1, tmp->tm_year + 1900,
	 tmp->tm_hour, tmp->tm_min);
  printf("ACARS mode: %c \n", msg->mode);
  printf("Message label: %s ", msg->label);

  i=0;
  while(acars_mls[i].ml_code){
    if(!strcmp(acars_mls[i].ml_code, (const char*)msg->label)){
      printf("(%s)\n",acars_mls[i].ml_label);
    }
    i++;
  }

  printf("Aircraft reg: %s, ", msg->addr);
  printf("flight id: %s\n", msg->fid);
  i=0;
  if ((msg->addr)&&(strlen(msg->addr)<8)&&(strlen(msg->addr)>1)&&(strcmp(msg->addr,".......")!=0))
    {
      while(acars_aircrafts[i].registration)
	{
	  char regtmp[8];
	  memset(regtmp,0,8);
	  int ind = 0;
	  while ((ind<8)&&(msg->addr[ind]=='.')) ind++;
	  strcpy(regtmp,&msg->addr[ind]);

	  int len = strlen(regtmp);
	  if ((len>0)&&(!strncmp(acars_aircrafts[i].registration, regtmp,len))){
	    printf("Aircraft: %s \n",acars_aircrafts[i].manufacturer);
	    printf("Registration: %s \n",acars_aircrafts[i].registration);
	    printf("Mode-S ID: %s\n",acars_aircrafts[i].modes);
	    goto aircraft_finished;
	  }
	  i++;
	}
    }

 aircraft_finished:

  i=0;
  int found=0;
  int found2=0;
  int found3=0;
	
  char regtmp[8];
  memset(regtmp,0,8);
  int ind = 0;
  regtmp[0]=msg->fid[0];
  regtmp[1]=msg->fid[1];
  ind = 1;
  int correct = is_flight_num(msg->fid);
  while ((ind<7)/*&&(msg->fid[ind]=='0')*/) 
    {
      regtmp[2]='0';
      if (ind>1) strncpy(&regtmp[3], &msg->fid[ind], 7-ind);
      else strncpy(&regtmp[2], &msg->fid[ind+1], 4);
      ind++;
      if (strlen(msg->fid)>1) while(acars_flights[i].flightid){
	  if ((!found)&&(!strncmp(acars_flights[i].flightid, regtmp,2))&&(correct)) {
	    printf("Airline: %s \n",acars_flights[i].airline);
	    found++;
	  }

	  if ((correct)&&(!strncmp(acars_flights[i].flightid, regtmp,2)) && (!strncmp(acars_flights[i].flightid+3, regtmp+3,strlen(regtmp+3)))&&(strlen(regtmp+3)>0))
	    {
	      long x = 0;
	      while((acars_airports[x].code)&&(found2==0)){
		if(!strcmp(acars_airports[x].code, acars_flights[i].from)){
		  printf("From: %s - %s (%s, %s) \n",acars_airports[x].code,acars_airports[x].name,acars_airports[x].city,acars_airports[x].country);
		  found2++;
		  break;
		}
		x++;
	      }
	      x=0;
	      while((acars_airports[x].code)&&(found3==0)){
		if(!strcmp(acars_airports[x].code, acars_flights[i].to)){
		  printf("To: %s - %s (%s, %s) \n",acars_airports[x].code,acars_airports[x].name,acars_airports[x].city,acars_airports[x].country);
		  found3++;
		  break;
		}
		x++;
	      }
	    }
	  i++;
	}
    }


  printf("\nBlock id: %d, ", (int) msg->bid);
  printf(" msg. no: %s\n", msg->no);
  if (!strcmp(msg->label,"QV")) process_qv(msg->txt);
  if (!strcmp(msg->label,"5U")) process_5u(msg->txt);
  if (!strcmp(msg->label,"SA")) process_sa(msg->txt);
  if (!strcmp(msg->label,"Q1")) process_q1(msg->txt);
  if (!strcmp(msg->label,"Q2")) process_q2(msg->txt);
  if (!strcmp(msg->label,"QA")) process_qa(msg->txt);
  if (!strcmp(msg->label,"QB")) process_qb(msg->txt);
  if (!strcmp(msg->label,"QC")) process_qc(msg->txt);
  if (!strcmp(msg->label,"QD")) process_qd(msg->txt);
  if (!strcmp(msg->label,"QE")) process_qr(msg->txt);
  if (!strcmp(msg->label,"QF")) process_qf(msg->txt);
  if (!strcmp(msg->label,"QG")) process_qg(msg->txt);
  if (!strcmp(msg->label,"QH")) process_qh(msg->txt);
  if (!strcmp(msg->label,"QK")) process_qk(msg->txt);
  if (!strcmp(msg->label,"QL")) process_ql(msg->txt);
  if (!strcmp(msg->label,"QM")) process_qm(msg->txt);
  if (!strcmp(msg->label,"QN")) process_qn(msg->txt);
  if (!strcmp(msg->label,"QP")) process_qp(msg->txt);
  if (!strcmp(msg->label,"QQ")) process_qq(msg->txt);
  if (!strcmp(msg->label,"QR")) process_qr(msg->txt);
  if (!strcmp(msg->label,"QS")) process_qs(msg->txt);
  if (!strcmp(msg->label,"QT")) process_qt(msg->txt);
  if (!strcmp(msg->label,"57")) process_57(msg->txt);
  if (!strcmp(msg->label,"H1")) process_h1(msg->txt);
  if (!strcmp(msg->label,"54")) process_54(msg->txt);

  printf("Message content:-\n%s", msg->txt);



  rx_idx++;

  printf
    ("\n\n[END_MESSAGE ]------------------------------------------------------------\n\n");

}

///////// END OF ACARS ROUTINES //////////






/* more cond dumbness */
#define safe_cond_signal(n, m) pthread_mutex_lock(m); pthread_cond_signal(n); pthread_mutex_unlock(m)
#define safe_cond_wait(n, m) pthread_mutex_lock(m); pthread_cond_wait(n, m); pthread_mutex_unlock(m)


/* 90 rotation is 1+0j, 0+1j, -1+0j, 0-1j
   or [0, 1, -3, 2, -4, -5, 7, -6] */

void
rotate_90( uint8_t* buf, uint32_t len ) {
  
  for( uint32_t i = 0; i < len; i += 8 ) {

    uint8_t tmp;
    
    /* uint8_t negation = 255 - x */

    tmp = 255 - buf[i+3];
    buf[i+3] = buf[i+2];
    buf[i+2] = tmp;

    buf[i+4] = 255 - buf[i+4];
    buf[i+5] = 255 - buf[i+5];

    tmp = 255 - buf[i+6];
    buf[i+6] = buf[i+7];
    buf[i+7] = tmp;
  }
}

void low_pass(struct fm_state *fm, unsigned char *buf, uint32_t len)
/* simple square window FIR */
{
  int i=0, i2=0, seq=0;
  while (i < (int)len) {
    fm->now_r += ((int)buf[i]   - 127);
    fm->now_j += ((int)buf[i+1] - 127);
    i += 2;
    fm->prev_index++;

    if ( (fm->prev_index<(fm->downsample)) ) continue;
    if ((seq%2)==1)
      {
	// signal is ~10khz wide, don't need whole 48khz
	// eliminate some RF noise by attenuating stuff outside 24khz a bit
	fm->signal[i2]   = (fm->now_r*5)/(8); 
	fm->signal[i2+1] = (fm->now_j*5)/(8);
      }
    else
      {
	fm->signal[i2]   = fm->now_r;// * fm->output_scale;
	fm->signal[i2+1] = fm->now_j;// * fm->output_scale;
      }
    seq++;
    fm->prev_index = 0;
    fm->now_r = 0;
    fm->now_j = 0;
    i2 += 2;
  }
  fm->signal_len = i2;
}


void build_fir(struct fm_state *fm)
/* hamming */
/* point = sum(sample[i] * fir[i] * fir_len / fir_sum) */
{
  double a, b, w, N1;
  int i, len;
  len = fm->downsample;
  a = 25.0/46.0;
  b = 21.0/46.0;
  N1 = (double)(len-1);
  for(i = 0; i < len; i++) {
    w = a - b*cos(2*i*M_PI/N1);
    fm->fir[i] = (int)(w * 255);
  }
  fm->fir_sum = 0;
  for(i = 0; i < len; i++) {
    fm->fir_sum += fm->fir[i];
  }
}


void low_pass_fir(struct fm_state *fm, unsigned char *buf, uint32_t len)
/* perform an arbitrary FIR, doubles CPU use */
// possibly bugged, or overflowing
{
  int i=0, i2=0, i3=0;
  while (i < (int)len) {
    i3 = fm->prev_index;
    fm->now_r += ((int)buf[i]   - 127) * fm->fir[i3];
    fm->now_j += ((int)buf[i+1] - 127) * fm->fir[i3];
    i += 2;
    fm->prev_index++;
		
    if (fm->prev_index < fm->downsample) {
      continue;
    }
    fm->now_r *= fm->downsample;
    fm->now_j *= fm->downsample;
    fm->now_r /= fm->fir_sum;
    fm->now_j /= fm->fir_sum;
    fm->signal[i2]   = fm->now_r; //* fm->output_scale;
    fm->signal[i2+1] = fm->now_j; //* fm->output_scale;
    fm->prev_index = 0;
    fm->now_r = 0;
    fm->now_j = 0;
    i2 += 2;
  }
  fm->signal_len = i2;
}


int low_pass_simple(int16_t *signal2, int len, int step)
// no wrap around, length must be multiple of step
{
  int i, i2, sum;
  for(i=0; i < len; i+=step) {
    sum = 0;
    for(i2=0; i2<step; i2++)
      sum += (int)signal2[i + i2];
   
    //signal2[i/step] = (int16_t)(sum / step);
    signal2[i/step] = (int16_t)(sum);
  }
  signal2[i/step + 1] = signal2[i/step];
  return len / step;
}


void low_pass_real(struct fm_state *fm)
/* simple square window FIR */
// add support for upsampling?
{
  int i=0, i2=0;
  int fast = (int)fm->sample_rate / (fm->post_downsample);
  int slow = fm->output_rate;
  while (i < fm->signal2_len) {
    fm->now_lpr += fm->signal2[i];
    i++;
    fm->prev_lpr_index += slow;
    if (fm->prev_lpr_index < fast) {
      continue;
    }
    fm->signal2[i2] = (int16_t)(fm->now_lpr / (fast/slow));
    fm->prev_lpr_index -= fast;
    fm->now_lpr = 0;
    i2 += 1;
  }
  fm->signal2_len = i2;
}


/* define our own complex math ops
   because ARMv5 has no hardware float */

inline void
multiply(int ar, int aj, int br, int bj, int *cr, int *cj)
{
  *cr = ar*br - aj*bj;
  *cj = aj*br + ar*bj;
}


int polar_discriminant(int ar, int aj, int br, int bj)
{
  int cr, cj;
  double angle;
  multiply(ar, aj, br, -bj, &cr, &cj);
  angle = atan2((double)cj, (double)cr);
  return (int)(angle / 3.14159 * (1<<14));
}


int fast_atan2(int y, int x)
/* pre scaled for int16 */
{
  int yabs, angle;
  int pi4=(1<<12), pi34=3*(1<<12);  // note pi = 1<<14
  if (x==0 && y==0) {
    return 0;
  }
  yabs = y;
  if (yabs < 0) {
    yabs = -yabs;
  }
  if (x >= 0) {
    angle = pi4  - pi4 * (x-yabs) / (x+yabs);
  } else {
    angle = pi34 - pi4 * (x+yabs) / (yabs-x);
  }
  if (y < 0) {
    return -angle;
  }
  return angle;
}


int polar_disc_fast(int ar, int aj, int br, int bj)
{
  int cr, cj;
  multiply(ar, aj, br, -bj, &cr, &cj);
  return fast_atan2(cj, cr);
}


int atan_lut_init()
{
  int i = 0;

  atan_lut = (int*)malloc(atan_lut_size * sizeof(int));

  for (i = 0; i < atan_lut_size; i++) {
    atan_lut[i] = (int) (atan((double) i / (1<<atan_lut_coef)) / 3.14159 * (1<<14));
  }

  return 0;
}


int polar_disc_lut(int ar, int aj, int br, int bj)
{
  int cr, cj, x, x_abs;

  multiply(ar, aj, br, -bj, &cr, &cj);

  /* special cases */
  if (cr == 0 || cj == 0) {
    if (cr == 0 && cj == 0)
      {return 0;}
    if (cr == 0 && cj > 0)
      {return 1 << 13;}
    if (cr == 0 && cj < 0)
      {return -(1 << 13);}
    if (cj == 0 && cr > 0)
      {return 0;}
    if (cj == 0 && cr < 0)
      {return 1 << 14;}
  }

  /* real range -32768 - 32768 use 64x range -> absolute maximum: 2097152 */
  x = (cj << atan_lut_coef) / cr;
  x_abs = abs(x);

  if (x_abs >= atan_lut_size) {
    /* we can use linear range, but it is not necessary */
    return (cj > 0) ? 1<<13 : -1<<13;
  }

  if (x > 0) {
    return (cj > 0) ? atan_lut[x] : atan_lut[x] - (1<<14);
  } else {
    return (cj > 0) ? (1<<14) - atan_lut[-x] : -atan_lut[-x];
  }

  return 0;
}


void am_demod(struct fm_state *fm)
// todo, fix this extreme laziness
{
  int i, pcm;
  for (i = 0; i < (fm->signal_len); i += 2) {
    // hypot uses floats but won't overflow
    //fm->signal2[i/2] = (int16_t)hypot(fm->signal[i], fm->signal[i+1]);
    pcm = fm->signal[i] * fm->signal[i];
    pcm += fm->signal[i+1] * fm->signal[i+1];
    fm->signal2[i/2] = (int16_t)sqrt(pcm) * fm->output_scale;
    // Milen: add some gain to signal
    fm->signal2[i/2] *= 8;
  }
  fm->signal2_len = fm->signal_len/2;
  // lowpass? (3khz)  highpass?  (dc)
}


void deemph_filter(struct fm_state *fm)
{
  static int avg;  // cheating...
  int i, d;
  // de-emph IIR
  // avg = avg * (1 - alpha) + sample * alpha;
  for (i = 0; i < fm->signal2_len; i++) {
    d = fm->signal2[i] - avg;
    if (d > 0) {
      avg += (d + fm->deemph_a/2) / fm->deemph_a;
    } else {
      avg += (d - fm->deemph_a/2) / fm->deemph_a;
    }
    fm->signal2[i] = (int16_t)avg;
  }
}


void dc_block_filter(struct fm_state *fm)
{
  int i, avg;
  int64_t sum = 0;
  for (i=0; i < fm->signal2_len; i++) {
    sum += fm->signal2[i];
  }
  avg = sum / fm->signal2_len;
  avg = (avg + fm->dc_avg * 9) / 10;
  for (i=0; i < fm->signal2_len; i++) {
    fm->signal2[i] -= avg;
  }
  fm->dc_avg = avg;
}


int mad(const int *samples, int len, int step)
/* mean average deviation */
{
  int i=0, sum=0, ave=0;
  if (len == 0)
    {return 0;}
  for (i=0; i<len; i+=step) {
    sum += samples[i];
  }
  ave = sum / (len * step);
  sum = 0;
  for (i=0; i<len; i+=step) {
    sum += abs(samples[i] - ave);
  }
  return sum / (len / step);
}


int post_squelch(struct fm_state *fm)
/* returns 1 for active signal, 0 for no signal */
{
  int dev_r, dev_j, len, sq_l;
  /* only for small samples, big samples need chunk processing */
  len = fm->signal_len;
  sq_l = fm->squelch_level;
  dev_r = mad(&(fm->signal[0]), len, 2);
  dev_j = mad(&(fm->signal[1]), len, 2);
  //fprintf(stderr,"shits: %d dr: %d dj: %d sql: %d\n ",fm->squelch_hits,dev_r,dev_j,sq_l);

  if ((dev_r > sq_l) || (dev_j > sq_l)) {
    fm->squelch_hits = 0;
    return 1;
  }
  fm->squelch_hits++;
  return 0;
}


static void optimal_settings(struct fm_state *fm, int freq, int hopping)
{
  int r, capture_freq, capture_rate;
  fm->downsample = (1000000 / fm->sample_rate) + 1;

  fm->freq_now = freq;
  capture_rate = fm->downsample * fm->sample_rate;
  capture_freq = fm->freqs[freq] + capture_rate/4;
  capture_freq += fm->edge * fm->sample_rate / 2;
  fm->output_scale = (1<<15) / (128 * fm->downsample);
  if (fm->output_scale < 1) 
    fm->output_scale = 1;
  /* Set the frequency */
  r = rtlsdr_set_center_freq(dev, (uint32_t)capture_freq);
  if (hopping) {
    return;}
		
  // Milen: don't need 48khz signal, set cutoff at ~16khz
  //fm->downsample /=2;
  //fm->post_downsample*=2;

		
  fprintf(stderr, "Oversampling input by: %ix.\n", fm->downsample);
  fprintf(stderr, "Oversampling output by: %ix.\n", fm->post_downsample);
  fprintf(stderr, "Buffer size: %0.2fms\n",
	  1000 * 0.5 * (float)ACTUAL_BUF_LENGTH / (float)capture_rate);
  if (r < 0) {
    fprintf(stderr, "WARNING: Failed to set center freq.\n");}
  else {
    fprintf(stderr, "Tuned to %u Hz.\n", capture_freq);}

  /* Set the sample rate */
  fprintf(stderr, "Sampling at %u Hz.\n", capture_rate);
  if (fm->output_rate > 0) {
    fprintf(stderr, "Output at %u Hz.\n", fm->output_rate);
  } else {
    fprintf(stderr, "Output at %u Hz.\n", fm->sample_rate/fm->post_downsample);}
  r = rtlsdr_set_sample_rate(dev, (uint32_t)capture_rate);
  if (r < 0) {
    fprintf(stderr, "WARNING: Failed to set sample rate.\n");}

}


void full_demod(struct fm_state *fm)
{
  uint8_t dump[BUFFER_DUMP];
  int i, sr, freq_next, n_read, hop = 0;
  pthread_rwlock_wrlock(&data_rw);
  rotate_90(fm->buf, fm->buf_len);
  if (fm->fir_enable) {
    low_pass_fir(fm, fm->buf, fm->buf_len);
  } else {
    low_pass(fm, fm->buf, fm->buf_len);
  }
  pthread_rwlock_unlock(&data_rw);

  sr = post_squelch(fm);
  if (!sr && fm->squelch_hits > 1/*fm->conseq_squelch*/) {
    //if (fm->terminate_on_squelch) {
    //	fm->exit_flag = 1;}
    if (fm->freq_len == 1) {  /* mute */
      for (i=0; i<fm->signal_len; i++) {
	fm->signal2[i] = 0;}
    }  else {
      hop = 1;
    }
  }
  if (fm->post_downsample > 1)
    fm->signal2_len = low_pass_simple( fm->signal2, fm->signal2_len,
				       fm->post_downsample);
  if (fm->output_rate > 0) 
    low_pass_real(fm);
  if (fm->deemph) 
    deemph_filter(fm);
  if (fm->dc_block) 
    dc_block_filter(fm);
  
  /* ignore under runs for now */

  //fwrite(fm->signal2, 2, fm->signal2_len, fm->file);
  if (hop) {
    if (debug_hop) fprintf(stderr,"Hopping freq!\n");
    freq_next = (fm->freq_now + 1) % fm->freq_len;
    optimal_settings(fm, freq_next, 1);
    current_freq = fm->freqs[freq_next];
    fm->squelch_hits = fm->conseq_squelch + 1;  /* hair trigger */
    /* wait for settling and flush buffer */
    //usleep(5000);
    usleep(1000);
    rtlsdr_read_sync(dev, &dump, BUFFER_DUMP, &n_read);
    if (n_read != BUFFER_DUMP) {
      fprintf(stderr, "Error: bad retune.\n");}
  } else
    am_demod(fm);
  
}


void acars_decode(struct fm_state *fm) {

  int16_t* sample = fm->signal2;

  for( int ind = 0; ind < fm->signal2_len; ++ind ) {
    if( _getbit( sample[ind], rl )) {
      if( ++nbitl >= 8 ) {
	do { 

	  msg_t     msgl;
	  const int bitsConsumed = _getmesg( rl, &msgl );
	  
	  if( bitsConsumed == -1 ) {

	    print_mesg( &msgl );
	    nbitl  = 0;

	  } else
	    nbitl -= bitsConsumed;

	} while( nbitl >= 8 );
      }
    }
  }
}


void
rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
  struct fm_state *fm2 = (struct fm_state *)ctx;
  if (do_exit) {
    return;}
  if (!ctx) {
    return;}
  pthread_rwlock_wrlock(&data_rw);
  memcpy(fm2->buf, buf, len);
  fm2->buf_len = len;
  pthread_rwlock_unlock(&data_rw);
  safe_cond_signal(&data_ready, &data_mutex);
  /* single threaded uses 25% less CPU? */
  /* full_demod(fm2); */
}


static void sync_read(unsigned char *buf, uint32_t len, struct fm_state *fm)
{
  int r, n_read;
  r = rtlsdr_read_sync(dev, buf, len, &n_read);
  if (r < 0) {
    fprintf(stderr, "WARNING: sync read failed.\n");
    return;
  }
  pthread_rwlock_wrlock(&data_rw);
  memcpy(fm->buf, buf, len);
  fm->buf_len = len;
  pthread_rwlock_unlock(&data_rw);
  safe_cond_signal(&data_ready, &data_mutex);
  //full_demod(fm);
}


static void *demod_thread_fn(void *arg)
{
  struct fm_state *fm2 = (struct fm_state *)arg;
  // So that DBs will be loaded, we'd better use a mutex here
  pthread_mutex_lock(&dataset_mutex);
  pthread_mutex_unlock(&dataset_mutex);

  while (!do_exit) {
    safe_cond_wait(&data_ready, &data_mutex);
    full_demod(fm2);
    acars_decode(fm2);
    if (fm2->exit_flag) {
      do_exit = 1;
      //rtlsdr_cancel_async(dev);
    }
  }
  return 0;
}


double atofs(char *f)
/* standard suffixes */
{
  char last;
  int len;
  double suff = 1.0;
  len = strlen(f);
  last = f[len-1];
  f[len-1] = '\0';
  switch (last) {
  case 'g':
  case 'G':
    suff *= 1e3;
  case 'm':
  case 'M':
    suff *= 1e3;
  case 'k':
  case 'K':
    suff *= 1e3;
    suff *= atof(f);
    f[len-1] = last;
    return suff;
  }
  f[len-1] = last;
  return atof(f);
}


void frequency_range(struct fm_state *fm, char *arg)
{
  char *start, *stop, *step;
  int i;
  start = arg;
  stop = strchr(start, ':') + 1;
  stop[-1] = '\0';
  step = strchr(stop, ':') + 1;
  step[-1] = '\0';
  for(i=(int)atofs(start); i<=(int)atofs(stop); i+=(int)atofs(step))
    {
      fm->freqs[fm->freq_len] = (uint32_t)i;
      fm->freq_len++;
      if (fm->freq_len >= FREQUENCIES_LIMIT) {
	break;}
    }
  stop[-1] = ':';
  step[-1] = ':';
}


int nearest_gain(int target_gain)
{
  int err1, err2, count, close_gain;

  count = rtlsdr_get_tuner_gains(dev, NULL);
  if (count <= 0)
    return 0;

  Buffer<int> gains( count );
  
  count = rtlsdr_get_tuner_gains(dev, gains.get());
  close_gain = gains[0];
  for( int i = 0; i < count; ++i ) {
    err1 = abs(target_gain - close_gain);
    err2 = abs(target_gain - gains[i]);
    if(err2 < err1) 
      close_gain = gains[i];
  }
  gains.check();
  
  return close_gain;
}


void fm_init(struct fm_state *fm)
{
  fm->freqs[0] = 100000000;
  fm->sample_rate = Fe;
  fm->squelch_level = 0;
  fm->conseq_squelch = 0;
  fm->terminate_on_squelch = 0;
  fm->squelch_hits = 0;
  fm->freq_len = 0;
  fm->edge = 0;
  fm->fir_enable = 0;
  fm->prev_index = 0;
  fm->post_downsample = 1;  // once this works, default = 4
  fm->custom_atan = 0;
  fm->deemph = 0;
  fm->output_rate = -1;  // flag for disabled
  fm->pre_j = fm->pre_r = fm->now_r = fm->now_j = 0;
  fm->prev_lpr_index = 0;
  fm->deemph_a = 0;
  fm->now_lpr = 0;
  fm->dc_block = 0;
  fm->dc_avg = 0;

}


int
main( int argc, char** argv ) {

#ifndef _WIN32
  struct sigaction sigact;
#endif
  struct fm_state fm; 
  char *filename = NULL;
  int r, opt, wb_mode = 0;
  int gain = AUTO_GAIN; // tenths of a dB
  uint32_t dev_index = 0;
  int device_count;
  int ppm_error = 0;

  Buffer<uint8_t> buffer;

  fm_init(&fm);

  // Compute the number of threads for OpenMP with the minimum value
  // of two (sorry one and two core CPUs).

  n_omp = (( omp_get_max_threads() * 5 ) / 8 );
  n_omp = ( std::max( n_omp, 2 ));
    
  pthread_cond_init(&data_ready, NULL);
  pthread_rwlock_init(&data_rw, NULL);
  pthread_mutex_init(&data_mutex, NULL);
  pthread_mutex_init(&dataset_mutex, NULL);

  fm.sample_rate = uint32_t(Fe);

  while ((opt = getopt(argc, argv, "d:f:g:l:o:t:p:Frhv")) != -1) {
    switch (opt) {
    case 'd':
      dev_index = atoi(optarg);
      break;
    case 'f':
      if (fm.freq_len >= FREQUENCIES_LIMIT) 
	break;
      if (strchr(optarg, ':'))
	frequency_range(&fm, optarg);
      else {
	  fm.freqs[fm.freq_len] = (uint32_t)atofs(optarg);
	  fm.freq_len++;
      }
      break;
    case 'g':
      gain = (int)(atof(optarg) * 10);
      break;
    case 'l':
      fm.squelch_level = (int)atof(optarg);
      break;
    case 'o':
      fm.post_downsample = (int)atof(optarg);
      if (fm.post_downsample < 1 || fm.post_downsample > MAXIMUM_OVERSAMPLE) {
	fprintf(stderr, "Oversample must be between 1 and %i\n", MAXIMUM_OVERSAMPLE);}
      break;
    case 't':
      fm.conseq_squelch = (int)atof(optarg);
      if (fm.conseq_squelch < 0) {
	fm.conseq_squelch = -fm.conseq_squelch;
	fm.terminate_on_squelch = 1;
      }
      break;
    case 'r':
      debug_hop = 1;
      break;
    case 'p':
      ppm_error = atoi(optarg);
      break;
    case 'F':
      fm.fir_enable = 1;
      break;
    case 'v':
      ++verbose;
      break;
    case 'h':
    default:
      usage();
      break;
    }
  }

  /* quadruple sample_rate to limit to Δθ to ±π/2 */
  fm.sample_rate *= fm.post_downsample;

  if (fm.freq_len == 0) {
    fprintf(stderr, "Please specify a frequency.\n");
    exit(1);
  }

  if (fm.freq_len >= FREQUENCIES_LIMIT) {
    fprintf(stderr, "Too many channels, maximum %i.\n", FREQUENCIES_LIMIT);
    exit(1);
  }

  if (fm.freq_len > 1 && fm.squelch_level == 0) {
    fprintf(stderr, "Please specify a squelch level.  Required for scanning multiple frequencies.\n");
    exit(1);
  }

  if (fm.freq_len > 1) 
    fm.terminate_on_squelch = 0;

  if (argc <= optind) 
    filename = "-";
  else 
    filename = argv[optind];

  ACTUAL_BUF_LENGTH = lcm_post[fm.post_downsample] * DEFAULT_BUF_LENGTH;

  buffer.set( ACTUAL_BUF_LENGTH );
  
  device_count = rtlsdr_get_device_count();
  if (!device_count) {
    fprintf(stderr, "No supported devices found.\n");
    exit(1);
  }

  fprintf(stderr, "Found %d device(s):\n", device_count);
  for( int i = 0; i < device_count; ++i ) {

    Buffer<char> vendor(256), product(256), serial(256);
    
    rtlsdr_get_device_usb_strings( i,
				   vendor.get(),
				   product.get(),
				   serial.get());
    fprintf( stderr,
	     "  %d:  %s, %s, SN: %s\n", i,
	     vendor.get(), product.get(), serial.get());

    vendor.check();
    product.check();
    serial.check();
    
  }
  fprintf(stderr, "Using device %d: %s\n",
	  dev_index, rtlsdr_get_device_name(dev_index));

  r = rtlsdr_open(&dev, dev_index);
  if (r < 0) {
    fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
    exit(1);
  }
#ifndef _WIN32
  sigact.sa_handler = sighandler;
  sigemptyset(&sigact.sa_mask);
  sigact.sa_flags = 0;
  sigaction(SIGINT, &sigact, NULL);
  sigaction(SIGTERM, &sigact, NULL);
  sigaction(SIGQUIT, &sigact, NULL);
  sigaction(SIGPIPE, &sigact, NULL);
#else
  SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif

  /* WBFM is special */
  // I really should loop over everything
  // but you are more wrong for scanning broadcast FM

  if (wb_mode) 
    fm.freqs[0] += 16000;

  if (fm.deemph) 
    fm.deemph_a = (int)round(1.0/((1.0-exp(-1.0/(fm.output_rate * 75e-6)))));

  optimal_settings(&fm, 0, 0);
  build_fir(&fm);

  /* Set the tuner gain */
  if (gain == AUTO_GAIN) {
    r = rtlsdr_set_tuner_gain_mode(dev, 0);
  } else {
    r = rtlsdr_set_tuner_gain_mode(dev, 1);
    gain = nearest_gain(gain);
    r = rtlsdr_set_tuner_gain(dev, gain);
  }
  if (r != 0) 
    fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
  else
    if (gain == AUTO_GAIN) 
      fprintf(stderr, "Tuner gain set to automatic.\n");
    else 
      fprintf(stderr, "Tuner gain set to %0.2f dB.\n", gain/10.0);
  r = rtlsdr_set_freq_correction(dev, ppm_error);

  if (strcmp(filename, "-") == 0) { /* Write samples to stdout */
    fm.file = stdout;
#ifdef _WIN32
    //_setmode(_fileno(fm.file), _O_BINARY);
#endif
  } else {
    /* DO NOT WRITE TO FILE 
		
       fm.file = fopen(filename, "wb");
       if (!fm.file) {
       fprintf(stderr, "Failed to open %s\n", filename);
       exit(1);
       }
    */
  }

  /* Reset endpoint before we start reading from it (mandatory) */
  r = rtlsdr_reset_buffer(dev);
  if (r < 0) {
    fprintf(stderr, "WARNING: Failed to reset buffers.\n");}
  pthread_mutex_lock(&dataset_mutex);
  pthread_create(&demod_thread, NULL, demod_thread_fn, (void *)(&fm));
  /*rtlsdr_read_async(dev, rtlsdr_callback, (void *)(&fm),
    DEFAULT_ASYNC_BUF_NUMBER,
    ACTUAL_BUF_LENGTH);*/
  fprintf(stderr, "\n");
  load_aircrafts();
  load_airports();
  load_flights();
  load_message_labels();
  
  init_bits();
  _reset_bit_state_machine();
  _reset_message_state_machine();
  
  printf("Listening for ACARS traffic...\n");
  fprintf(stderr, "\n");
  pthread_mutex_unlock(&dataset_mutex);

  while (!do_exit) {

    buffer.check();

    sync_read( buffer.get(), ACTUAL_BUF_LENGTH, &fm);

  }
  if (do_exit) {
    fprintf(stderr, "\nUser cancel, exiting...\n");}
  else
    fprintf(stderr, "\nLibrary error %d, exiting...\n", r);
  
  //rtlsdr_cancel_async(dev);
  safe_cond_signal(&data_ready, &data_mutex);
  pthread_join(demod_thread, NULL);

  pthread_cond_destroy(&data_ready);
  pthread_rwlock_destroy(&data_rw);
  pthread_mutex_destroy(&data_mutex);

  /*
    if (fm.file != stdout) {
    fclose(fm.file);}
  */
  rtlsdr_close(dev);

  return r >= 0 ? r : -r;
}


// vim: tabstop=8:softtabstop=8:shiftwidth=8:noexpandtab

/*  LocalWords:  OpenMP PRE SOH ETX ETB CRC GNURadio GCC ACARS README
 */
/*  LocalWords:  SDR CPUs
 */
