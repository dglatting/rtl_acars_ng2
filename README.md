rtl_acars_ng2
============

A variation of the: Simple console rtl_sdr ACARS decoder

Dennis Glatting, July 2016

------------------------------------------------------------------------------

This is an update to rtl_acars_ng found elsewhere on github. This code
IS NOT meant as a replacement of rtl_acars_ng rather an it is an
augmentation of sorts. Specifically, I ported two key sections of
rtl_acars_ng (getbit() and getmesg()) to GNURadio which is a radically
different environment than the environment rtl_acars_ng was
intended. I ported the GNURadio code back to rtl_acars_ng as a
test/comparison vehicle.

I already had ACARS code in GNURadio but I liked rtl_acars_ng because
of its simplicity, so I replaced what I had under GNURadio with
rtl_acars_ng. During that work I ported back into rtl_ACARS_ng what I
had ported/crafted from GNURadio. There were several consequences and
I made other changes.

First, this is C++ code; specifically C++11. GNURadio is a C++
environment and I am a fan of C++11 and so that is the genesis of C++
into this code. There were several changes:

 * In many places in rtl_acars_ng there is a mix of "unsigned char"
   (now, in places, uint8_t -- as it should be) and char. Rather than
   go through the code and cast every use case I simply created inline
   functions to do the casting for me. This had the benefit of
   indicating where there may be use problems.

 * I replaced C enums (int) as "class enum" thereby making enum use
   more strict.

 * I used C++ standard library classes where I modified code and to
   reduce possible memory release problems; not that there were
   any. At least, now, those allocations/deallocations are automatic.

 * Tighter parameter passing.

I also const'ed things. The C++ versions I used where G++ 5.3.1 and
6.1.0 under Ubuntu 16.04. I installed GCC 6.1.0 from source off the
GCC git.

I plan to replace other code and data structures with C++ objects but
my goal is to test my ported/crafted code by replacing existing code
rather that generate a whole new wonder working thing.

Second, I eliminated unused variables and unused command line options
from the code. I also converted some memory allocations to my Buffer
classes and I scoped variables.

Third, some of my objects under GNURadio use VOLK, FFTW, and
OpenMP. To compile and link this code those libraries MUST be
installed. Frankly, I am too lazy to remove the FFTW dependency and in
a few places I do use VOLK functions.

Fourth, I replaced the CRC routines with the ones I developed under
GNURadio, which changes the model in rtl_acars_ng from the application
of a polynomial by shifts, adds, and logical operations to one that is
table driven. This may or may not be faster. I haven't bothered to
compare CPU usage.

That said, one and two bit error corrections was a bit hairy and that
code is now significantly simplified. However, for now I dumped two
bit corrections NOT because it cannot be useful but in practical terms
it WAS NOT useful. Under two bit error conditions the data was
typically seriously corrupted such that corrections didn't have
practical value. Further, looking at the data composing these failure
conditions I am convinced some messages can be algorithmically
recovered by other means however I am too lazy to implement something
at this time.

Also, it was somewhat amusing to watching the CPU load graph under two
bit corrections. In other code I have written (e.g., my ADS-B GNURadio
code), I utilized computed syndromes to correct bit errors however the
trade off was significantly increased data usage. I am unconvinced
anything greater than one bit error correction attempts is useful for
ACARS because the CRC polynomial isn't one known to have a great
Hamming distance. If you know different, please write me.

See:

 * Cyclic Redundency Code (CRC) Polynomial Selection For Embedded
   Networks.

   Koopman, Philip and Chakravarty, Tridib 
   http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf

 * Investigating CRC Polynomials that Correct Burst Errors

   Travis Mandel, Jens Mache
   http://homes.cs.washington.edu/~tmandel/papers/CRCBurst.pdf

 *  An Algorithm for Error Correcting Cyclic Redundance Checks

    McDaniel, Bill
    http://www.drdobbs.com/an-algorithm-for-error-correcting-cyclic/184401662
	    
Fifth, I added a verbosity command line option.

Finally, the code size increased. Some of the increase is additional
printf() and std::cout statements; some debug related (e.g., assert()
statements); and in other places I added const data structures.


Regarding working code under Windows, I could care less. I do not do
windows and I do not care.


Final notes.

* I considered removing the DC filter because there is no way to
  invoke the code. It appears the intention was to have a command line
  option but there wasn't any.

* I have seen the pre-modified and post modified code lock up. This
  seems to be SDR related. Ctl-C doesn't cause the application to exit
  and in those cases by disconnecting the SDR (i.e., pulling the USB
  cable) order was restored.

* Part of my debugging process included graphing registers, such as
  csample. Although I printed the graphs I did not save them,
  otherwise I would have included them here. They were interesting to
  look at but I am an old newbie and also working on a communications
  degree.

  Within my GNURadio code I have a debug output of type gr_complex
  where I can simultaneously graph two run-time data sets, such as
  hsample and lsample. Those too are interesting to look at.

* This code was tested against my transmitter written under GNURadio
  and NOT live capture. That said, this code and the origional code
  was happy with my transmitted signal.


Dennis Glatting
July 2016


LocalWords:  GNURadio VOLK FFTW CRC printf const ACARS rtl acars ng
LocalWords:  github SDR
