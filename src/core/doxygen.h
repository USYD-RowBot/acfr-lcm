/**
 * @file dummy.c
 * @brief Contains the content for miscellaneous documentation pages
 */

/**
 * @mainpage
 * 
 * @image html perl-logo-gray.png University of Michigan
 *
 */

/**
 * @page doxTut Doxygen Tutorial
 Under construction.  Look at what's already written for guidance.
*/

/**
 * @page codingStyle Programming Style 

 This page is our lab's official programming style guidelines.

 <HR>
 See Dr. Kieras' page on coding style for EECS 381.  His coding standards are similar to ours.

 <a href="http://www.engin.umich.edu/class/eecs381/handouts/C_Coding_Standards.pdf">
 C Coding Standards</a>

 <a href="http://www.engin.umich.edu/class/eecs381/handouts/C++_Coding_Standards.pdf">
 C++ Coding Standards</a>

 <HR>
 PeRL-specific standards:
<ul> 

  <li>
  Document (using doxygen comments) all header files and files
  containing the 'int main()'
  </li>
   
  <li>
  CamelCase or using_underscores will be left as personal preference
  when writing <i>standalone</i> exe's (e.g., heartbeat);
  however, when writing <i>library</i> functions stick to
  underscore_notation (e.g., perls-common).
  </li>

  <li>
  Appending '_c' to string constants is not necessary.
  </li>

  <li>
  Functions returning success or failure will use the
  <code>EXIT_SUCCESS</code> and <code>EXIT_FAILURE</code> macros, respectively.
  </li>

  <li>
  One-letter variable names should follow Dr. Kieras' guidelines.
  However, when implementing a filter that's very mathematical,
  one-letter variables names are OK only if they're
 documented with doxygen!
  </li>


  <li>
  Formatting and indentation style <b>ARE NOT</b> personal
  preference. Please adhere to the included PeRL formatting and
  indentation example.

 - Indentation is 4 white spaces per level, <b>DO NOT</b> use tabs in
   your editor!
   <p></p>

 - When calling a function, include a single white space between the function name and its args:
   \verbatim printf ("myvar len=%d\n", sizeof (myvar));\endverbatim
   <p></p>

 - Function return types <b>ALWAYS</b> go on the line preceeding the
   function signature (see <a
   href="http://en.wikipedia.org/wiki/Indent_style#GNU_style"> The GNU
   Indent style</a>); function body <b>ALWAYS</b> goes on the line proceeding the
   function signature:<br>
   <code>\verbatim
   int
   main (int a, int b)
   {
       return a > b;
   }
   \endverbatim</code>
   <p></p>

 - Pointer variables should be declared such that the placement of '<code>*</code>' makes their "dereferenced" value clear.  Examples:<br>
   \verbatim
   Bad:  char * mystring; double * myvar;
   Bad:  char* myvar; double* myvar;
   Good: char *mystring; double *myvar;
   \endverbatim
  </li>
   <p></p>

 - Curly bracket placement in <code>if/else</code> switchyards, <code>while</code> loops, and <code>switch/case</code> statements should
   be as follows (<code>ML</code> indicates multiline code entry,
   <code>SL</code> indicates single line code entry):<br>
   <code>\verbatim
   if (foo) {
       ML;
   }
   else if (bar) {
       ML;
   }
   else
       SL;
   \endverbatim</code>

</ul>


 <HR>
 PeRL format/style example:

 <code>\verbatim
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <glib.h>
#include <lcm/lcm.h>

#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/glib_util.h"
#include "perls-common/timestamp.h"

typedef struct value value_t;
struct value {
    int64_t utime;
    int64_t jitter;
};

static void 
message_handler (const lcm_recv_buf_t *rbuf, const char *channel, void *user)
{
    GHashTable *hash = user;

    value_t *ov = g_hash_table_lookup (hash, channel);

    int64_t utime = timestamp_now ();
    int64_t utime_last = ov ? ov->utime : 0;
    value_t nv = {
        .utime = utime,
        .jitter = utime - utime_last,
    };
    g_hash_table_insert (hash, strdup (channel), gu_dup (&nv, sizeof nv));

    printf ("jitter=%-16"PRId64"  Hz=%-12g  %s\n", nv.jitter, 1./nv.jitter*1E6, channel);
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "A generic LCM sink for testing channel traffic");
    getopt_add_help (gopt, NULL);
    getopt_add_string (gopt, 'c', "channels", ".*", "Comma seperated list of LCM regex channel names to monitor");
    getopt_add_example (gopt,
                        "Monitor EASYDAQ and all HEARTBEAT channels\n"
                        "%s --channels EASYDAQ,HEARTBEAT_.*HZ", argv[0]);

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=0) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    lcm_t *lcm = lcm_create (NULL);
    if (!lcm) {
        ERROR ("lcm_create()");
        exit (EXIT_FAILURE);
    }

    GHashTable *hash = g_hash_table_new_full (&g_str_hash, &g_str_equal, &g_free, &g_free);
    char **channels = g_strsplit (getopt_get_string (gopt, "channels"), ",", -1);
    for (int i=0; channels[i]!=NULL; i++) {
        const char *channel = channels[i];
        printf ("Adding channel %s\n", channel);
        lcm_subscribe (lcm, channel, message_handler, hash);
    }

    while (1)
        lcm_handle (lcm);
}
 \endverbatim</code>
 
*/

/**
 * @page surfgpuInstall Speeded Up SURF Installation Guide

 This page describes how to install the <a
 href="http://asrl.utias.utoronto.ca/code/gpusurf"> Speeded Up SURF</a> library needed to
 build the surfgpu server and client.  Throughout this guide, I will use "gpusurf" and
 "Speeded Up SURF" interchangabley to refer to the same thing.

 <HR>

 <b>Known Issues with surfgpu server:</b>
 <ul>

 <li> 
 Speeed Up SURF only works on some nvidia GPUs.  As of writing this page, the
 <b>Quadro FX 3800M</b> card appears to work acceptably well.  The GeForce GTX 460 
 does not work.
 </li>

 <li> 
 As of now, the surfgpu server will fail to process images after 3 or so client
 requests, where the 3 requests are of <b>large</b> images, each with <b>different
 resolutions</b>.  The server must be restarted if this happens!

 If surfgpu processes images that are all the same size (like PROSILICA channel images),
 it should be fine.
 </li>

 <li>
 OpenCV 2.1 is required to compile gpusurf.
 </li>

 </ul>

 <HR>

 <b>Steps to compile gpusurf:</b>

 <ol>

 <li>
 Make sure you have the latest nvidia drivers and CUDA toolkit installed. If you don't...

 <ol>


 <li> Download the nvidia drivers from <a href="www.nvidia.com/page/drivers">NVIDIA</a>
 and put them somewhere you'll remember.  Also download the CUDA Toolkit 4.0 or above
 </li>

 <li>
 Remove the official ubuntu nvidia drivers with
 <code>
 \verbatim
 $ sudo apt-get autoremove "nvidia-*"  \endverbatim
 </code>
 </li>

 <li>
 Stop X
 <code>
 \verbatim
 $ sudo service gdm stop \endverbatim
 </code>
 </li>

 <li>
 cd to the directory containing the NVIDIA drivers
 <code>
 \verbatim
 $ sudo sh <name of nvidia driver file> \endverbatim
 </code>
 and go through the motions (say yes to everything)
 </li>

 <li>
 cd to the directory containing the NVIDIA CUDA toolkit
 <code>
 \verbatim
 $ sudo sh <name of cuda toolkit file> \endverbatim
 </code>
 and go through the motions (install to /usr/local/cuda)
 </li>

 <li>
 Edit /etc/ld.so.conf as root and make sure it has the following two lines
 <code>
 \verbatim
 /usr/local/cuda/lib
 /usr/local/cuda/lib64\endverbatim
 </code>
 Then run
 <code>
 \verbatim
 sudo ldconfig\endverbatim
 </code>
 </li>

 <li>
 Edit /etc/modprobe.d/blacklist.conf as root and add the following line to the end:
 <code>
 \verbatim
 blacklist nouveau \endverbatim
 </code>
 </li>

 <li>
 <b>Now</b> restart the computer
 </li>
 </ol>
 </li>

 <li>
 Make sure you have GCC 4.4.  As of writing this page, GCC 4.6 doesn't work for compiling
 cuda stuff.  If you don't have GCC 4.4, google how to downgrade to 4.4.
 </li>

 <li>
 Install OpenCV 2.1 if you don't have it.  Just add the ppa ppa:gijzelaar/opencv2 and
 install the libopencv-dev package
 </li>

 <li>
 Download gpusurf from their website, and extract the tarball.
 </li>

 <li>
 In the source tree, cd to the 3rdParty directory
 <ol>
 <li>
 If running 64-bit linux, Edit the common.mk file as shown 
 <a href="http://asrl.utias.utoronto.ca/code/gpusurf/page-build-linux.html">
 here</a>
 <code>
 \verbatim
 L92  CXXFLAGS  := $(CXXWARN_FLAGS) -fPIC
 L93  CFLAGS    := $(CWARN_FLAGS) -fPIC
 L118 NVCCFLAGS += $(SMVERSIONFLAGS) --compiler-options -fPIC -shared \endverbatim
 </code>
 </li>

 <li>
 Add /usr/local/cuda/bin to PATH
 <code>
 \verbatim
 export PATH=/usr/local/cuda/bin:$PATH \endverbatim
 </code>
 </li>

 <li>
 cd to gpusurf-x.x.x/3rdParty/cudpp/cudpp and run make
 </li>

 </ol>
 </li>

 <li>
  To be cuda 4.0 compatible, open <b>gpusurf-x.x.x/src/GpuIntegralImageProcessor.hpp</b>
  include <driver_types.h> after <boost/shared_ptr.hpp>
  <code>
  \verbatim
  #include <boost/shared_ptr.hpp>
  #include <driver_types.h> \endverbatim
  </code>
  also, comments the following two lines
  <code>
  \verbatim
  //struct cudaArray;
  typedef size_t CUDPPHandle;
  //typedef int cudaStream_t; \endverbatim
  </code>
 </li>

 <li>
 Compile gpusurf with 'ccmake ..' and 'make' from the build/ directory
 </li>

 <li>

 <b>There is no 'make install'</b> option.  Run the following commands to install the
 library in /usr/local

 <code>
 \verbatim
 sudo cp gpusurf-x.x.x/build/lib/libgpusurf.so /usr/local/lib
 sudo cp -r gpusurf-x.x.x/include/gpusurf /usr/local/include
 sudo ldconfig \endverbatim
 </code>
 </li>

 <li>
 You should now be able to compile surfgpu in perls.
 </li>

 </ol>


*/

