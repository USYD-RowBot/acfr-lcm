#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "perls-common/ascii.h"
#include "perls-common/nmea.h"

#include "os5000.h"

#define DEBUG 0

// Format Type 0x01: "$C" format
os5000_t
os5000_parse_0x01 (const char buf[], size_t len)
{
    os5000_t os = {0};

    if (!nmea_validate_checksum (buf))
        return os;

    char *ptr;
    if ((ptr=strchr (buf, 'C')) && sscanf (ptr, "C%lf", &os.H))
        os.bitmask |= OS5000_BITMASK_H;
    
    if ((ptr=strchr (buf, 'P')) && sscanf (ptr, "P%lf", &os.P))
        os.bitmask |= OS5000_BITMASK_P;
    
    if ((ptr=strchr (buf, 'R')) && sscanf (ptr, "R%lf", &os.R))
            os.bitmask |= OS5000_BITMASK_R;
    
    if ((ptr=strchr (buf, 'T')) && sscanf (ptr, "T%lf", &os.T))
        os.bitmask |= OS5000_BITMASK_T;
    
    if ((ptr=strchr (buf, 'D')) && sscanf (ptr, "D%lf", &os.D))
        os.bitmask |= OS5000_BITMASK_D;
    
    if ((ptr=strchr (buf, 'M')) && sscanf (ptr, "M%lf", &os.M))
        os.bitmask |= OS5000_BITMASK_M;
    
    if ((ptr=strstr (buf, "Mx")) && sscanf (ptr, "Mx%lf", &os.Mxyz[0]) &&
        (ptr=strstr (buf, "My")) && sscanf (ptr, "My%lf", &os.Mxyz[1]) &&
        (ptr=strstr (buf, "Mz")) && sscanf (ptr, "Mz%lf", &os.Mxyz[2]))
        os.bitmask |= OS5000_BITMASK_M_XYZ;
    
    if ((ptr=strchr (buf, 'A')) && sscanf (ptr, "A%lf", &os.A))
        os.bitmask |= OS5000_BITMASK_A;
    
    if ((ptr=strstr (buf, "Ax")) && sscanf (ptr, "Ax%lf", &os.Axyz[0]) &&
        (ptr=strstr (buf, "Ay")) && sscanf (ptr, "Ay%lf", &os.Axyz[1]) &&
        (ptr=strstr (buf, "Az")) && sscanf (ptr, "Az%lf", &os.Axyz[2]))
        os.bitmask |= OS5000_BITMASK_A_XYZ;


#if DEBUG
#include "perls-common/stdio_util.h"
    printf ("os: H=%f P=%f R=%f T=%f D=%f M=%f Mx=%f My=%f Mz=%f A=%f Ax=%f Ay=%f Az=%f\n", 
            os.H, os.P, os.R, os.T, os.D, 
            os.M, os.Mxyz[0], os.Mxyz[1], os.Mxyz[2], 
            os.A, os.Axyz[0], os.Axyz[1], os.Axyz[2]);
    printf ("hex="); stdiou_hexdump (&os.bitmask, sizeof (os.bitmask), NULL); printf ("\n");
    printf ("bin="); stdiou_bindump (&os.bitmask, sizeof (os.bitmask), NULL); printf ("\n");
#endif

    return os;
}
