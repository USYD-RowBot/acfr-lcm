//System includes
#include <iostream>
#include <sys/socket.h>
#include <signal.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/shm.h>

//OpenCV includes
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"

//PeRLs includes
#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"
#include "perls-common/timestamp.h"
#include "perls-common/units.h"

#include "perls-vision/botimage.h"
#include "perls-vision/feature.h"
#include "perls-vision/featuregpu.h"

#include "perls-lcmtypes/perllcm_van_cvsurf_params_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_cvsurf_t.h"
#include "perls-lcmtypes/perllcm_van_feature_t.h"

//lcm
#include <lcm/lcm.h>

//Misc includes
#include <gsl/gsl_math.h>

//For STL
using namespace std; 

#define SURFGPU_MAX_OPTIONS  64
#define SURFGPU_MAX_PENDING  100 /* Maximum outstanding connection requests */
#define SURFGPU_MAX_FEATURES 10000

#define SURFGPU_CLIENT_TCP  1
#define SURFGPU_CLIENT_SHM  2

#define DTOR (UNITS_DEGREE_TO_RADIAN);

#define DESCRIPTION                                                     \
    "SurfGPU server encapsulates the C++ GpuSurfDetector object code into a standalone process.\n" \
    "Client processes interact with surfgpu-server via a TCP/SHM API."

typedef struct surfgpu surfgpu_t;
struct surfgpu {
    cv::gpu::SURF_GPU *surf;
    cv::gpu::GpuMat *keypointsGpu;
    cv::gpu::GpuMat *descriptorsGpu;

    lcm_t *lcm;

    int servSock;                /* Socket descriptor for server */
    int clntSock;                /* Socket descriptor for client */
    struct sockaddr_in servAddr; /* Local address */
    struct sockaddr_in clntAddr; /* Client address */
    uint16_t servPort;           /* Server port */

    int32_t  bufsize;
    uint8_t *buf;

    int32_t  shmid;
    int32_t  shmsize;
    uint8_t *shm;
    
    int n_max_features;
};


static bool
featureSort (pair<float,int> const & lhs, pair<float,int> const & rhs)
{
    return lhs.first > rhs.first;
}

static void
sortFeatures (vector<cv::KeyPoint> &keypoints, vector<float> &descriptors, 
              int descriptorSize, uint16_t maxNumFeats)
{
    //Get a pair of <feature index, response> we'll use to sort the features
    vector<pair<float,int> > strengthIndex (keypoints.size ());
    for (uint16_t k = 0; k < keypoints.size (); k++) {
        strengthIndex[k].first = keypoints[k].response;
        strengthIndex[k].second = k;
    }

    if(maxNumFeats > 0 && (int)keypoints.size () > maxNumFeats) {

        partial_sort (strengthIndex.begin (), 
                      strengthIndex.begin () + maxNumFeats, 
                      strengthIndex.end (), 
                      &featureSort);

        strengthIndex.resize (maxNumFeats);
    }
    else
        return;

    vector<cv::KeyPoint> keypointsCopy (maxNumFeats);
    vector<float> descriptorsCopy (maxNumFeats*descriptorSize);

    for (uint16_t i=0; i<maxNumFeats; i++) {
    
        keypointsCopy[i] = keypoints[strengthIndex[i].second];
    
        vector<float>::const_iterator d = descriptors.begin() + (strengthIndex[i].second * descriptorSize);
        for (uint16_t j=0; j<descriptorSize; j++)
            descriptorsCopy[j] = *d;
    }

    keypoints = keypointsCopy;
    descriptors = descriptorsCopy;
}

static void
runSurf (surfgpu_t *surfgpu, bot_core_image_t *bot)
{
    int64_t start_time = timestamp_now ();
    IplImage *img = vis_botimage_to_iplimage_copy (bot);
    cv::Mat imageMat (img);
    cv::gpu::GpuMat imageMatGpu (imageMat);

    (*surfgpu->surf)(imageMatGpu, cv::gpu::GpuMat(), *surfgpu->keypointsGpu, *surfgpu->descriptorsGpu);

    int64_t end_time = timestamp_now ();
    printf ("SurfGPU:\tTook %.4f s to process\n", timestamp_to_double (end_time - start_time));
    cvReleaseImage (&img);
    imageMat.release ();
}

static perllcm_van_feature_t*
OcvDescriptorToPerllcmVanFeature (vector<cv::KeyPoint>& keypoints, vector<float>& descriptors, 
                                  int32_t keylen, int max_npts)
{
    int npts = keypoints.size ();

    //Create the feature_t
    perllcm_van_feature_t *f = (perllcm_van_feature_t *) calloc (1, sizeof (*f));
    f->npts = GSL_MIN (npts, max_npts);
    f->u = (float *) malloc (f->npts * sizeof (*f->u));
    f->v = (float *) malloc (f->npts * sizeof (*f->v));
    f->keylen = keylen;
    f->keys = (float **) malloc (f->npts * sizeof (*f->keys));

    //The surf feature uses opencv's surf type
    perllcm_van_feature_attr_cvsurf_t *attr = (perllcm_van_feature_attr_cvsurf_t *) calloc (1, sizeof (*attr));
    attr->npts = f->npts;
    attr->laplacian = (int8_t *) malloc (f->npts * sizeof (*attr->laplacian));
    attr->size = (float *) malloc (f->npts * sizeof (*attr->size));
    attr->dir  = (float *) malloc (f->npts * sizeof (*attr->dir));
    attr->hessian = (float *) malloc (f->npts * sizeof (*attr->hessian));

    //Populate the points and descriptors in f
    for (int n=0; n < f->npts; n++) {
        f->u[n] = keypoints[n].pt.x;
        f->v[n] = keypoints[n].pt.y;
        f->keys[n] = (float *) malloc (f->keylen * sizeof (*f->keys[n]));

        //Copy over the descriptor.  What a PITA
        for (int m=0; m < f->keylen; m++)
            f->keys[n][m] = descriptors[f->keylen * n + m];

        //attr->laplacian[n] = isLastBitSet(keypoints[n].response);
        attr->laplacian[n] = 0;

        //Scale and orientation (in radians)
        attr->size[n] = keypoints[n].size/2;
        attr->dir[n] = keypoints[n].angle * DTOR;
        attr->hessian[n] = -1; //Not used?
    }

    int32_t max_attr_size = perllcm_van_feature_attr_cvsurf_t_encoded_size (attr);
    f->attrtype = PERLLCM_VAN_FEATURE_T_ATTRTYPE_SURFGPU;
    f->attr = (uint8_t *) malloc (max_attr_size);
    f->attrsize = perllcm_van_feature_attr_cvsurf_t_encode (f->attr, 0, max_attr_size, attr);

    /* test */
    perllcm_van_feature_attr_cvsurf_t *attr2 = (perllcm_van_feature_attr_cvsurf_t *) malloc (sizeof (*attr2));
    if (perllcm_van_feature_attr_cvsurf_t_decode (f->attr, 0, f->attrsize, attr2) != f->attrsize) {
        ERROR ("perllcm_van_feature_attr_cvsurf_t_decode() failed");
        free (attr);
        return NULL;
    }

    // assign null user to feature_t
    f->usertype = PERLLCM_VAN_FEATURE_T_USERTYPE_NONE;
    f->usersize = 0;
    f->user = NULL;

    //clean up
    perllcm_van_feature_attr_cvsurf_t_destroy (attr);

    return f;
}

/**
 * Download the keypoint and descriptor from the GPU, and convert it to
 * perllcm_van_feature_t
 */
static perllcm_van_feature_t *
surfgpu_to_feature_t (surfgpu_t *surfgpu, int max_npts)
{
    //Download the keypoints from the GPU
    vector<cv::KeyPoint> keypoints;
    surfgpu->surf->downloadKeypoints (*surfgpu->keypointsGpu, keypoints);
  
    //Download the descriptors from the GPU
    vector<float> descriptors;
    surfgpu->surf->downloadDescriptors (*surfgpu->descriptorsGpu, descriptors);

    //We need to sort the keypoints and descriptors by strength
    sortFeatures(keypoints, descriptors, surfgpu->surf->descriptorSize (), SURFGPU_MAX_FEATURES);
    printf("%d\n", (int)keypoints.size ());
    int npts = keypoints.size ();

    printf ("Number of features : %d\n", npts);

    return OcvDescriptorToPerllcmVanFeature (keypoints, descriptors, 
                                             surfgpu->surf->descriptorSize (), max_npts);
}

static int
CreateTCPServerSocket (surfgpu_t *surfgpu)
{
    /* Create socket for incoming connections */
    if ((surfgpu->servSock = socket (PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        PERROR ("socket() failed");
        return -1;
    }
    int opt = 1;
    if (setsockopt (surfgpu->servSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof (opt)) < 0) {
        PERROR ("socket() failed");
        return -1;
    }

    /* Construct local address structure */
    memset (&surfgpu->servAddr, 0, sizeof (surfgpu->servAddr)); /* Zero out structure */
    surfgpu->servAddr.sin_family = AF_INET;                     /* Internet address family */
    surfgpu->servAddr.sin_addr.s_addr = htonl (INADDR_ANY);     /* Any incoming interface */
    surfgpu->servAddr.sin_port = htons (surfgpu->servPort);     /* Local port */

    /* Bind to the local address */
    if (bind (surfgpu->servSock, (struct sockaddr *) &surfgpu->servAddr, sizeof (surfgpu->servAddr)) < 0) {
        PERROR ("bind() failed");
        return -1;
    }

    /* Mark the socket so it will listen for incoming connections */
    if (listen (surfgpu->servSock, SURFGPU_MAX_PENDING) < 0) {
        PERROR ("listen() failed");
        return -1;
    }

    return 0; 
}

static int
AcceptTCPConnection (surfgpu_t *surfgpu)
{
    /* Wait for client to connect */
    uint32_t clntLen = sizeof (surfgpu->clntAddr);
    if ((surfgpu->clntSock = accept (surfgpu->servSock, (struct sockaddr *) &surfgpu->clntAddr, &clntLen)) < 0) {
        PERROR ("accept() failed");
        return -1;
    }

    char hostname[256];
    getnameinfo ((struct sockaddr *) &surfgpu->clntAddr, sizeof (surfgpu->clntAddr), 
                 hostname, sizeof hostname, NULL, 0, NI_NUMERICHOST);
    if (0==strcmp("127.0.0.1", hostname)) {
        printf ("SurfGPU:\tHandling SHM client %s\n", inet_ntoa (surfgpu->clntAddr.sin_addr));
        return SURFGPU_CLIENT_SHM;
    }
    else {
        printf ("SurfGPU:\tHandling TCP client %s\n", inet_ntoa (surfgpu->clntAddr.sin_addr));
        return SURFGPU_CLIENT_TCP;
    }
}

static void
HandleTCPClient (surfgpu_t *surfgpu)
{
    // int32_t max_size;
    int32_t buflen, msglen;

    /* Receive marshalled image data from client */
    if (recv (surfgpu->clntSock, &msglen, sizeof (msglen), 0) != sizeof (msglen)) {
        PERROR ("recv() failed");
        close (surfgpu->clntSock);
        return;
    }
  
    msglen = ntohl (msglen);
    buflen = msglen - sizeof (msglen);

    if (buflen > surfgpu->bufsize) {
        ERROR ("buflen [%d] is greater than bufsize [%d]",
               buflen, surfgpu->bufsize);
        close (surfgpu->clntSock);
        return;
    }
    if (recv (surfgpu->clntSock, surfgpu->buf, buflen, MSG_WAITALL) != buflen) {
        PERROR ("recv() failed");
        close (surfgpu->clntSock);
        return;
    }

    /* Unmarshall image data */
    bot_core_image_t *bot = (bot_core_image_t *) malloc (sizeof (*bot));
    if (bot_core_image_t_decode (surfgpu->buf, 0, buflen, bot) < 0) {
        ERROR ("bot_core_image_t_decode() failed");
        free (bot);
        close (surfgpu->clntSock);
        return;
    }
    if (bot->pixelformat != BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
        ERROR ("only BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY is supported [%d]", bot->pixelformat);
        bot_core_image_t_destroy (bot);
        close (surfgpu->clntSock);
        return;
    }
    if (bot->row_stride != bot->width) {
        ERROR ("row_stride doesn't match width");
        bot_core_image_t_destroy (bot);
        close (surfgpu->clntSock);
        return;
    }

    try {
        runSurf (surfgpu, bot);
    }
    catch (exception const &e) {
        cout << "Exception during processing " << e.what() << endl;
        return;
    }

    /* Send marshalled feature data back to client */
    perllcm_van_feature_t *f = surfgpu_to_feature_t (surfgpu, surfgpu->n_max_features);
    // max_size = perllcm_van_feature_t_encoded_size (f);
    buflen = perllcm_van_feature_t_encode (surfgpu->buf, sizeof(msglen), surfgpu->bufsize-sizeof(msglen), f);
    msglen = buflen + sizeof (msglen);
    *((int32_t *)surfgpu->buf) = htonl (msglen);
    if (send (surfgpu->clntSock, surfgpu->buf, msglen, MSG_NOSIGNAL) != msglen) {
        PERROR ("send() failed");
        bot_core_image_t_destroy (bot);
        perllcm_van_feature_t_destroy (f);
        close (surfgpu->clntSock);
        return;
    }

    /* Clean up */
    bot_core_image_t_destroy (bot);
    perllcm_van_feature_t_destroy (f);
    close (surfgpu->clntSock);

    return;
}

static void
HandleSHMClient (surfgpu_t *surfgpu)
{
    int32_t msg, buflen;
    // int32_t max_size;

    /* Receive request from client */
    if (recv (surfgpu->clntSock, &msg, sizeof (msg), 0) != sizeof (msg)) {
        PERROR ("recv() failed");
        close (surfgpu->clntSock);
        return;
    }

    /* Acknowledge client's request */
    msg = htonl (surfgpu->shmsize);
    if (send (surfgpu->clntSock, &msg, sizeof (msg), MSG_NOSIGNAL) != sizeof (msg)) {
        PERROR ("send() failed");
        close (surfgpu->clntSock);
        return;
    }

    /* Wait for client ack that image data has been written to shared memory */
    if (recv (surfgpu->clntSock, &msg, sizeof (msg), 0) != sizeof (msg)) {
        PERROR ("recv() failed");
        close (surfgpu->clntSock);
        return;
    }
    buflen = ntohl (msg);

    /* Unmarshall image data */
    bot_core_image_t *bot = (bot_core_image_t *) malloc (sizeof (*bot));
    if (bot_core_image_t_decode (surfgpu->shm, 0, buflen, bot) < 0) {
        ERROR ("bot_core_image_t_decode() failed");
        free (bot);
        close (surfgpu->clntSock);
        return;
    }
    if (bot->pixelformat != BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
        ERROR ("only BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY is supported [%d]", bot->pixelformat);
        bot_core_image_t_destroy (bot);
        close (surfgpu->clntSock);
        return;
    }
    if (bot->row_stride != bot->width) {
        ERROR ("row_stride doesn't match width");
        bot_core_image_t_destroy (bot);
        close (surfgpu->clntSock);
        return;
    }

    try {
        runSurf (surfgpu, bot);
    }
    catch (exception const &e) {
        cout << "Exception during processing " << e.what() << endl;
        return;
    }

    /* Send marshalled feature data back to client */
    perllcm_van_feature_t *f = surfgpu_to_feature_t (surfgpu, surfgpu->n_max_features);
    // max_size = perllcm_van_feature_t_encoded_size (f);
    buflen = perllcm_van_feature_t_encode (surfgpu->shm, 0, surfgpu->shmsize, f);
    msg = htonl (buflen);
    if (send (surfgpu->clntSock, &msg, sizeof (msg), MSG_NOSIGNAL) != sizeof (msg)) {
        PERROR ("send() failed");
        bot_core_image_t_destroy (bot);
        perllcm_van_feature_t_destroy (f);
        close (surfgpu->clntSock);
        return;
    }

    /* Wait for client ack that feature data has been read from shared memory */
    if (recv (surfgpu->clntSock, &msg, sizeof (msg), 0) != sizeof (msg)) {
        PERROR ("recv() failed");
        close (surfgpu->clntSock);
        return;
    }


    /* Clean up */
    bot_core_image_t_destroy (bot);
    perllcm_van_feature_t_destroy (f);
    close (surfgpu->clntSock);
    return;
}

static int
CreateSHM (surfgpu_t *surfgpu)
{
    // init surfgpu's ipc shared memory segment
    surfgpu->shmsize = surfgpu->bufsize;
    if ((surfgpu->shmid = shmget (VIS_SURFGPU_SHM_KEY, surfgpu->shmsize, 0644 | IPC_CREAT)) == -1) {
        PERROR ("shmget: unable to create shared memory segment");
        return -1;
    }

    // attach to ipc shared memory segment and get a pointer to it
    surfgpu->shm = (uint8_t *) shmat (surfgpu->shmid, NULL, 0);
    if (surfgpu->shm == (uint8_t *)(-1)) {
        PERROR ("shmat: unable to attach to shared memory segment");
        return -1;
    }

    return 0;
}

static int
DestroySHM (surfgpu_t *surfgpu)
{
    if (shmdt (surfgpu->shm) < 0) {
        PERROR ("shmdt() failed");
        return -1;
    }

    if (shmctl (surfgpu->shmid, IPC_RMID, NULL) < 0) {
        PERROR ("shmctl() failed");
        return -1;
    }
    return 0;
}

static bool done = 0;
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    done = 1;
}

static void
setSurfParams (surfgpu_t *surfgpu, perllcm_van_cvsurf_params_t *params)
{
    if (surfgpu->surf)
        delete surfgpu->surf;
    surfgpu->surf = new cv::gpu::SURF_GPU (params->hessianThreshold,
                                           params->nOctaves,
                                           params->nOctaveLayers,
                                           params->extended);
}

static void
prepGpu (surfgpu_t *surfgpu, getopt_t *gopt)
{
    printf ("SurfGPU:\tInitializing GPU...\n");

    perllcm_van_cvsurf_params_t surfgpuParams;
    surfgpuParams.hessianThreshold = getopt_get_double (gopt, "hessian-threshold");
    surfgpuParams.nOctaves         = getopt_get_int (gopt, "n-octaves");
    surfgpuParams.nOctaveLayers    = getopt_get_int (gopt, "n-octave-layers");
    surfgpuParams.extended         = !getopt_get_bool (gopt, "short");

    setSurfParams (surfgpu, &surfgpuParams);

    surfgpu->keypointsGpu = new cv::gpu::GpuMat;
    surfgpu->descriptorsGpu = new cv::gpu::GpuMat;
    
    printf ("SurfGPU:\tDone initializing GPU.\n");
}

static void
perllcm_van_cvsurf_params_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                      const perllcm_van_cvsurf_params_t *msg, void *user)
{
    printf ("SurfGPU:\tSetting the detector parameters...\n");
    surfgpu_t *surfgpu = (surfgpu_t *)user;

    perllcm_van_cvsurf_params_t surfgpuParams;
    surfgpuParams.hessianThreshold = msg->hessianThreshold;
    surfgpuParams.nOctaves         = msg->nOctaves;
    surfgpuParams.nOctaveLayers    = msg->nOctaveLayers;
    surfgpuParams.extended         = msg->extended;

    setSurfParams (surfgpu, &surfgpuParams);
}

int main (int argc, char* argv[])
{  
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // options
    char default_port_str[16];
    snprintf (default_port_str, sizeof (default_port_str), "%d", VIS_SURFGPU_TCP_PORT);
    char default_npts_str[16];
    snprintf(default_npts_str, sizeof (default_npts_str), "%d", VIS_SURFGPU_MAX_FEATURES);
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, DESCRIPTION);
    getopt_add_bool   (gopt, 'D',  "daemon",              0,                "Run as system daemon");
    getopt_add_int    (gopt, 'p',  "port",                default_port_str, "Server TCP/IP port");
    getopt_add_int    (gopt, 'b',  "bufsize",             "20971520",       "Server buffer size (bytes)");  
    getopt_add_double (gopt, 't',  "hessian-threshold",   "500.0",          "Hessian threshold");
    getopt_add_int    (gopt, 'v',  "n-octaves",           "4",              "Number of octaves");
    getopt_add_int    (gopt, 'l',  "n-octave-layers",     "2",              "Number of octave layers");
    getopt_add_bool   (gopt, 's',  "short"      ,         0,                "Use 64-length descriptor?");
    getopt_add_int    (gopt, 'n',  "n-features",          default_npts_str, "The target number of features to return (0 for max)");
    getopt_add_help   (gopt, NULL);

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len !=0) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    surfgpu_t *surfgpu = (surfgpu_t *) calloc (1, sizeof (*surfgpu));
    if (!surfgpu) {
        ERROR ("malloc: surfgpu_t");
        exit (EXIT_FAILURE);
    }

    // options
    surfgpu->servPort = getopt_get_int (gopt, "port");
    surfgpu->bufsize = getopt_get_int (gopt, "bufsize");
    surfgpu->n_max_features   = getopt_get_int (gopt, "n-features");

    // allocate receive buffer
    surfgpu->buf = (uint8_t *) malloc (surfgpu->bufsize);
    if (!surfgpu->buf) {
        ERROR ("malloc() failed");
        exit (EXIT_FAILURE);
    }

    //lcm
    surfgpu->lcm = lcm_create(NULL);
    perllcm_van_cvsurf_params_t_subscription_t *perllcm_van_cvsurf_params_t_sub =
        perllcm_van_cvsurf_params_t_subscribe (surfgpu->lcm, "SURFGPU_PARAMS", &perllcm_van_cvsurf_params_t_callback, surfgpu);

    if (CreateTCPServerSocket (surfgpu) < 0) {
        ERROR ("CreateTCPServerSocket() failed");
        if (errno == EADDRINUSE)
            ERROR ("is another instance of surfgpu-server already running?");
        exit (EXIT_FAILURE);
    }

    if (CreateSHM (surfgpu) < 0) {
        ERROR ("CreateSHM() failed");
        exit (EXIT_FAILURE);
    }

    // install custom signal handler
    struct sigaction act = {{0}};
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    //Initialize the GPU (this sometimes takes a minute)
    prepGpu(surfgpu, gopt);

    if (getopt_get_bool (gopt, "daemon")) {
        printf ("SurfGPU:\tRunning as daemon\n");
        daemon_fork ();
    }

    while (!done) {
        int lcm_fd = lcm_get_fileno (surfgpu->lcm);
        fd_set sockSet;
        FD_ZERO (&sockSet);
        FD_SET (surfgpu->servSock, &sockSet);
        FD_SET(lcm_fd, &sockSet);

        struct timeval timeout = {0};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select (surfgpu->servSock+1, &sockSet, NULL, NULL, &timeout);
        if (ret < 0)
            PERROR ("select()");
        else if (ret == 0) {
            printf ("SurfGPU:\tNo requests...\n");
        }
        else if (FD_ISSET (surfgpu->servSock, &sockSet)) {
            printf ("SurfGPU:\tRequest received\n");
            switch (AcceptTCPConnection (surfgpu)) {
            case SURFGPU_CLIENT_SHM:
                HandleSHMClient (surfgpu);
                break;
            case SURFGPU_CLIENT_TCP:
                HandleTCPClient (surfgpu);
                break;
            default:
                break;
            }
        }
        else if (FD_ISSET (lcm_fd, &sockSet)) {
            // LCM has events ready to be processed.
            lcm_handle (surfgpu->lcm);
        }
    }

    DestroySHM (surfgpu);
    free (surfgpu->buf);
    close (surfgpu->servSock);
    //Free the GpuSurfDetector class
    free (surfgpu);  
    //Free any strdup's, strings, etc.

    //lcm subscriptions
    perllcm_van_cvsurf_params_t_unsubscribe (surfgpu->lcm, perllcm_van_cvsurf_params_t_sub);

    fprintf (stderr, "\nSurfGPU:\tGoodbye\n");
    exit (EXIT_SUCCESS);
}