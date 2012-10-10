#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

#if OpenCV_VERSION >= 230
#include <opencv/cv.hpp>
#endif

// external linking req'd
#include <gsl/gsl_math.h>
#include <gsl/gsl_sort.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <siftgpu/SiftGPU.h>

#include "perls-common/daemon.h"
#include "perls-common/error.h"
#include "perls-common/getopt.h"

#include "perls-vision/botimage.h"
#include "perls-vision/feature.h"
#include "perls-vision/featuregpu.h"

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_siftgpu_t.h"
#include "perls-lcmtypes/perllcm_van_siftgpu_params_t.h"

#define SIFTGPU_MAX_OPTIONS  64
#define SIFTGPU_MAX_PENDING  100 /* Maximum outstanding connection requests */
#define SIFTGPU_MAX_FEATURES 1000

#define SIFTGPU_CLIENT_TCP  1
#define SIFTGPU_CLIENT_SHM  2

#define DESCRIPTION                                                     \
    "SiftGPU server encapsulates the C++ SiftGPU object code into a standalone process.\n" \
    "Client processes interact with siftgpu-server via a TCP/SHM API."

typedef struct siftgpu siftgpu_t;
struct siftgpu {
    SiftGPU *sift;

    getopt_t *gopt;
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

static perllcm_van_feature_t *
siftgpu_to_feature_t (SiftGPU *sift, int max_npts)
{
    // allocate feature_t
    int npts = sift->GetFeatureNum ();
    perllcm_van_feature_t *f = (perllcm_van_feature_t *) calloc (1, sizeof (*f));
    f->npts = GSL_MIN (npts, max_npts);
    f->u = (float *) malloc (f->npts * sizeof (*f->u));
    f->v = (float *) malloc (f->npts * sizeof (*f->v));
    f->keylen = 128;
    f->keys = (float **) malloc (f->npts * sizeof (*f->keys));

    // read back keypoints and descriptors from GPU
    SiftGPU::SiftKeypoint *keys = (SiftGPU::SiftKeypoint *) malloc (npts * sizeof (*keys)); // {x, y, scale, orientation}
    float *descriptors = (float *) malloc (npts * f->keylen * sizeof (*descriptors));
    sift->GetFeatureVector (&keys[0], &descriptors[0]);

    // sort keypoints by scale?, yes, otherwise they are ordered spatially and reducing max points
    // causes weird spatial distributions
    size_t *p = (size_t *) malloc (f->npts * sizeof (*p));
    double *scales = (double *) malloc (npts * sizeof (*scales));
    for (int n=0; n<npts; n++ ) {
        scales[n] = keys[n].s;
    }
    gsl_sort_largest_index (p, (size_t)f->npts, scales, 1, npts);

    // store keypoints and attributes
    perllcm_van_feature_attr_siftgpu_t *attr = (perllcm_van_feature_attr_siftgpu_t *) calloc (1, sizeof (*attr));
    attr->npts = f->npts;
    attr->s = (float *) malloc (f->npts * sizeof (*attr->s));
    attr->o = (float *) malloc (f->npts * sizeof (*attr->o));
    for (int i=0; i < f->npts; i++) {
        int n = p[i];
        f->u[i] = keys[n].x;
        f->v[i] = keys[n].y;
        f->keys[i] = (float *) malloc (f->keylen * sizeof (*f->keys[i]));
        memcpy (f->keys[i], &descriptors[f->keylen*n], f->keylen * sizeof (*f->keys[i]));
        attr->s[i] = keys[n].s;
        attr->o[i] = keys[n].o;
    }
    //printf ("Scale: Min: %f, Max: %f \n", attr->s[f->npts-1], attr->s[0]);

    int32_t max_attr_size = perllcm_van_feature_attr_siftgpu_t_encoded_size (attr);
    f->attrtype = PERLLCM_VAN_FEATURE_T_ATTRTYPE_SIFTGPU;
    f->attr = (uint8_t *) malloc (max_attr_size);
    f->attrsize = perllcm_van_feature_attr_siftgpu_t_encode (f->attr, 0, max_attr_size, attr);

    // assign null user to feature_t
    f->usertype = PERLLCM_VAN_FEATURE_T_USERTYPE_NONE;
    f->usersize = 0;
    f->user = NULL;

    // clean up
    free (p);
    free (scales);
    free (keys);
    free (descriptors);
    perllcm_van_feature_attr_siftgpu_t_destroy (attr);

    return f;
}

static int
CreateTCPServerSocket (siftgpu_t *siftgpu)
{
    /* Create socket for incoming connections */
    if ((siftgpu->servSock = socket (PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        PERROR ("socket() failed");
        return -1;
    }
    int opt = 1;
    if (setsockopt (siftgpu->servSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof (opt)) < 0) {
        PERROR ("socket() failed");
        return -1;
    }

    /* Construct local address structure */
    memset (&siftgpu->servAddr, 0, sizeof (siftgpu->servAddr)); /* Zero out structure */
    siftgpu->servAddr.sin_family = AF_INET;                     /* Internet address family */
    siftgpu->servAddr.sin_addr.s_addr = htonl (INADDR_ANY);     /* Any incoming interface */
    siftgpu->servAddr.sin_port = htons (siftgpu->servPort);     /* Local port */

    /* Bind to the local address */
    if (bind (siftgpu->servSock, (struct sockaddr *) &siftgpu->servAddr, sizeof (siftgpu->servAddr)) < 0) {
        PERROR ("bind() failed");
        return -1;
    }

    /* Mark the socket so it will listen for incoming connections */
    if (listen (siftgpu->servSock, SIFTGPU_MAX_PENDING) < 0) {
        PERROR ("listen() failed");
        return -1;
    }

    return 0;
}

static int
AcceptTCPConnection (siftgpu_t *siftgpu)
{
    /* Wait for client to connect */
    uint32_t clntLen = sizeof (siftgpu->clntAddr);
    if ((siftgpu->clntSock = accept (siftgpu->servSock, (struct sockaddr *) &siftgpu->clntAddr, &clntLen)) < 0) {
        PERROR ("accept() failed");
        return -1;
    }

    char hostname[256];
    getnameinfo ((struct sockaddr *) &siftgpu->clntAddr, sizeof (siftgpu->clntAddr), 
                 hostname, sizeof hostname, NULL, 0, NI_NUMERICHOST);
    if (0==strcmp("127.0.0.1", hostname)) {
        printf ("SiftGPU:\tHandling SHM client %s\n", inet_ntoa (siftgpu->clntAddr.sin_addr));
        return SIFTGPU_CLIENT_SHM;
    }
    else {
        printf ("SiftGPU:\tHandling TCP client %s\n", inet_ntoa (siftgpu->clntAddr.sin_addr));
        return SIFTGPU_CLIENT_TCP;
    }
}

static void
HandleTCPClient (siftgpu_t *siftgpu)
{
    // int32_t max_size;
    int32_t buflen, msglen;

    /* Receive marshalled image data from client */
    if (recv (siftgpu->clntSock, &msglen, sizeof (msglen), 0) != sizeof (msglen)) {
        PERROR ("recv() failed");
        close (siftgpu->clntSock);
        return;
    }
    msglen = ntohl (msglen);
    buflen = msglen - sizeof (msglen);
    if (buflen > siftgpu->bufsize) {
        ERROR ("buflen [%d] is greater than bufsize [%d]",
               buflen, siftgpu->bufsize);
        close (siftgpu->clntSock);
        return;
    }
    if (recv (siftgpu->clntSock, siftgpu->buf, buflen, MSG_WAITALL) != buflen) {
        PERROR ("recv() failed");
        close (siftgpu->clntSock);
        return;
    }

    /* Unmarshall image data */
    bot_core_image_t *bot = (bot_core_image_t *) malloc (sizeof (*bot));
    if (bot_core_image_t_decode (siftgpu->buf, 0, buflen, bot) < 0) {
        ERROR ("bot_core_image_t_decode() failed");
        free (bot);
        close (siftgpu->clntSock);
        return;
    }
    if (bot->pixelformat != BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
        ERROR ("only BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY is supported [%d]", bot->pixelformat);
        bot_core_image_t_destroy (bot);
        close (siftgpu->clntSock);
        return;
    }
    if (bot->row_stride != bot->width) {
        ERROR ("row_stride doesn't match width");
        bot_core_image_t_destroy (bot);
        close (siftgpu->clntSock);
        return;
    }

    /* RunSIFT */
    if (1 != siftgpu->sift->RunSIFT (bot->width, bot->height, bot->data, GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
        ERROR ("RunSIFT() failed");
        bot_core_image_t_destroy (bot);
        close (siftgpu->clntSock);
        return;
    }

    /* Send marshalled feature data back to client */
    perllcm_van_feature_t *f = siftgpu_to_feature_t (siftgpu->sift, siftgpu->n_max_features);
    // max_size = perllcm_van_feature_t_encoded_size (f);
    buflen = perllcm_van_feature_t_encode (siftgpu->buf, sizeof(msglen), siftgpu->bufsize-sizeof(msglen), f);
    msglen = buflen + sizeof (msglen);
    *((int32_t *)siftgpu->buf) = htonl (msglen);
    if (send (siftgpu->clntSock, siftgpu->buf, msglen, MSG_NOSIGNAL) != msglen) {
        PERROR ("send() failed");
        bot_core_image_t_destroy (bot);
        perllcm_van_feature_t_destroy (f);
        close (siftgpu->clntSock);
        return;
    }

    /* Clean up */
    bot_core_image_t_destroy (bot);
    perllcm_van_feature_t_destroy (f);
    close (siftgpu->clntSock);
    return;
}

static int
CreateSHM (siftgpu_t *siftgpu)
{
    // init siftgpu's ipc shared memory segment
    siftgpu->shmsize = siftgpu->bufsize;
    if ((siftgpu->shmid = shmget (VIS_SIFTGPU_SHM_KEY, siftgpu->shmsize, 0644 | IPC_CREAT)) == -1) {
        PERROR ("shmget: unable to create shared memory segment");
        return -1;
    }

    // attach to ipc shared memory segment and get a pointer to it
    siftgpu->shm = (uint8_t *) shmat (siftgpu->shmid, NULL, 0);
    if (siftgpu->shm == (uint8_t *)(-1)) {
        PERROR ("shmat: unable to attach to shared memory segment");
        return -1;
    }

    return 0;
}

static int
DestroySHM (siftgpu_t *siftgpu)
{
    if (shmdt (siftgpu->shm) < 0) {
        PERROR ("shmdt() failed");
        return -1;
    }

    if (shmctl (siftgpu->shmid, IPC_RMID, NULL) < 0) {
        PERROR ("shmctl() failed");
        return -1;
    }
    return 0;
}

static void
HandleSHMClient (siftgpu_t *siftgpu)
{
    int32_t msg, buflen;
    //int max_size;

    /* Receive request from client */
    if (recv (siftgpu->clntSock, &msg, sizeof (msg), 0) != sizeof (msg)) {
        PERROR ("recv() failed");
        close (siftgpu->clntSock);
        return;
    }

    /* Acknowledge client's request */
    msg = htonl (siftgpu->shmsize);
    if (send (siftgpu->clntSock, &msg, sizeof (msg), MSG_NOSIGNAL) != sizeof (msg)) {
        PERROR ("send() failed");
        close (siftgpu->clntSock);
        return;
    }

    /* Wait for client ack that image data has been written to shared memory */
    if (recv (siftgpu->clntSock, &msg, sizeof (msg), 0) != sizeof (msg)) {
        PERROR ("recv() failed");
        close (siftgpu->clntSock);
        return;
    }
    buflen = ntohl (msg);

    /* Unmarshall image data */
    bot_core_image_t *bot = (bot_core_image_t *) malloc (sizeof (*bot));
    if (bot_core_image_t_decode (siftgpu->shm, 0, buflen, bot) < 0) {
        ERROR ("bot_core_image_t_decode() failed");
        free (bot);
        close (siftgpu->clntSock);
        return;
    }
    if (bot->pixelformat != BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
        ERROR ("only BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY is supported [%d]", bot->pixelformat);
        bot_core_image_t_destroy (bot);
        close (siftgpu->clntSock);
        return;
    }
    if (bot->row_stride != bot->width) {
        ERROR ("row_stride doesn't match width");
        bot_core_image_t_destroy (bot);
        close (siftgpu->clntSock);
        return;
    }

    /* RunSIFT */
    if (1 != siftgpu->sift->RunSIFT (bot->width, bot->height, bot->data, GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
        ERROR ("RunSIFT() failed");
        bot_core_image_t_destroy (bot);
        close (siftgpu->clntSock);
        return;
    }

    /* Send marshalled feature data back to client */
    perllcm_van_feature_t *f = siftgpu_to_feature_t (siftgpu->sift, siftgpu->n_max_features);
    // max_size = perllcm_van_feature_t_encoded_size (f);
    buflen = perllcm_van_feature_t_encode (siftgpu->shm, 0, siftgpu->shmsize, f);
    msg = htonl (buflen);
    if (send (siftgpu->clntSock, &msg, sizeof (msg), MSG_NOSIGNAL) != sizeof (msg)) {
        PERROR ("send() failed");
        bot_core_image_t_destroy (bot);
        perllcm_van_feature_t_destroy (f);
        close (siftgpu->clntSock);
        return;
    }

    /* Wait for client ack that feature data has been read from shared memory */
    if (recv (siftgpu->clntSock, &msg, sizeof (msg), 0) != sizeof (msg)) {
        PERROR ("recv() failed");
        close (siftgpu->clntSock);
        return;
    }


    /* Clean up */
    bot_core_image_t_destroy (bot);
    perllcm_van_feature_t_destroy (f);
    close (siftgpu->clntSock);
    return;
}


static bool done = 0;
static void
my_signal_handler (int signum, siginfo_t *siginfo, void *ucontext_t)
{
    done = 1;
}

//You must free the returned pointer
static void
paramsToString (getopt_t *gopt, int *siftargc, char ***siftargv, 
                const perllcm_van_siftgpu_params_t *params)
{
    *siftargc = 0;    
    *siftargv = (char **)malloc(SIFTGPU_MAX_OPTIONS * sizeof(**siftargv));

    char *tmp;
    tmp = strdup("-f");
    (*siftargv)[(*siftargc)++] = tmp;
    tmp = (char *)calloc(128, sizeof(*tmp));
    sprintf(tmp, "%f", params->filterWidthFactor);
    (*siftargv)[(*siftargc)++] = tmp;
    
    tmp = strdup("-w");
    (*siftargv)[(*siftargc)++] = tmp;
    tmp = (char *)calloc(128, sizeof(*tmp));
    sprintf(tmp, "%f", params->orientWindowFactor);
    (*siftargv)[(*siftargc)++] = tmp;
    
    tmp = strdup("-dw");
    (*siftargv)[(*siftargc)++] = tmp;
    tmp = (char *)calloc(128, sizeof(*tmp));
    sprintf(tmp, "%f", params->gridSizeFactor);
    (*siftargv)[(*siftargc)++] = tmp;
    
    tmp = strdup("-t");
    (*siftargv)[(*siftargc)++] = tmp;
    tmp = (char *)calloc(128, sizeof(*tmp));
    sprintf(tmp, "%f", params->dogThreshold);
    (*siftargv)[(*siftargc)++] = tmp;
    
    tmp = strdup("-e");
    (*siftargv)[(*siftargc)++] = tmp;
    tmp = (char *)calloc(128, sizeof(*tmp));
    sprintf(tmp, "%f", params->edgeThreshold);
    (*siftargv)[(*siftargc)++] = tmp;
    
    tmp = strdup("-d");
    (*siftargv)[(*siftargc)++] = tmp;
    tmp = (char *)calloc(128, sizeof(*tmp));
    sprintf(tmp, "%d", params->dogLevels);
    (*siftargv)[(*siftargc)++] = tmp;
    
}

static void
setSiftParams (siftgpu_t *siftgpu, const perllcm_van_siftgpu_params_t *params)
{

    char **siftargv;
    int siftargc;
    paramsToString(siftgpu->gopt, &siftargc, &siftargv, params);

    for (int i=0; i<siftargc; i++)
        printf("%s ", siftargv[i]);
    printf("\n");

    siftgpu->sift->ParseParam(siftargc, siftargv);

    if (siftgpu->sift->VerifyContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        ERROR ("siftgpu->VerifyContextGL() failed");
        exit (EXIT_FAILURE);
    }

    // Free all the stuff allocated in paramsToString()
    for (int i=0; i<siftargc; i++)
        free (siftargv[i]);

    free (siftargv);
}

static void
perllcm_van_siftgpu_params_t_callback (const lcm_recv_buf_t *rbuf, const char *channel,
                                       const perllcm_van_siftgpu_params_t *msg, void *user)
{
    printf ("SiftGPU:\tSetting the detector parameters...\n");
    siftgpu_t *siftgpu = (siftgpu_t *)user;
    setSiftParams(siftgpu, msg);
}

int
main (int argc, char *argv[])
{
    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);

    // options
    char default_port_str[16];
    snprintf (default_port_str, sizeof (default_port_str), "%d", VIS_SIFTGPU_TCP_PORT);
    char default_npts_str[16];
    snprintf (default_npts_str, sizeof (default_npts_str), "%d", SIFTGPU_MAX_FEATURES);
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, DESCRIPTION);
    getopt_add_bool   (gopt, 'D',  "daemon",   0,                "Run as system daemon");
    getopt_add_int    (gopt, 'p',  "port",     default_port_str, "Server TCP/IP port");
    getopt_add_int    (gopt, 'b',  "bufsize",  "20971520",       "Server buffer size (bytes)");  
    getopt_add_int    (gopt, 'n',  "npts",     default_npts_str, "Max SIFT points returned");
    getopt_add_string (gopt, 'o',  "options",  "\"-loweo -s 1 -fo 0 -v 1 -p 1360x1024 -tc1 1000\"", "SiftGPU options");
    getopt_add_help   (gopt, NULL);

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len !=0) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_FAILURE);
    }
    else if (getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt, NULL);
        exit (EXIT_SUCCESS);
    }

    siftgpu_t *siftgpu = (siftgpu_t *) calloc (1, sizeof (*siftgpu));
    if (!siftgpu) {
        ERROR ("malloc: siftgpu_t");
        exit (EXIT_FAILURE);
    }

    // options
    siftgpu->gopt = gopt;
    siftgpu->servPort = getopt_get_int (gopt, "port");
    siftgpu->bufsize = getopt_get_int (gopt, "bufsize");
    siftgpu->n_max_features   = getopt_get_int (gopt, "npts");

    // allocate receive buffer
    siftgpu->buf = (uint8_t *) malloc (siftgpu->bufsize);
    if (!siftgpu->buf) {
        ERROR ("malloc() failed");
        exit (EXIT_FAILURE);
    }

    siftgpu->lcm = lcm_create(NULL);
    perllcm_van_siftgpu_params_t_subscription_t *perllcm_van_siftgpu_params_t_sub =
        perllcm_van_siftgpu_params_t_subscribe (siftgpu->lcm, "SIFTGPU_PARAMS", &perllcm_van_siftgpu_params_t_callback, siftgpu);

    if (CreateTCPServerSocket (siftgpu) < 0) {
        ERROR ("CreateTCPServerSocket() failed");
        if (errno == EADDRINUSE)
            ERROR ("is another instance of siftgpu-server already running?");
        exit (EXIT_FAILURE);
    }

    if (CreateSHM (siftgpu) < 0) {
        ERROR ("CreateSHM() failed");
        exit (EXIT_FAILURE);
    }

    // SIFTGPU params
    int siftargc = 0;
    char *siftargv[SIFTGPU_MAX_OPTIONS] = {NULL};
    char *options = strdup (getopt_get_string (gopt, "options"));
    char *saveptr = NULL;
    while (char *arg = strtok_r (!saveptr ? options : NULL, " ", &saveptr)) {
        siftargv[siftargc++] = strdup (arg);
        if (siftargc == SIFTGPU_MAX_OPTIONS) {
            ERROR ("number of options cannot exceed %d", SIFTGPU_MAX_OPTIONS);
            exit (EXIT_FAILURE);
        }
    }
    free (options);

    // SIFTGPU object
    siftgpu->sift = new SiftGPU;
    siftgpu->sift->ParseParam (siftargc, siftargv);
    if (siftgpu->sift->CreateContextGL () != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        ERROR ("siftgpu->CreateContextGL() failed");
        exit (EXIT_FAILURE);
    }
    printf ("SiftGPU:\tReady!\n");

    // install custom signal handler
    struct sigaction act = {{0}};
    act.sa_sigaction = my_signal_handler;
    sigfillset (&act.sa_mask);
    act.sa_flags |= SA_SIGINFO;
    sigaction (SIGTERM, &act, NULL);
    sigaction (SIGINT,  &act, NULL);

    if (getopt_get_bool (gopt, "daemon")) {
        printf ("SiftGPU:\tRunning as daemon\n");
        daemon_fork ();
    }

    while (!done) {
        int lcm_fd = lcm_get_fileno(siftgpu->lcm);
        fd_set sockSet;
        FD_ZERO (&sockSet);
        FD_SET (siftgpu->servSock, &sockSet);
        FD_SET(lcm_fd, &sockSet);
        struct timeval timeout = {0};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select (siftgpu->servSock+1, &sockSet, NULL, NULL, &timeout);
        if (ret < 0)
            PERROR ("select()");
        else if (ret == 0) {
            printf ("SiftGPU:\tNo requests...\n");
        }
        else if (FD_ISSET (siftgpu->servSock, &sockSet)) {
            printf ("SiftGPU:\tRequest received\n");
            switch (AcceptTCPConnection (siftgpu)) {
            case SIFTGPU_CLIENT_SHM:
                HandleSHMClient (siftgpu);
                break;
            case SIFTGPU_CLIENT_TCP:
                HandleTCPClient (siftgpu);
                break;
            default:
                break;
            }
        }
        else if (FD_ISSET (lcm_fd, &sockSet)) {
            // LCM has events ready to be processed.
            lcm_handle (siftgpu->lcm);
        }
    }
    
    // clean up
    DestroySHM (siftgpu);
    free (siftgpu->buf);
    close (siftgpu->servSock);
    delete siftgpu->sift;
    free (siftgpu);
    for (int i=0; i<siftargc; i++)
        free (siftargv[i]);

    //lcm subscriptions
    perllcm_van_siftgpu_params_t_unsubscribe (siftgpu->lcm, perllcm_van_siftgpu_params_t_sub);

    fprintf (stderr, "\nSiftGPU:\tGoodbye\n");
    exit (EXIT_SUCCESS);
}
