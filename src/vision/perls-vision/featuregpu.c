#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <opencv/cv.h>

#include "perls-lcmtypes/bot_core_image_t.h"
#include "perls-lcmtypes/perllcm_van_feature_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_cvsurf_t.h"
#include "perls-lcmtypes/perllcm_van_feature_attr_siftgpu_t.h"

#include "perls-common/error.h"
#include "perls-common/magic.h"

#include "featuregpu.h"

static int
ConnectToServer (const char *ipaddr, int port, int feature_type)
{
    /* Determine the default port, based on the feature type */
    uint16_t default_port;
    switch (feature_type)
    {
    case VIS_FEATUREGPU_TYPE_SIFT:
        default_port = VIS_SIFTGPU_TCP_PORT;
        break;
    case VIS_FEATUREGPU_TYPE_SURF:
        default_port = VIS_SURFGPU_TCP_PORT;
        break;
    default:
        ERROR("unknown feature type");
        return -1;
    }

    /* Create a reliable, stream socket using TCP */
    int sock;
    if ((sock = socket (PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    {
        PERROR ("socket() failed");
        return -1;
    }

    /* Establish connection to the server */
    struct sockaddr_in servAddr =
    {
        .sin_family = AF_INET,
        .sin_addr.s_addr = ipaddr ? inet_addr (ipaddr) : inet_addr ("127.0.0.1"),
        .sin_port = port > 0 ? htons (port) : htons (default_port),
    };
    if (connect (sock, (struct sockaddr *) &servAddr, sizeof (servAddr)) < 0)
    {
        if (errno == ECONNREFUSED)
            ERROR ("Is the gpu server running?");
        else
            PERROR ("connect() failed");
        close (sock);
        return -1;
    }

    return sock;
}

static bot_core_image_t
bot_core_image_t_view (const IplImage *image)
{
    bot_core_image_t bot =
    {
        .width = image->width,
        .height = image->height,
        .row_stride = image->widthStep,
        .pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY,
        .size = image->imageSize,
        .data = (uint8_t *) image->imageData,
    };
    return bot;
}

static perllcm_van_feature_t *
featuregpu_client_SHM (const IplImage *image, const char *server_ipaddr, int server_port, int npts_max, int feature_type)
{

    uint32_t shm_key;
    switch (feature_type)
    {
    case VIS_FEATUREGPU_TYPE_SIFT:
        shm_key = VIS_SIFTGPU_SHM_KEY;
        break;
    case VIS_FEATUREGPU_TYPE_SURF:
        shm_key = VIS_SURFGPU_SHM_KEY;
        break;
    default:
        ERROR("unknown feature type");
        return NULL;
    }

    if (image->depth != IPL_DEPTH_8U)
    {
        ERROR ("only IPL_DEPTH_8U is supported");
        return NULL;
    }

    int sock = ConnectToServer (server_ipaddr, server_port, feature_type);
    if (sock < 0)
        return NULL;

    /* Send request to server */
    int32_t msg = 0;
    if (send (sock, &msg, sizeof (msg), MSG_NOSIGNAL) != sizeof (msg))
    {
        PERROR ("send() failed");
        close (sock);
        return NULL;
    }

    /* Receive ack back with shmsize info*/
    if (recv (sock, &msg, sizeof (msg), 0) != sizeof (msg))
    {
        PERROR ("recv() failed or connection closed prematurly");
        close (sock);
        return NULL;
    }
    int32_t shmsize = ntohl (msg);

    /* Connect to server's shared memory segment */
    int32_t shmid;
    if ((shmid = shmget (shm_key, shmsize, 0644 | IPC_CREAT)) < 0)
    {
        PERROR ("shmget() failed");
        close (sock);
        return NULL;
    }
    uint8_t *shm = shmat (shmid, NULL, 0);
    if (shm == (uint8_t *)(-1))
    {
        PERROR ("shmat() failed");
        close (sock);
        return NULL;
    }

    /* Send marshalled bot_core_image_t to server */
    bot_core_image_t bot = bot_core_image_t_view (image);
    int32_t buflen, maxsize;
    maxsize = bot_core_image_t_encoded_size (&bot);
    if (maxsize > shmsize)
    {
        ERROR ("maxsize [%d] is greater than shmsize [%d], fail", maxsize, shmsize);
        close (sock);
        return NULL;
    }
    if ((buflen = bot_core_image_t_encode (shm, 0, shmsize, &bot)) < 0)
    {
        ERROR ("bot_core_image_t_encode() failed");
        close (sock);
        return NULL;
    }
    msg = htonl (buflen);
    if (send (sock, &msg, sizeof (msg), MSG_NOSIGNAL) != sizeof (msg))
    {
        PERROR ("send() failed");
        close (sock);
        return NULL;
    }

    /* Receive marshalled perllcm_van_feature_t back */
    if (recv (sock, &msg, sizeof (msg), 0) != sizeof (msg))
    {
        PERROR ("recv() failed");
        close (sock);
        return NULL;
    }
    buflen = ntohl (msg);
    perllcm_van_feature_t *f = malloc (sizeof (*f));
    if (perllcm_van_feature_t_decode (shm, 0, buflen, f) < 0)
    {
        ERROR ("perllcm_van_feature_t_decode() failed");
        free (f);
        close (sock);
        return NULL;
    }

    /* Send ack back to server so that it knows we are done with shm */
    msg = htonl (buflen);
    if (send (sock, &msg, sizeof (msg), MSG_NOSIGNAL) != sizeof (msg))
    {
        PERROR ("send() failed");
        close (sock);
        return NULL;
    }

    /* Clean up */
    if (shmdt (shm) < 0)
    {
        PERROR ("shmdt() failed");
    }
    close (sock);
    return f;
}

static perllcm_van_feature_t *
featuregpu_client_TCP (const IplImage *image, const char *server_ipaddr, int server_port, int npts_max, int feature_type)
{
    if (image->depth != IPL_DEPTH_8U)
    {
        ERROR ("only IPL_DEPTH_8U is supported");
        return NULL;
    }

    int sock = ConnectToServer (server_ipaddr, server_port, feature_type);
    if (sock < 0)
        return NULL;

    /* Send marshalled image data to server */
    bot_core_image_t bot = bot_core_image_t_view (image);
    int32_t msglen, buflen, bufsize, maxsize;
    maxsize = bot_core_image_t_encoded_size (&bot);
    bufsize = maxsize + sizeof (msglen);
    uint8_t *buf = malloc (bufsize);
    if (!buf)
    {
        PERROR ("malloc() failed");
        return NULL;
    }
    buflen = bot_core_image_t_encode (buf, sizeof(msglen), bufsize-sizeof(msglen), &bot);
    if (buflen < 0)
    {
        ERROR ("bot_core_image_t_encode() failed");
        free (buf);
        close (sock);
        return NULL;
    }
    msglen = buflen + sizeof (msglen);
    *((int32_t *)buf) = htonl (msglen);
    if (send (sock, buf, msglen, MSG_NOSIGNAL) != msglen)
    {
        PERROR ("send() failed");
        free (buf);
        close (sock);
        return NULL;
    }


    /* Receive marshalled feature data back */
    if (recv (sock, &msglen, sizeof (msglen), 0) != sizeof (msglen))
    {
        PERROR ("recv() failed or connection closed prematurly");
        free (buf);
        close (sock);
        return NULL;
    }
    msglen = ntohl (msglen);
    buflen = msglen - sizeof (msglen);
    if (buflen > bufsize)   // reallocate a larger buffer and be noisy about it
    {
        printf ("Warning: buflen [%d] is greater than bufsize [%d], consider increasing bufsize\n",
                buflen, bufsize);
        free (buf);
        if (!(buf = malloc (buflen)))
        {
            PERROR ("malloc() failed");
            free (buf);
            close (sock);
            return NULL;
        }
    }
    if (recv (sock, buf, msglen, MSG_WAITALL) != buflen)
    {
        PERROR ("recv() failed or connection closed prematurly");
        free (buf);
        close (sock);
        return NULL;
    }

    /* Unmarshal feature data */
    perllcm_van_feature_t *f = malloc (sizeof (*f));
    if (perllcm_van_feature_t_decode (buf, 0, buflen, f) < 0)
    {
        ERROR ("perllcm_van_feature_t_decode() failed");
        free (f);
        free (buf);
        close (sock);
        return NULL;
    }

    // clean up
    free (buf);
    close (sock);
    return f;
}

static perllcm_van_feature_t *
apply_image_mask_sift (const perllcm_van_feature_t *f, const IplImage *mask)
{
    if (!mask)
        return (perllcm_van_feature_t *) f;

    perllcm_van_feature_t *fm = perllcm_van_feature_t_copy (f);
    perllcm_van_feature_attr_siftgpu_t *attr = malloc (sizeof (*attr));
    perllcm_van_feature_attr_siftgpu_t_decode (f->attr, 0, f->attrsize, attr);

    int stride = mask->widthStep;
    uint8_t *data = (uint8_t *) mask->imageData;
    for (int i=0, n=0; i<f->npts; i++)
    {
        int ui = f->u[i], vi = f->v[i];
        uint8_t pixel = *(data + vi*stride + ui);
        if (pixel > 0)
        {
            fm->u[n] = f->u[i];
            fm->v[n] = f->v[i];
            memcpy (fm->keys[n], f->keys[i], f->keylen * sizeof (*f->keys[i]));
            attr->s[n] = attr->s[i];
            attr->o[n] = attr->o[i];
            attr->npts = fm->npts = ++n;
        }
    }

    // free double pointer keys for any discarded points
    for (int i=fm->npts; i<f->npts; i++)
        free (fm->keys[i]);

    free (fm->attr);
    int32_t maxsize = perllcm_van_feature_attr_siftgpu_t_encoded_size (attr);
    fm->attr = malloc (maxsize);
    fm->attrsize = perllcm_van_feature_attr_siftgpu_t_encode (fm->attr, 0, maxsize, attr);
    perllcm_van_feature_attr_siftgpu_t_destroy (attr);

    return fm;
}

static perllcm_van_feature_t *
apply_image_mask_surf (const perllcm_van_feature_t *f, const IplImage *mask)
{
    if (!mask)
        return (perllcm_van_feature_t *) f;

    perllcm_van_feature_t *fm = perllcm_van_feature_t_copy (f);
    perllcm_van_feature_attr_cvsurf_t *attr = malloc (sizeof (*attr));
    perllcm_van_feature_attr_cvsurf_t_decode (f->attr, 0, f->attrsize, attr);

    int stride = mask->widthStep;
    uint8_t *data = (uint8_t *) mask->imageData;
    for (int i=0, n=0; i<f->npts; i++)
    {
        int ui = f->u[i], vi = f->v[i];
        uint8_t pixel = *(data + vi*stride + ui);
        if (pixel > 0)
        {
            fm->u[n] = f->u[i];
            fm->v[n] = f->v[i];
            memcpy (fm->keys[n], f->keys[i], f->keylen * sizeof (*f->keys[i]));
            attr->dir[n] = attr->dir[i];
            attr->size[n] = attr->size[i];
            attr->npts = fm->npts = ++n;
        }
    }

    // free double pointer keys for any discarded points
    for (int i=fm->npts; i<f->npts; i++)
        free (fm->keys[i]);

    free (fm->attr);
    int32_t maxsize = perllcm_van_feature_attr_cvsurf_t_encoded_size (attr);
    fm->attr = malloc (maxsize);
    fm->attrsize = perllcm_van_feature_attr_cvsurf_t_encode (fm->attr, 0, maxsize, attr);
    perllcm_van_feature_attr_cvsurf_t_destroy (attr);

    return fm;
}

static perllcm_van_feature_t *
vis_featuregpu_client (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max, int feature_type)
{
    perllcm_van_feature_t *f = NULL;
    if (server_ipaddr == NULL ||
            0==strcmp ("localhost", server_ipaddr) ||
            0==strcmp ("127.0.0.1", server_ipaddr))
    {
        f = featuregpu_client_SHM (image, "127.0.0.1", server_port, npts_max, feature_type);
    }
    else
        f = featuregpu_client_TCP (image, server_ipaddr, server_port, npts_max, feature_type);

    if (f)
    {
        if (mask)
        {
            perllcm_van_feature_t *fm = NULL;

            switch (feature_type)
            {
            case VIS_FEATUREGPU_TYPE_SIFT:
                fm = apply_image_mask_sift (f, mask);
                break;
            case VIS_FEATUREGPU_TYPE_SURF:
                fm = apply_image_mask_surf (f, mask);
                break;
            default:
                ERROR("unknown feature type");
                return NULL;
            }

            perllcm_van_feature_t_destroy (f);
            return fm;
        }
        else
            return f;
    }
    else
        return NULL;
}

perllcm_van_feature_t *
vis_siftgpu_client (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max)
{
    return vis_featuregpu_client(image, mask, server_ipaddr, server_port, npts_max, VIS_FEATUREGPU_TYPE_SIFT);
}

perllcm_van_feature_t *
vis_surfgpu_client (const IplImage *image, const IplImage *mask, const char *server_ipaddr, int server_port, int npts_max)
{
    return vis_featuregpu_client(image, mask, server_ipaddr, server_port, npts_max, VIS_FEATUREGPU_TYPE_SURF);
}

