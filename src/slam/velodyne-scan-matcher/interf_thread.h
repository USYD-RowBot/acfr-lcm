#ifndef PERLS_VELODYNE_SCAN_MATCHER_INTERFACE_THREAD_H
#define PERLS_VELODYNE_SCAN_MATCHER_INTERFACE_THREAD_H

#include <lcm/lcm-cpp.hpp>

#include "perls-common/bot_util.h"
#include "perls-common/cache.h"
#include "perls-common/error.h"
#include "perls-common/units.h"

#include "perls-sensors/velodyne.h"

#include "perls-lcmtypes++/perllcm/position_t.hpp"
#include "perls-lcmtypes++/perllcm/heartbeat_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_plink_collection_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_plink_t.hpp"
#include "perls-lcmtypes++/perllcm/isam_vlink_t.hpp"
#include "perls-lcmtypes++/perllcm/pose3d_t.hpp"
#include "perls-lcmtypes++/senlcm/velodyne_t.hpp"


typedef struct _interf_thread_data_t interf_thread_data_t;
struct _interf_thread_data_t {
    lcm::LCM *lcm;
    BotParam *param;
    GAsyncQueue *gq;
    BotPtrCircular   *velodyne_data_delay_circ;
    GMutex *circ_mutex;
    int *done;
};

typedef enum _queue_element_type_t {
    QE_DROP_SCAN = 0,
    QE_PLINK = 1,
} queue_element_type_t;

typedef struct _queue_element_t queue_element_t;
struct _queue_element_t {
    
    int64_t utime;
    queue_element_type_t type;
    void *msg;
};

void * 
interf_thread_f(void *user_data);

#endif
