#ifndef _HAUV_VEHICLESTATE_H
#define _HAUV_VEHICLESTATE_H

#include <isam/Slam.h>
#include <isam/slam2d.h>

using namespace isam;

class VehicleState
{
public:
    VehicleState();
    VehicleState(int64_t time, Pose2d_Node* node, const Pose2d & vehiclePose);
    ~VehicleState();

    int64_t time() const {return m_time;}
    Pose2d_Node* node() {return m_node;}
    Pose2d vehiclePose() const {return m_vehiclePose;}
    Pose2d value() const {return m_node->value();}
private:	
    int64_t m_time;
    Pose2d_Node* m_node;
    Pose2d m_vehiclePose;
};

#endif

