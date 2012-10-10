#include "vehiclestate.h"

VehicleState::VehicleState() :
	m_time(0),
	m_node(NULL),
	m_vehiclePose(Pose2d(0,0,0))
{
}

VehicleState::VehicleState(int64_t time, Pose2d_Node* node, const Pose2d & vehiclePose) :
	m_time(time),
	m_node(node),
	m_vehiclePose(vehiclePose)
{
}

VehicleState::~VehicleState() 
{
}

