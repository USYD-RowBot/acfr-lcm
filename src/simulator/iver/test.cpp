#include <small/Pose3D.hh>

int main() {
    SMALL::Pose3D auvPose;
    auvPose.setPosition(0,0,1);
    auvPose.setRollPitchYawRad(0, 0, 0);
    
    SMALL::Vector6D nu;
    nu = 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    
    SMALL::Vector6D eta_dot;
    eta_dot = auvPose.transformTo(nu);
    std::cout << eta_dot.toString() << std::endl;
    
    return 0;
}
