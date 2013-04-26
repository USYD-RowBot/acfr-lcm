#include <small/Pose3D.hh>

using namespace std;

int main() {
    SMALL::Pose3D auvPose;
    auvPose.setPosition(0.0,0.0,1.0);
    auvPose.setRollPitchYawRad(0.0, 0.0, 0.0);
    
    SMALL::Vector4D nu1, nu2;
    nu1 = 1.0, 0.0, 0.0, 0.0;
    nu2 = 0.0, 1.1, 0.0, 0.0;
    
    
   // cout << "rotation = " << auvPose.getRotationMatrix().toString() << endl;
    
        
    SMALL::Vector3D eta_dot;
    eta_dot = (auvPose.get3x4TransformationMatrix() * nu1);
    cout << eta_dot.toString() << endl << endl;
    
    eta_dot = (auvPose.get3x4TransformationMatrix() * nu2);
    cout << eta_dot.toString() << endl << endl;

    
    return 0;
}
