#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>

void getNewQuat(){
    tf2::Quaternion q_orig, q_rot, q_new;

    q_orig.setRPY(0.0, 0.0, 0.0);

    q_rot.setRPY(3.14159, 0.0, 0.0);
    q_new = q_rot * q_orig;
    q_new.normalize();
    std::cout << q_new.x() << ", "
                << q_new.y() << ", "
                << q_new.z() << ", "
                << q_new.w() << ", " << std::endl;
}
    
int main(){
    getNewQuat();
    return 0;
}