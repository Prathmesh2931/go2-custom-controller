#include<rclcpp.rclcpp.hpp>

class ik_node : public rclcpp ::Node 
{
    private :
        struct Legangles {double hip; double upper; double lower;};
        
        Legangles ik(double x, double y, double z)
        {
            const float l0 = 0.0955f; // horizontal distance from hip to thigh
            const float l1 = 0.213f; // length of thigh
            const float l2 = 0.213f; // length of calf
            const float knee_dir = -1.0f; // direction of knee bend, 1 for forward, -1 for backward

            float hip = atan2(y, -z) - atan2(l0, sqrt(x*x + z*z - l0*l0)); // angle of hip joint, accounting for hip offset

            float z2d = sqrt(y*y + z*z - l0*l0); // z distance in the 2D plane of the leg, after accounting for hip offset

            float calf = knee_dir * acos((x*x + z2d*z2d - l1*l1 - l2*l2) / (2*l1*l2)); // law of cosines to find angle of calf joint
            float thigh = atanf(x/z2d) - atanf(l2*sin(calf) /l1 + l2*cos(calf)); // angle of thigh joint

            return {hip, thigh, calf};
        }

}