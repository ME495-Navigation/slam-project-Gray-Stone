#include "turtlelib/geometry2d.hpp"


namespace turtlelib{
    
    double normalize_angle(double rad){
        // Citation ---------- [2] ----------
        return rad - (std::ceil((rad + PI) / (2 * PI)) - 1) * 2 * PI;
    }
    
}