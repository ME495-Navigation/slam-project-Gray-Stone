//! @file This is a manual test executable for using Se2d and Svg class.
//! This file should not be registed in CTest as it won't contain any actual
//! testing. 
//! After running this file, a generated output

#include <turtlelib/svg.hpp>
#include <fstream>
#include<iostream>
using namespace turtlelib;


int main(){

    // This is not a necessary, but make's things more obvious
    Transform2D frame_s{{0,0} , 0.0};

    Transform2D T_s1{{120,80} , deg2rad(45)};
    
    // Vectors and frames are tailed with the frame they are in.
    Point2D pa_s{10,20};

    Point2D pa_1 = T_s1.inv()(pa_s);

    Svg svg;
    std::string object_contents;
    svg.AddObject(T_s1, "1");
    svg.AddObject(pa_s,"purple"); 
    svg.AddObject(pa_1,"tan",T_s1);
    svg.AddObject(frame_s,"origin");

    std::ofstream out_stream("manual_file.svg");
    
    out_stream<<svg;
    out_stream.close();

std::cout<<"done!"<<std::endl;
}