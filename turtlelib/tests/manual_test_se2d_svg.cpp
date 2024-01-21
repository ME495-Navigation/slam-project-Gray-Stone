//! @file This is a manual test executable for using Se2d and Svg class.
//! This file should not be registed in CTest as it won't contain any actual
//! testing. 
//! After running this file, a generated output

#include <turtlelib/svg.hpp>
#include <fstream>
#include<iostream>
using namespace turtlelib;

void some_fun(){
        // This is not a necessary, but make's things more obvious
    Transform2D frame_s{{0,0} , 0.0};

    Transform2D frame_1{{120,80} , 0.0};
    Transform2D frame_2{{-10,50} , 0.0};


    
    // Vectors and frames are tailed with the frame they are in.
    Vector2D va_s{30,60};

    Vector2D va_1 = frame_1(va_s);

    Svg svg;
    std::string object_contents;
    object_contents += svg.MakeFrame(frame_1, "1");
    object_contents += svg.MakeObject(va_s); 
    object_contents += svg.WarpWithGroupTransform(svg.MakeObject(va_1), frame_1);
    // object_contents += svg.WarpWithGroupTransform(svg.MakeFrame(frame_2,"2"), frame_1);

    object_contents += svg.WarpWithGroupTransform(svg.MakeObject(va_1), frame_1);
    std::ofstream out_stream("manual_file.svg");
    
    // out_stream<<std::endl;
    // out_stream.close();
    svg.FinishAndWriteToFile(out_stream,object_contents);


}

int main(){

    // // This is not a necessary, but make's things more obvious
    // Transform2D frame_s{{0,0} , 0.0};

    // Transform2D frame_1{{120,80} , 0.0};
    
    // // Vectors and frames are tailed with the frame they are in.
    // Vector2D va_s{30,60};

    // Vector2D va_1 = frame_1(va_s);

    // Svg svg;
    // std::string object_contents;
    // object_contents += svg.MakeFrame(frame_1, "1");
    // object_contents += svg.MakeObject(va_s); 
    // object_contents += svg.WarpWithGroupTransform(svg.MakeObject(va_1), frame_1);

    // std::ofstream out_stream("manual_file.svg");
    
    // svg.FinishAndWriteToFile(out_stream,object_contents);
    // out_stream<<std::endl;
    // out_stream.close();

some_fun();

std::cout<<"done!"<<std::endl;
}