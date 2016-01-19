/// \author Quirin Körner: ga34diz

#pragma once

#include <ros/ros.h>

//Open CV includes

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//std include
#include <iostream>
#include <string>


//++++ Functions +++++/

int main(int argc, char** argv);
void load_map();
void display_mapReal();
void display_mapTest();

//+++++ Variable ++++++/
cv::Mat mapReal;
cv::Mat mapTest;







