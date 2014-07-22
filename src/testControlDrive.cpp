#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <controlDrive.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Pioneer");
  TestPioneer pioneer;
  pioneer.initialize();
  ros::spin();
}
