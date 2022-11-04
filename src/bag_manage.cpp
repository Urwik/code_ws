// cpp
#include <iostream>
#include <boost/filesystem.hpp>
#include <experimental/filesystem>
#include <typeinfo>
#include <string>
#include <algorithm>

#include <rosbag/bag.h>


int main(int argc, char **argv)
{
  rosbag::Bag bag;

  try{
    bag.open(argv[1]);
  }
  catch(const std::exception& e){
    std::cerr << e.what() << '\n';
  }

  return 0;
}