// C++
#include <iostream>
#include <boost/filesystem.hpp>
#include <experimental/filesystem>
#include <typeinfo>
#include <string>
#include <algorithm>


int main(int argc, char **argv)
{
  namespace fs = boost::filesystem;
  fs::path mydir("/home/arvc/workSpaces/data/realsense/2022.07.27");
  fs::directory_iterator end_iterator;
  std::vector<fs::directory_entry> files;
  std::string old_name;

  for(auto const& dir_entry : fs::directory_iterator(mydir)){
  
    if (dir_entry.path().extension() == ".pcd")
    {
      old_name = dir_entry.path().filename().c_str();
      old_name.erase(std::remove(old_name.begin(), old_name.end()-3, '.'), old_name.end());
      old_name.append(".pcd");
      fs::rename(dir_entry.path(), dir_entry.path().parent_path()/old_name);
    } 
  }
}
