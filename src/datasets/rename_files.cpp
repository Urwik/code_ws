/**
 * @file rename_files.cpp
 * @author Fran Soler (f.soler@umh.es)
 * @brief Code that enables to change all filenames in a folder specifying the
 * extension of files to change, and a new name set as prefix of the older name.
 * @version 0.1
 * @date 2023-01-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
// C++
#include <iostream>
#include <stdlib.h>
#include <filesystem>

namespace fs = std::filesystem;

void rename(fs::path data_path, std::string target_ext, std::string out_name){
    
  for (const auto & entry : fs::directory_iterator(data_path))
  {
    std::string file_ext = entry.path().filename().extension();
    std::string old_name = entry.path().stem();

    if (file_ext == target_ext){
      fs::rename(entry.path(), data_path.c_str() + out_name + old_name + file_ext);
    }
  }
}


int main(int argc, char **argv)
{
  // fs::path current_path = fs::current_path();
  // rename(current_path, ".ply", "pc_");

  fs::path current_dir = fs::current_path();
  fs::directory_iterator end_iterator;
  std::string old_name;

  for(auto const& dir_entry : fs::directory_iterator(current_dir)){
  
    if (dir_entry.path().extension() == ".pcd")
    {
      old_name = dir_entry.path().filename().c_str();
      old_name.erase(std::remove(old_name.begin(), old_name.end()-3, '.'), old_name.end());
      old_name.append(".pcd");
      fs::rename(dir_entry.path(), dir_entry.path().parent_path()/old_name);
    } 
  }

  fs::path data_path("/media/arvc/data/experiments/ouster/simulated/73_pcds/raw/ply/");
  rename(data_path, ".ply", "pc_");

  return 0;
}