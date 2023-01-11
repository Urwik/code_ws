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

  fs::path data_path("/media/arvc/data/experiments/ouster/simulated/73_pcds/raw/ply/");
  rename(data_path, ".ply", "pc_");

  return 0;
}