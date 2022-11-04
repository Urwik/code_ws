// cpp
#include <iostream>
#include <stdlib.h>
#include <filesystem>

// PCL

namespace fs = std::filesystem;

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////


void rename(fs::path data_path, std::string target_ext, std::string in_name, std::string out_name){
    
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
  std::string ROOT_DIR, OUTPUT_DIR;
  
  fs::path data_path("/media/arvc/data/experiments/ouster/simulated/73_pcds/raw/ply/");

  rename(data_path, ".ply", "foo", "pc_");
  
  return 0;
}



