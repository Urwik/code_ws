#include "arvc_utils.hpp"

using namespace std;

void print_vector(vector<int> _vector)
{
  for (auto &i : _vector)
  {
    std::cout << i << ' ';
  }
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  cout << "Hello World!" << endl;
  std::vector<int> vec = {2, 2, 4, 1, 8, 3, 6, 6, 7, 6, 8, 53, 53};

  vector<int> duplicates = arvc::get_duplicates(vec);


  return 0;
}