#include <cstdlib>
#include <iostream>

#include "Analysis.hpp"

int main(int Argc, char **Argv) {
  if (Argc < 2) {
    std::cout << "No bitcode file supplied!" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string Filename(Argv[1]);
  Analysis A(Filename);

  std::cout << "Analysis done." << std::endl;
  return EXIT_SUCCESS;
}
