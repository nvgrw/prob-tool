#include <cstdlib>
#include <iostream>

#include <llvm/Bitcode/BitcodeReader.h>
#include <llvm/Support/MemoryBuffer.h>

#include "Analysis.hpp"

using namespace llvm;

int main(int Argc, char **Argv) {
  if (Argc < 2) {
    std::cerr << "No bitcode file supplied!" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string Filename(Argv[1]);
  auto BufferOrError = MemoryBuffer::getFile(Filename);
  if (std::error_code Ec = BufferOrError.getError()) {
    report_fatal_error(errorCodeToError(Ec));
  }
  MemoryBufferRef Buffer(*BufferOrError.get());
  LLVMContext Context;
  auto ModuleOrError = parseBitcodeFile(Buffer, Context);
  if (Error Error = ModuleOrError.takeError()) {
    report_fatal_error(std::move(Error));
  }

  pt::Analysis A(ModuleOrError.get());
  A.dumpLabeled();
  std::cout << A.run() << std::endl;
  return EXIT_SUCCESS;
}
