#pragma once

#if defined(__cpp_lib_filesystem)
  #include <filesystem>
  namespace Filesystem = std::filesystem;
#else
  #include <experimental/filesystem>
  namespace Filesystem = std::experimental::filesystem;
#endif
