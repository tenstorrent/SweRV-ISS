load("@pybind11_bazel//:python_configure.bzl", "python_configure")

def whisper_dependencies2():
  python_configure(name = "local_config_python")
