load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def whisper_dependencies():

  pybind11_bazel_commit = "fd7d88857cca3d7435b06f3ac6abab77cd9983b2"
  maybe(
    http_archive,
    name = "pybind11_bazel",
    strip_prefix = "pybind11_bazel-%s" % pybind11_bazel_commit,
    urls = ["https://github.com/pybind/pybind11_bazel/archive/%s.zip" % pybind11_bazel_commit],
  )

  # We still require the pybind library.
  pybind11_version = "2.11.1"
  maybe(
    http_archive,
    name = "pybind11",
    build_file = "@pybind11_bazel//:pybind11.BUILD",
    strip_prefix = "pybind11-%s" % pybind11_version,
    urls = ["https://github.com/pybind/pybind11/archive/v%s.tar.gz" % pybind11_version],
  )
