package(
    default_visibility = ["//visibility:public"],
)

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
  name = "libmini_ros",
  includes = [
    "include/mini_ros",
  ],
  srcs = glob([
    "include/mini_ros/*.h",
    "src/*.cpp",
  ]),
  linkopts = ["-lpthread"],
  linkstatic = False,
  copts = ["-std=c++11"],
)

cc_binary(
  name = "sample_ps",
  includes = [
    "include",
  ],
  srcs = glob([
    "sample/pub_sub/*.cpp",
    "sample/pub_sub/*.h",
  ]),
  deps = [":libmini_ros"],
  copts = ["-std=c++11"],
)

cc_binary(
  name = "sample_sc",
  includes = [
    "include",
  ],
  srcs = glob([
    "sample/srv_call/*.cpp",
    "sample/srv_call/*.h",
  ]),
  deps = [":libmini_ros"],
  copts = ["-std=c++11"],
)
