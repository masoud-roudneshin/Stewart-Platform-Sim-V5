# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-src")
  file(MAKE_DIRECTORY "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-src")
endif()
file(MAKE_DIRECTORY
  "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-build"
  "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-subbuild/googletest-populate-prefix"
  "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-subbuild/googletest-populate-prefix/tmp"
  "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp"
  "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-subbuild/googletest-populate-prefix/src"
  "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/masou/Desktop/Projects/Github_Control/Stewart_Platform_V3/build_tests/_deps/googletest-subbuild/googletest-populate-prefix/src/googletest-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
