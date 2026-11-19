# Box3D as a subproject, for the single-threaded flavours (standard, scalar).
#
# Box3D's top-level CMakeLists.txt adds `-pthread -s USE_PTHREADS=1` to every Emscripten build where it is the top-level
# project (Box3D #201, from 33626332). A single-threaded flavour built that way compiles the library with the atomics
# feature and the binding (glue.cpp, flat.cpp) without it, and LTO does not inline a callee whose target features the
# caller lacks: every body getter became a call of its own (box3d-efa v0.4.0 to v0.4.8, reads 7 to 32 % slower than
# v0.3.0). Under this project PROJECT_IS_TOP_LEVEL is false in Box3D's, so that block is skipped, and so are its samples,
# tests and the options it keeps to the top level (scripts/build.sh passes BOX3D_VALIDATE and BOX3D_PROFILE OFF).
# The threaded flavour (deluxe) still configures Box3D directly.
cmake_minimum_required(VERSION 3.22)
project(box3d_efa C CXX)

if (NOT BOX3D_SRC)
	message(FATAL_ERROR "BOX3D_SRC must name Box3D's source (deps/box3d)")
endif()

add_subdirectory("${BOX3D_SRC}" box3d)
