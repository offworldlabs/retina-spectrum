cmake_minimum_required(VERSION 3.16)
project(retina-spectrum)
set(CMAKE_CXX_STANDARD 17)

# ── Dependencies ──────────────────────────────────────────────────────────────

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3F REQUIRED IMPORTED_TARGET fftw3f)

# cpp-httplib — header-only, fetched at configure time
include(FetchContent)
FetchContent_Declare(httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG        v0.15.3)
FetchContent_MakeAvailable(httplib)

# ── Binary ────────────────────────────────────────────────────────────────────

option(MOCK_ONLY "Build without SDRplay hardware dependency (dev/CI builds)" OFF)

add_executable(retina-spectrum
    src/main.cpp
    src/sdr.cpp
    src/dsp.cpp)

target_include_directories(retina-spectrum PRIVATE
    src/)                       # config.h, sdr.h, dsp.h

target_link_libraries(retina-spectrum PRIVATE
    PkgConfig::FFTW3F           # fftw3f (includes lib path via imported target)
    pthread
    httplib::httplib)

if(MOCK_ONLY)
    target_compile_definitions(retina-spectrum PRIVATE MOCK_ONLY)
    message(STATUS "MOCK_ONLY build — SDRplay API not required")
else()
    target_include_directories(retina-spectrum PRIVATE
        /usr/local/include)     # sdrplay_api.h (installed by SDRplay API package)
    target_link_directories(retina-spectrum PRIVATE
        /usr/local/lib)         # libsdrplay_api.so.3.15
    target_link_libraries(retina-spectrum PRIVATE
        sdrplay_api)            # -lsdrplay_api
endif()

# ── Install web assets alongside binary ───────────────────────────────────────

install(TARGETS retina-spectrum DESTINATION bin)
install(DIRECTORY web/ DESTINATION /web)
