cmake_minimum_required(VERSION 3.16)
project(retina-spectrum)
set(CMAKE_CXX_STANDARD 17)

# ── Dependencies ──────────────────────────────────────────────────────────────

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3F REQUIRED fftw3f)

# cpp-httplib — header-only, fetched at configure time
include(FetchContent)
FetchContent_Declare(httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG        v0.15.3)
FetchContent_MakeAvailable(httplib)

# ── Binary ────────────────────────────────────────────────────────────────────

add_executable(retina-spectrum
    src/main.cpp
    src/sdr.cpp
    src/dsp.cpp)

target_include_directories(retina-spectrum PRIVATE
    src/                        # config.h, sdr.h, dsp.h
    /usr/local/include          # sdrplay_api.h (installed by SDRplay API package)
    ${FFTW3F_INCLUDE_DIRS})

target_link_directories(retina-spectrum PRIVATE
    /usr/local/lib)             # libsdrplay_api.so.3.15

target_link_libraries(retina-spectrum PRIVATE
    ${FFTW3F_LIBRARIES}         # -lfftw3f
    sdrplay_api                 # -lsdrplay_api
    pthread
    httplib::httplib)

# ── Install web assets alongside binary ───────────────────────────────────────

install(TARGETS retina-spectrum DESTINATION bin)
install(DIRECTORY web/ DESTINATION /web)
