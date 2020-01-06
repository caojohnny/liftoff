cmake_minimum_required(VERSION 3.13)

include(FetchContent)
FetchContent_GetProperties(mpl)

if (NOT mpl_POPULATED)
    find_package(Python2 COMPONENTS Development NumPy)
    FetchContent_Declare(
            mpl
            URL https://github.com/lava/matplotlib-cpp/archive/master.zip
            URL_HASH SHA256=40a302a3a4f4d318fdbdfb592bf8989750b6b54338d5bd3cefa8fa3e452c248a
    )
    FetchContent_Populate(mpl)

    file(COPY "${mpl_SOURCE_DIR}/matplotlibcpp.h"
            DESTINATION "${mpl_BINARY_DIR}/include/mpl")

    add_library(mpl INTERFACE IMPORTED)
    target_include_directories(mpl
            INTERFACE "${mpl_BINARY_DIR}/include/"
            INTERFACE "${PYTHON_INCLUDE_DIRS}"
            INTERFACE "${Python2_NumPy_INCLUDE_DIRS}")
    target_link_libraries(mpl
            INTERFACE Python2::Python
            INTERFACE Python2::NumPy)
endif ()