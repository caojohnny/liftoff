cmake_minimum_required(VERSION 3.13)

include(FetchContent)
FetchContent_GetProperties(mpl)

if (NOT mpl_POPULATED)
    find_package(PythonLibs 2.7)
    FetchContent_Declare(
            mpl
            URL https://github.com/lava/matplotlib-cpp/archive/master.zip
            URL_HASH SHA256=5fd300ff30adc220c417d0de672e15d702cf7a3629ec4a91f161378a36f87318
    )
    FetchContent_Populate(mpl)

    file(COPY "${mpl_SOURCE_DIR}/matplotlibcpp.h"
            DESTINATION "${mpl_BINARY_DIR}/include/mpl")

    add_library(mpl INTERFACE IMPORTED)
    target_include_directories(mpl
            INTERFACE "${mpl_BINARY_DIR}/include/"
            INTERFACE "${PYTHON_INCLUDE_DIRS}")
    target_link_libraries(mpl
            INTERFACE "${PYTHON_LIBRARIES}")
endif ()