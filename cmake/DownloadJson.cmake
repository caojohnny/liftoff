cmake_minimum_required(VERSION 3.13)

include(FetchContent)

if (NOT json)
    FetchContent_Declare(
            json
            GIT_REPOSITORY https://github.com/nlohmann/json.git
    )
    FetchContent_Populate(json)

    file(COPY "${json_SOURCE_DIR}/single_include/"
            DESTINATION "${json_BINARY_DIR}/lib")

    add_library(json INTERFACE IMPORTED)
    target_include_directories(json
            INTERFACE "${json_BINARY_DIR}/lib/")
endif ()
