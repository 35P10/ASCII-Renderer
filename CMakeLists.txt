cmake_minimum_required(VERSION 3.14)
project ("ASCII-Renderer")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

option(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
option(ASSIMP_BUILD_TESTS OFF)
option(ASSIMP_BUILD_SAMPLES OFF)
option(ASSIMP_INSTALL OFF)
option(ASSIMP_NO_EXPORT ON)

include(FetchContent)
message(STATUS "PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/libs CACHE PATH "Base directory for dependencies" FORCE)

FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG master
)

FetchContent_MakeAvailable(assimp)
FetchContent_GetProperties(assimp SOURCE_DIR ASSIMP_SOURCE_DIR)
message(STATUS "ASSIMP_SOURCE_DIR = ${ASSIMP_SOURCE_DIR}")

file(GLOB PROJECT_SOURCES src/*.cpp src/*.h)
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${ASSIMP_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} 
    assimp
)

file(COPY data DESTINATION ${CMAKE_BINARY_DIR})