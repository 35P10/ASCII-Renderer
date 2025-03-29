cmake_minimum_required(VERSION 3.14)
project(ASCII-Renderer CXX)

# Configuracion de C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configuracion de FetchContent para manejar dependencias
include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${PROJECT_SOURCE_DIR}/libs CACHE PATH "Directorio base para dependencias" FORCE)

# Configuracion de Assimp
FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG master
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Opciones para la compilacion de Assimp
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Construir bibliotecas estaticas" FORCE)
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "Deshabilitar pruebas de Assimp" FORCE)
set(ASSIMP_INJECT_DEBUG_POSTFIX OFF CACHE BOOL "Deshabilitar postfijo de depuracion" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "Deshabilitar instalacion de Assimp" FORCE)

# Hacer disponible Assimp
FetchContent_MakeAvailable(assimp)

# Definir archivos fuente
set(SOURCE_FILES 
    src/main.cpp
)

# Crear ejecutable
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Directorios de inclusion
target_include_directories(${PROJECT_NAME} PRIVATE
    include/
    libs/glad/include
    libs/glfw/include
    libs/glm
    libs/imgui
    libs/assimp/include
    ${assimp_SOURCE_DIR}/include
)

# Enlazar bibliotecas
target_link_libraries(${PROJECT_NAME} 
    assimp
)