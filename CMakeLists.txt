cmake_minimum_required(VERSION 3.16)

# Activar C++ y CUDA
project(Verlet-Multithread LANGUAGES CXX CUDA)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

include(FetchContent)

# ---- SFML ----
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

# ---- Archivos fuente ----
file(GLOB_RECURSE source_files
    "src/*.cpp"
    "src/*.cu"      # ← AGREGADO
)

add_executable(${PROJECT_NAME} ${source_files})

# ---- Includes ----
target_include_directories(${PROJECT_NAME} PRIVATE "src" "engine")

# ---- Librerías ----
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics)

# ---- Estándares ----
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# ---- Flags recomendados para CUDA ----
set_target_properties(${PROJECT_NAME} PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_STANDARD 17
)

# ---- Copia recursos ----
add_custom_command(
    TARGET ${PROJECT_NAME}
    COMMENT "Copy Res directory"
    PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/res
        $<TARGET_FILE_DIR:${PROJECT_NAME}>/res
)

# ---- Windows OpenAL ----
if(WIN32)
    add_custom_command(
        TARGET ${PROJECT_NAME}
        COMMENT "Copy OpenAL DLL"
        PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy
            ${SFML_SOURCE_DIR}/extlibs/bin/$<IF:$<EQUAL:${CMAKE_SIZEOF_VOID_P},8>,x64,x86>/openal32.dll
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
endif()
