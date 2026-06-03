cmake_minimum_required(VERSION 3.25)

project(wave_sim LANGUAGES C CXX)

# --- MinGW/GCC: avoid COFF relocation overflow from giant DWARF (Eigen) ---
if (MINGW)
    # allow very large object files
    add_compile_options(-Wa,-mbig-obj)
endif()

# Make Debug symbols lightweight (line tables only) to keep .debug_* small
set(CMAKE_C_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG} -g1 -fno-var-tracking-assignments")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g1 -fno-var-tracking-assignments")

# (Optional) prefer RelWithDebInfo for day-to-day builds
# set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO} -g1")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g1")
if (MINGW)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO} -Wa,-mbig-obj")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -Wa,-mbig-obj")
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------- GLFW -------------------------------------------------
add_subdirectory(external/glfw)

# -------- GLAD -------------------------------------------------
add_library(glad external/glad/src/glad.c
        src/SimCore.hpp)

# GLAD's generated glad.c includes "glad.h" (no subfolder),
# so we must add BOTH the root include and the /glad subfolder.
target_include_directories(glad PUBLIC
        external/glad/include
        external/glad/include/glad
)


# NEW (add both dirs – safest):
target_include_directories(glad PUBLIC external/glad/include)
# -------- Dear ImGui (docking branch via FetchContent) --------
include(FetchContent)

FetchContent_Declare(
        imgui_src
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG docking
        GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui_src)  # <- replaces the GetProperties/Populate combo
set(IMGUI_DIR ${imgui_src_SOURCE_DIR})

add_library(imgui
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Make sure the backends can find GLFW headers
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    external/glfw/include
)

# Let the OpenGL3 backend know we're using GLAD (safer across setups)
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)


# -------- GLM --------------------------------------------------
add_library(glm INTERFACE)
target_include_directories(glm INTERFACE external/glm)

# -------- Main executable --------------------------------------
add_executable(wave_sim src/SimCore.cpp)
target_link_libraries(wave_sim PRIVATE glfw glad imgui glm)

# -------- Eigen3 (header-only) --------------------------------
find_package(Eigen3 3.3 REQUIRED)
target_link_libraries(wave_sim PRIVATE Eigen3::Eigen)

# --- PUMA forward analysis (single-file; links GL/GLFW/ImGui) ---
add_executable(PUMA_Robot_Forward_Analysis
        src/PUMA_Robot_Forward_Analysis.cpp
)
target_link_libraries(PUMA_Robot_Forward_Analysis
        PRIVATE Eigen3::Eigen glad glfw imgui
)
# Ensure ImGui OpenGL3 backend knows we use GLAD
target_compile_definitions(PUMA_Robot_Forward_Analysis PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)

# Trim Eigen debug checks a bit (optional)
target_compile_definitions(PUMA_Robot_Forward_Analysis PRIVATE EIGEN_NO_DEBUG)

# If Debug still fails to link on MinGW, compile this heavy TU with no debug info.
# (Has no effect in non-Debug configs.)
if (MINGW)
    set_source_files_properties(src/PUMA_Robot_Forward_Analysis.cpp
            PROPERTIES COMPILE_FLAGS "-g0 -Wa,-mbig-obj")
endif()


# -------- Platform-specific OpenGL & friends -------------------
if (WIN32)
    target_link_libraries(wave_sim PRIVATE opengl32)
    target_link_libraries(PUMA_Robot_Forward_Analysis PRIVATE opengl32)
elseif (APPLE)
    find_library(OPENGL_LIBRARY OpenGL REQUIRED)
    target_link_libraries(wave_sim PRIVATE ${OPENGL_LIBRARY})
    target_link_libraries(PUMA_Robot_Forward_Analysis PRIVATE ${OPENGL_LIBRARY})
else() # Linux
    find_package(OpenGL REQUIRED)
    target_link_libraries(wave_sim PRIVATE OpenGL::GL dl pthread)
    target_link_libraries(PUMA_Robot_Forward_Analysis PRIVATE OpenGL::GL dl pthread)
endif()



# Optional: copy shaders/etc. next to binary on build
