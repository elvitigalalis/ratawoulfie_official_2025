# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

# Pull in the Pico SDK (must be before the project declaration)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(ratawoulfie_official_2025 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

# Set common compile options
add_compile_options(-Wall
    -Wno-format          # Suppress format warnings (e.g., int vs int32_t differences)
    -Wno-unused-function # Some functions are for documentation and aren't called
)
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# Create the executable.
# Adjust the file paths if your source files are in subdirectories.
add_executable(ratawoulfie_official_2025
    src/Main.cpp
    src/API.cpp
    src/Constants/Constants.cpp
    src/Constants/Logger.cpp
    src/Maze_Traversal/Maze/Cell.cpp
    src/Maze_Traversal/Maze/MouseLocal.cpp
    src/Maze_Traversal/Maze/Movement.cpp
    src/Maze_Traversal/AStar.cpp
    src/Maze_Traversal/FrontierBased.cpp
    src/Subsystems/Drive/Drivetrain.cpp
    src/Subsystems/Drive/Motor.cpp
    src/Subsystems/Sensors/Encoder.cpp
)

# Allows files to throw exceptions.
set_source_files_properties(src/Maze_Traversal/Maze/Cell.cpp PROPERTIES COMPILE_FLAGS "-fexceptions")
set_source_files_properties(src/Maze_Traversal/Maze/MouseLocal.cpp PROPERTIES COMPILE_FLAGS "-fexceptions")
set_source_files_properties(src/API.cpp PROPERTIES COMPILE_FLAGS "-fexceptions")


# Generate extra outputs (UF2, etc.) for the Pico
pico_add_extra_outputs(ratawoulfie_official_2025)

# Link with Pico's standard libraries and hardware-specific libraries
target_link_libraries(ratawoulfie_official_2025 pico_stdlib hardware_gpio hardware_pwm hardware_irq)

# Enable USB stdio (disable UART)
pico_enable_stdio_usb(ratawoulfie_official_2025 1)
pico_enable_stdio_uart(ratawoulfie_official_2025 0)