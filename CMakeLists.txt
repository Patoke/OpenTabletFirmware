cmake_minimum_required(VERSION 3.17.0)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake")

project(OpenTabletFirmware)

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(USE_LTO "Enable LTO" ON)

if(USE_LTO)
	include(CheckIPOSupported)
	check_ipo_supported(RESULT supported OUTPUT error)
	set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if (isMultiConfig)
	set(CMAKE_CROSS_CONFIGS "Release;Debug")
	set(CMAKE_DEFAULT_BUILD_TYPE "Release")
	set(CMAKE_DEFAULT_CONFIGS "Release;Debug")
endif ()

option (FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." TRUE)
if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_options (-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_options (-fcolor-diagnostics)
    endif ()
endif ()

if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 12.0)
add_link_options(
	-Wl,--no-warn-rwx-segments
)
endif()

add_compile_definitions($<$<COMPILE_LANGUAGE:CXX>:register=>)

add_definitions(
	-DCMAKE_EXPORT_COMPILE_COMMANDS=1
	-DGD32F3x0
	-DGD32F350
)

add_compile_options(
	-mcpu=cortex-m4
	-mfloat-abi=hard
	-mfpu=fpv4-sp-d16

	-ffunction-sections
	-fdata-sections
	-fstack-usage
	-ffast-math

	$<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
	
	"$<$<CONFIG:Debug>:-Og;-DDEBUG;-g;-funwind-tables>"
	"$<$<CONFIG:Release>:-O2;-DNDEBUG>"
	"$<$<CONFIG:MinSizeRel>:-Os;-DNDEBUG>"
	"$<$<CONFIG:RelWithDebInfo>:-O2;-g;-DNDEBUG>"
)

set(LINKER_SCRIPT ${PROJECT_SOURCE_DIR}/gd32f3x0.ld)

add_link_options(
	-mcpu=cortex-m4
	-mfloat-abi=hard
	-mfpu=fpv4-sp-d16

	-specs=nosys.specs
	-static
	-u _printf_float
	-Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	-Wl,--gc-sections
	-T${LINKER_SCRIPT}
	-Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map
)

include_directories(main/inc 
	Firmware/CMSIS/
	Firmware/CMSIS/GD/GD32F3x0/Include

	Firmware/GD32F3x0_standard_peripheral/Include

	Firmware/GD32F3x0_usbfs_library/driver/Include 
	
	#Firmware/GD32F3x0_usbfs_library/device/class/audio/Include
	#Firmware/GD32F3x0_usbfs_library/device/class/cdc/Include
	#Firmware/GD32F3x0_usbfs_library/device/class/dfu/Include
	Firmware/GD32F3x0_usbfs_library/device/class/hid/Include
	#Firmware/GD32F3x0_usbfs_library/device/class/iap/Include
	#Firmware/GD32F3x0_usbfs_library/device/class/msc/Include
	#Firmware/GD32F3x0_usbfs_library/device/class/printer/Include
	Firmware/GD32F3x0_usbfs_library/device/core/Include 

	Firmware/GD32F3x0_usbfs_library/ustd/common
	Firmware/GD32F3x0_usbfs_library/ustd/class/cdc 
	Firmware/GD32F3x0_usbfs_library/ustd/class/hid
	Firmware/GD32F3x0_usbfs_library/ustd/class/msc

	Firmware/GD32F3x0_usbfs_library/host/core/Include
	Firmware/GD32F3x0_usbfs_library/host/class/hid/Include
	Firmware/GD32F3x0_usbfs_library/host/class/msc/Include

	Utilities
)

file(GLOB_RECURSE USER_SOURCES "main/src/*.*")
file(GLOB_RECURSE STD_PERIPH_SOURCES "Firmware/GD32F3x0_standard_peripheral/Source/*.c")
file(GLOB_RECURSE CMSIS_SOURCES "Firmware/CMSIS/*.c" "Firmware/CMSIS/*.cpp")

set(USB_SOURCES
    Firmware/GD32F3x0_usbfs_library/driver/Source/drv_usb_core.c
    Firmware/GD32F3x0_usbfs_library/driver/Source/drv_usb_dev.c
    Firmware/GD32F3x0_usbfs_library/driver/Source/drv_usbd_int.c
    Firmware/GD32F3x0_usbfs_library/device/core/Source/usbd_core.c
    Firmware/GD32F3x0_usbfs_library/device/core/Source/usbd_enum.c
    Firmware/GD32F3x0_usbfs_library/device/core/Source/usbd_transc.c

    Firmware/GD32F3x0_usbfs_library/device/class/hid/Source/standard_hid_core.c
)

set(UTIL_SOURCES
	Utilities/gd32f350r_eval.c
)

add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${STD_PERIPH_SOURCES} ${CMSIS_SOURCES} ${USB_SOURCES} ${UTIL_SOURCES} ${LINKER_SCRIPT})

if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
    # 1. Ask GCC for the full path to stddef.h (Compiler Internal Headers)
    execute_process(
        COMMAND "${CMAKE_C_COMPILER}" -print-file-name=include/stddef.h
        OUTPUT_VARIABLE GCC_STDDEF_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    get_filename_component(GCC_INTERNAL_PATH "${GCC_STDDEF_PATH}" DIRECTORY)
    file(TO_CMAKE_PATH "${GCC_INTERNAL_PATH}" GCC_INTERNAL_PATH)

    # 2. Ask GCC for the standard library path (stdlib.h)
    #    (Usually found via -print-file-name=libc.a, then go up to include)
    execute_process(
        COMMAND "${CMAKE_C_COMPILER}" -print-file-name=libc.a
        OUTPUT_VARIABLE GCC_LIBC_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    get_filename_component(GCC_LIBC_DIR "${GCC_LIBC_PATH}" DIRECTORY)
    # The include folder is typically adjacent to the lib folder (../include)
    get_filename_component(GCC_STD_INCLUDE_PATH "${GCC_LIBC_DIR}/../include" ABSOLUTE)
    file(TO_CMAKE_PATH "${GCC_STD_INCLUDE_PATH}" GCC_STD_INCLUDE_PATH)

    # 3. Create the .clangd content
    #    We force both paths using -isystem to satisfy Clangd
    set(CLANGD_CONTENT 
"CompileFlags:
  Add:
    - \"--target=arm-none-eabi\"
    - \"-isystem${GCC_STD_INCLUDE_PATH}\"
    - \"-isystem${GCC_INTERNAL_PATH}\"
  Remove:
    - \"-fno-fat-lto-objects\"
    - \"-fstack-usage\"
")

    # 4. Write the file to the project root
    file(WRITE "${CMAKE_SOURCE_DIR}/.clangd" "${CLANGD_CONTENT}")
    
    message(STATUS "Auto-generated .clangd configuration with internal paths.")
endif()

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")