cmake_minimum_required(VERSION 3.20)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
include(cmake/target.cmake)
include(cmake/gcc-arm-none-eabi.cmake)
project(F401_COURSE LANGUAGES C ASM)
add_executable(${PROJECT_NAME})
include(cmake/flags.cmake)
include(cmake/components.cmake)
include(cmake/files.cmake)
set(COURSE_SOURCES
 App/Src/imu.c App/Src/config.c App/Src/wire.c App/Src/control.c
 App/Src/motion.c App/Src/balance.c App/Src/mission.c App/Src/app.c
 BSP/Src/board.c BSP/Src/runtime.c)
target_sources(${PROJECT_NAME} PRIVATE ${COURSE_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE App/Inc BSP/Inc)
set_source_files_properties(${COURSE_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wextra;-Werror=implicit-function-declaration;-Werror=return-type")
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--print-memory-usage)
target_link_libraries(${PROJECT_NAME} PRIVATE m)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
 COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
 COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
 COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)
