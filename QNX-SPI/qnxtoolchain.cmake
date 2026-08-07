# Minimal QNX aarch64le cross toolchain (no Qt dependency).
# Uses QNX_HOST/QNX_TARGET from qnxsdp-env.sh if sourced, otherwise defaults
# to a qnx800 checkout next to the repo.
set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(DEFINED ENV{QNX_HOST})
    set(QNX_HOST "$ENV{QNX_HOST}")
else()
    set(QNX_HOST "${CMAKE_CURRENT_LIST_DIR}/../../../qnx800/host/linux/x86_64")
endif()
if(DEFINED ENV{QNX_TARGET})
    set(QNX_TARGET "$ENV{QNX_TARGET}")
else()
    set(QNX_TARGET "${CMAKE_CURRENT_LIST_DIR}/../../../qnx800/target/qnx")
endif()

set(CMAKE_C_COMPILER "${QNX_HOST}/usr/bin/qcc")
set(CMAKE_CXX_COMPILER "${QNX_HOST}/usr/bin/q++")
set(CMAKE_ASM_COMPILER "${QNX_HOST}/usr/bin/qcc")
set(CMAKE_C_FLAGS "-Vgcc_ntoaarch64le")
set(CMAKE_CXX_FLAGS "-Vgcc_ntoaarch64le")
set(CMAKE_FIND_ROOT_PATH "${QNX_TARGET}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
