# phase-206 W2, final item — the RTOS image that LINKS Cyclone and carries a
# bringup config.
#
# A SEPARATE entry package from `zephyr_entry`, and that is forced rather than
# chosen: the image is resolved by `nros image-facts --for-entry <pkg>`, i.e.
# ENTRY PACKAGE -> IMAGE, one way. Two images cannot share one entry package —
# the lookup would have to pick, and RFC-0085 D4 records that a first-match scan
# over candidates picks the wrong one. Trying to share `zephyr_entry` produced
# exactly that: the build merged this file's `prj-cyclonedds.conf` while
# `image-facts` still answered `demo_bringup:zephyr` (rmw=zenoh), and
# `nros_check_image_agreement` refused the build —
#
#     image demo_bringup:zephyr declares: zenoh
#     Kconfig selected:                   cyclonedds
#
# which is the gate doing its job. `BOARD` names the image.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nros_zephyr_workspace_entry_cpp_cyclonedds)

find_package(nano_ros REQUIRED)

# The same two component packages the zenoh entry links. They are what makes
# this image a real acceptance for W2: the backend is REFERENCED, so
# `--gc-sections` cannot drop it the way it dropped Cyclone from the 212.H.1
# stub app, where the user's bytes reached `session.cpp.obj` and never the ELF.
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../talker_pkg" talker_pkg)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../listener_pkg" listener_pkg)

# BOARD is the board FAMILY token, not the image id — `board_family()` matches
# exact strings ("zephyr", "nuttx", "freertos", ...) and an unknown one falls
# through to the HOST default, which emits `int main(int, char**)` where the
# Zephyr kernel declares `extern int main(void)`. Measured: naming the image
# here produced exactly that conflict at
# `zephyr_cyclonedds_entry_nros_main_generated.cpp:57`. The IMAGE is selected by
# the entry PACKAGE name instead (`nros image-facts --for-entry`), which is what
# this directory is for.
nano_ros_add_executable(zephyr_cyclonedds_entry
    BOARD   zephyr
    BRINGUP "${CMAKE_CURRENT_SOURCE_DIR}/../demo_bringup"
    TYPED
    DEPLOY  zephyr)
