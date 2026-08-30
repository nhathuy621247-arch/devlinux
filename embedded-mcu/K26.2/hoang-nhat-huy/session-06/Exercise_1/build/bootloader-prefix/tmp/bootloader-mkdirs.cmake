# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/ToolEsp/esp-idf/components/bootloader/subproject"
  "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader"
  "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader-prefix"
  "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader-prefix/tmp"
  "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader-prefix/src"
  "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Study_ESP/devlinux/embedded-mcu/K26.2/hoang-nhat-huy/session-06/Exercise_1/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
