# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v5.1.4/components/bootloader/subproject"
  "D:/Espressif/frameworks/air_sense/build/bootloader"
  "D:/Espressif/frameworks/air_sense/build/bootloader-prefix"
  "D:/Espressif/frameworks/air_sense/build/bootloader-prefix/tmp"
  "D:/Espressif/frameworks/air_sense/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Espressif/frameworks/air_sense/build/bootloader-prefix/src"
  "D:/Espressif/frameworks/air_sense/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Espressif/frameworks/air_sense/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Espressif/frameworks/air_sense/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
