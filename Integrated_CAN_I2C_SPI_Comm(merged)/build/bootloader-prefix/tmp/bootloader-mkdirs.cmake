# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.2.2/components/bootloader/subproject"
  "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader"
  "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader-prefix"
  "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader-prefix/tmp"
  "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader-prefix/src"
  "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/prath/workspace/Integrated_CAN_I2C_SPI_Comm(merged)/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
