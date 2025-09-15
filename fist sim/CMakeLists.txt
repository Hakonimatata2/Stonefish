cmake_minimum_required(VERSION 3.15)
project(MySim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Bruk den installerte Stonefish’en
find_package(Stonefish REQUIRED CONFIG)
find_package(Freetype REQUIRED)

add_executable(MySim #Navn på program
  src/sc1/main.cpp 
  src/sc1/UnderwaterTestApp.cpp 
  src/sc1/UnderwaterTestManager.cpp
)

target_link_libraries(MySim
  PRIVATE
    Stonefish::Stonefish
    Freetype::Freetype 
)

# (valgfritt) hvis du har ressurser/data i prosjektet ditt:
add_compile_definitions(DATA_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/")
