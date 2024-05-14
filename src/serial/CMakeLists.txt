cmake_minimum_required(VERSION 3.5)
project(serial)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

include_directories(include)

# Manually specify source files or make sure GLOB_RECURSE finds all cpp files
file(GLOB_RECURSE SOURCES "src/*.cc" "src/impl/*.cc") # Adjust pattern to include all cc files

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} Threads::Threads)

# Install rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
