cmake_minimum_required(VERSION 3.24)
project(Ontwerpen3 C)

set(CMAKE_C_STANDARD 11)

include_directories(/opt/homebrew/Cellar/avr-gcc@9/9.4.0/avr/include/)

add_executable(Ontwerpen3 central_node/main.c lamp_node/main.c window_node/main.c central_node_gas/main.c)
