cmake_minimum_required(VERSION 3.24)
project(Ontwerpen3 C)

set(CMAKE_C_STANDARD 11)

include_directories(/opt/homebrew/Cellar/avr-gcc@9/9.4.0/avr/include/)

add_executable(Ontwerpen3 window_node/Makefile window_node/nrf24L01.c window_node/nrf24spiXM2.c window_node/serialF0.c lamp_node/main.c lamp_node/Makefile lamp_node/nrf24L01.c lamp_node/nrf24L01.h lamp_node/nrf24spiXM2.c lamp_node/nrf24spiXM2.h lamp_node/serialF0.c lamp_node/serialF0.h central_node_gas/main.c central_node_gas/Makefile central_node_gas/nrf24L01.c central_node_gas/nrf24L01.h central_node_gas/nrf24spiXM2.c central_node_gas/nrf24spiXM2.h central_node_gas/serialF0.c central_node_gas/serialF0.h central_node/main.c central_node/Makefile central_node/nrf24L01.c central_node/nrf24L01.h central_node/nrf24spiXM2.c central_node/nrf24spiXM2.h central_node/serialF0.c central_node/serialF0.h)