add_library(simplemotion rs232.c simplemotion.c busdevice.c sm_consts.c)

target_include_directories (simplemotion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})