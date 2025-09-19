add_executable(3R 3R.cpp)
target_link_libraries(3R kc::kc)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/templates/3R.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/3R.hpp)
target_include_directories(3R PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_executable(Stanford Stanford.cpp)
target_link_libraries(Stanford kc::kc)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/templates/Stanford.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/Stanford.hpp)
target_include_directories(Stanford PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_executable(KUKA KUKA.cpp)
target_link_libraries(KUKA kc::kc)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/templates/KUKA.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/KUKA.hpp)
target_include_directories(KUKA PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
