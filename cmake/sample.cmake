# sample.cmake

include_directories(
  include
  sample/pub_sub/
  sample/srv_call/
  sample/threadhandler/
)

add_executable(sample_ps sample/pub_sub/main.cpp)
add_executable(sample_sc sample/srv_call/main.cpp)
add_executable(sample_th sample/threadhandler/main.cpp)

set(SAMPLE_DEPENDS miniros)
target_link_libraries(sample_sc ${SAMPLE_DEPENDS})
target_link_libraries(sample_ps ${SAMPLE_DEPENDS})
target_link_libraries(sample_th ${SAMPLE_DEPENDS})
