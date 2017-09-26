
get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
# include(${SELF_DIR}/editdistance-targets.cmake)
get_filename_component(vodigrex_INCLUDE_DIRS "${SELF_DIR}/../../include/vodigrex" ABSOLUTE)
set(vodigrex_LIBRARIES "${SELF_DIR}/libVoronoi.so;${SELF_DIR}/libLineFollower.so;${SELF_DIR}/libEVGthin.so")
