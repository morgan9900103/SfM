add_library(sfm SHARED
    sfm.cpp
)

target_link_libraries(sfm
    ${THIRD_PARTY_LIBS}
    ${OpenCV_LIBS}
    ${CSPARSE_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)
