add_library(trslam SHARED
        viewer.cpp
        )

target_link_libraries(trslam
        ${THIRD_PARTY_LIBS})
