add_library(trslam SHARED
        camera.cpp
        data.cpp
        plot.cpp
        )

target_link_libraries(trslam
        ${THIRD_PARTY_LIBS})
