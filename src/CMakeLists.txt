add_library(myslam SHARED
        camera.cpp
        )

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
