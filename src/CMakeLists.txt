add_executable(PxRecognize main.cpp hdl_grabber.cpp vlp_grabber.cpp)
target_link_libraries(PxRecognize ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${PCAP_LIBRARY})
