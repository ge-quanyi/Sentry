AUX_SOURCE_DIRECTORY(armour ARMOUR_DIR)
AUX_SOURCE_DIRECTORY(IndusCamera CAMERA_DIR)
AUX_SOURCE_DIRECTORY(serial SERIAL_DIR)
AUX_SOURCE_DIRECTORY(classifier CLASSIFIER_DIR)
add_library(src SHARED ${ARMOUR_DIR}
                ${CAMERA_DIR}
                ${SERIAL_DIR}
                ${CLASSIFIER_DIR})

target_link_libraries(src /usr/lib/libgxiapi.so
                          ${OpenCV_LIBS})