add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/GeographicLib/include/)
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)