# robonode::vision_cv — a real vision node behind the Detector seam. Owns the
# OpenCV dependency; nothing above the seam links or names it.
find_package(OpenCV QUIET COMPONENTS core imgproc)

add_library(robonode_vision_cv INTERFACE)
add_library(robonode::vision_cv ALIAS robonode_vision_cv)
target_include_directories(robonode_vision_cv INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robonode_vision_cv INTERFACE robonode::vision opencv_core opencv_imgproc)
