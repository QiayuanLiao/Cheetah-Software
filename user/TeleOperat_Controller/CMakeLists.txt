include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

add_executable(teleoperat_ctrl main.cpp TeleOperat_Controller.cpp)
target_link_libraries(teleoperat_ctrl robot biomimetics dynacore_param_handler qpOASES)
