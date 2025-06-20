#include "../include/kobuki_operation/kobuki_operation.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kobuki_operation");
    KobukiOperation ope;
    ope.spin();
    std::cout << "\nexit program\n" << std::endl;
    return 0;
}