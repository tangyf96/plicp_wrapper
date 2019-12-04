#include <plicp/plicp_node.hpp>

int main(int argc, char** argv)
{
    // ros
    ros::init(argc, argv, "plicp");
    ros::NodeHandle nh("~");
    icp_tools::Plicp plicp_handle(nh);
    
    std::ofstream  ofile;
    std::string filename = "/home/yifan/data.txt";
    // 打开保存的文件
    ofile.open(filename.c_str());
    if (!ofile)
    {
        std::cout << "Fail to open file!" << std::endl;
        return 0;
    }
    else
        ofile << "delta_x , delta_y, delta_theta, timeStamp" << std::endl;
    
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if (plicp_handle.output_.valid)
        {
            // std::cout << "save data" << std::endl;
            ofile << plicp_handle.output_.x[0] << ", " 
                        << plicp_handle.output_.x[1] << ", "
                        << plicp_handle.output_.x[2] << ", "
                        <<  plicp_handle.curTime << std::endl;
        }
    }
    return 0;
}