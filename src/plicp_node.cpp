#include <plicp/plicp_node.hpp>

int main(int argc, char** argv)
{
    std::string addr = "udpm://239.255.76.67:7077"; // 电脑ip修改需要更改后三位为ip
    lcm::LCM lcm(addr, true);
    if(!lcm.good())
        return 1;
    
    icp_tools::Plicp plicp_handle(&lcm);
    while(true)
    {
        
    }
    return 0;
}