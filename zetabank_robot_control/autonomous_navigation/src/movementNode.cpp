#include "autonomous_navigation/zetabotControl.h"
#include "autonomous_navigation/zetabotStatus.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ZetabotNode");

    ZetabotControl mc;
    ZetabotStatus ms;

    boost::thread* thr = new boost::thread(boost::bind(&ZetabotControl::doAutoNav, &mc));
    boost::thread* thr2 = new boost::thread(boost::bind(&ZetabotStatus::publishZetabotStatus, &ms));
   
    thr->join();     
    thr2->join();        
    
}
