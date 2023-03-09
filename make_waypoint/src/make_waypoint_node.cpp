#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <termios.h>
//#include <fstream>

#include "boost/shared_ptr.hpp"

#include <boost/thread/thread.hpp>

#define POINTNUM            50
#define PI                  3.1415926535897932384626433832795
#define MATH_RAD2DEG        57.2957795f
#define MATH_DEG2RAD        0.0174532f

void clickedPointCallback(const geometry_msgs::PointStamped& cp_msg);
void saveWPCallback(const std_msgs::String& str_msg);
void makeWayPoint(ros::NodeHandle nobj);

float x[POINTNUM];
float y[POINTNUM];
float z[POINTNUM];
float ang[POINTNUM];
int pnum = 0;
char key = 0;

using namespace std;
int fcnt = 1;
//std::string fname;

int main(int argc, char** argv)
{
    int rate_wc = 100;

    ros::init(argc, argv, "make way point node");

    ROS_INFO("Run makeing way point process !!!");

    ros::NodeHandle node_obj;

      
    ros::Subscriber cp_subsciber = node_obj.subscribe("clicked_point", 10, clickedPointCallback);

    ros::Subscriber swp_subsciber = node_obj.subscribe("save_wp", 10, saveWPCallback);

    //boost::thread thread_mwp(makeWayPoint, node_obj);

    //thread_mwp.join(); 

    //ros::spinOnce();


    /*boost::shared_ptr<std_msgs::Empty const> save_wpmess = ros::topic::waitForMessage<std_msgs::Empty>("/save_waypoint", node_obj);

    if(save_wpmess != NULL)
    {
        ROS_INFO("receive save_wpmess!!!");
    }*/

    /*ros::Rate loop_rate(100);

    while(ros::ok())
    {
        boost::shared_ptr<std_msgs::Empty const> save_wpmess = ros::topic::waitForMessage<std_msgs::Empty>("/save_waypoint", node_obj);

        if(save_wpmess != NULL)
        {
            ROS_INFO("receive save_wpmess!!!");
        }
        
        loop_rate.sleep();

        ros::spinOnce();

    }*/
    //ros::Rate loop_rate(10);

    ros::spin();

    ROS_INFO("Exit makeing way point process !!!");

    return 0;
}


void saveWPCallback(const std_msgs::String& str_msg)
{
    std::string recv_str = str_msg.data.c_str();
    ROS_INFO_STREAM("Receive msgs : " << recv_str);

    geometry_msgs::Pose pos;

    if(recv_str.compare("save") == 0)
    {
        ROS_INFO("Save waypoint file.");

        std::string path = ros::package::getPath("make_waypoint");
        ROS_INFO_STREAM("path : " << path.c_str());

        /*char current_absilute_path[1000];
        if(realpath("./", current_absilute_path) == NULL)
        {
            ROS_INFO("Erro : read path...");
        }
        strcat(current_absilute_path, "/");
        ROS_INFO("current absolute path : %s", current_absilute_path);*/

        char fname[100] = {0};
        char buff[50] = {0};

        //sprintf(fname, "wapoint%d.csv", fcnt);
        sprintf(fname, "%s/waypoint%d.csv", path.c_str(), fcnt);
        ROS_INFO("file name : %s", fname);

        FILE *fp = fopen(fname, "w");

        /*fname = "waypoint";
        fname += to_string(fcnt);
        fname += ".csv";*/


        //ofstream wpfile(fname, ios::out | ios::text);

        if(fp != NULL)
        {
            for(int i = 0; i<pnum; i++)
            {
                memset(buff, 0, 50);

                if(i == (pnum - 1))
                {
                    ang[i] = 0.0f;
                    sprintf(buff, "%f,%f,%f,%f\n", x[i], y[i], z[i], ang[i]);
                    fwrite(buff, strlen(buff), 1, fp);
                    //wpfile << x[i] << "," << y[i] << "," << z[i] << "," << w[i] << "\n";
                }
                else
                {
                    sprintf(buff, "%f,%f,%f,%f\n", x[i], y[i], z[i], ang[i]);
                    fwrite(buff, strlen(buff), 1, fp);
                    //wpfile << x[i] << "," << y[i] << "," << z[i] << "," << w[i] << "\n";
                }
            }

            fclose(fp);
        }
        else 
        {
            ROS_INFO("File(%s) open error.", fname);
        }


        memset(fname, 0, 100);
        sprintf(fname, "%s/waypoint%d.yaml", path.c_str(), fcnt);

        fp = fopen(fname, "w");

        if(fp != NULL)
        {

            ROS_INFO("Save waypoint yaml file(%s).", fname);

            memset(buff, 0, 50);
            sprintf(buff, "waypoints:\n");
            fwrite(buff, strlen(buff), 1, fp);

            for(int i = 0; i<pnum; i++)
            {
                memset(buff, 0, 50);
                sprintf(buff, "  - name: name%d\n", i+1);
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "    frame_id: map\n");
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "    pose:\n");
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "      position:\n");
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "        x: %f\n", x[i]);
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "        y: %f\n", y[i]);
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "        z: %f\n", z[i]);
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "      orientation:\n");
                fwrite(buff, strlen(buff), 1, fp);

                pos.orientation = tf::createQuaternionMsgFromYaw(ang[i]);

                memset(buff, 0, 50);
                sprintf(buff, "        x: %f\n", pos.orientation.x);
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "        y: %f\n", pos.orientation.y);
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "        z: %f\n", pos.orientation.z);
                fwrite(buff, strlen(buff), 1, fp);

                memset(buff, 0, 50);
                sprintf(buff, "        w: %lf\n", pos.orientation.w);
                fwrite(buff, strlen(buff), 1, fp);

            }

            memset(buff, 0, 50);
            sprintf(buff, "trajectories:\n");
            fwrite(buff, strlen(buff), 1, fp);

            memset(buff, 0, 50);
            sprintf(buff, "  - name: goto_pos1\n");
            fwrite(buff, strlen(buff), 1, fp);

            memset(buff, 0, 50);
            sprintf(buff, "    waypoints:\n");
            fwrite(buff, strlen(buff), 1, fp);

            memset(buff, 0, 50);
            sprintf(buff, "    - name1\n");
            fwrite(buff, strlen(buff), 1, fp);

            memset(buff, 0, 50);
            sprintf(buff, "    - name2\n");
            fwrite(buff, strlen(buff), 1, fp);

            memset(buff, 0, 50);
            sprintf(buff, "    - name3\n");
            fwrite(buff, strlen(buff), 1, fp);

        }
        else {
                ROS_INFO("File(%s) open error.", fname);
        }
    }

}

void makeWayPoint(ros::NodeHandle nobj)
{
    ros::Rate loop_rate(100);

    ROS_INFO("Run thread !!!");

    //boost::shared_ptr<std_msgs::Empty const> save_wpmess = NULL;

    while(true)
    {
        //save_wpmess = ros::topic::waitForMessage<std_msgs::Empty>("/save_waypoint", nobj, ros::Duration(1));
        //save_wpmess = ros::topic::waitForMessage<std_msgs::Empty>("/save_waypoint", nobj);
        //ros::spin();    
        //ros::spinOnce();

        struct termios oldt;
        struct termios newt;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        newt.c_lflag &= ~ICANON;
        /*newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;*/
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        key = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        if((key == 's') || (key == 'S'))
        {
            ROS_INFO("Save waypoint file.");

            char fname[100] = {0};
            char buff[50] = {0};

            sprintf(fname, "wapoint%d.csv", fcnt);
            ROS_INFO("file name : %s", fname);

            FILE *fp = fopen(fname, "w");

            /*fname = "waypoint";
            fname += to_string(fcnt);
            fname += ".csv";*/


            //ofstream wpfile(fname, ios::out | ios::text);

            if(fp != NULL)
            {
                for(int i = 0; i<pnum; i++)
                {
                    memset(buff, 0, 50);

                    if(i == (pnum -1))
                    {
                        ang[i] = 0.0;
                        sprintf(buff, "%f,%f,%f,%f\n", x[i], y[i], z[i], ang[i]);
                        fwrite(buff, strlen(buff), 1, fp);
                        //wpfile << x[i] << "," << y[i] << "," << z[i] << "," << w[i] << "\n";
                    }
                    else
                    {
                        sprintf(buff, "%f,%f,%f,%f\n", x[i], y[i], z[i], ang[i]);
                        fwrite(buff, strlen(buff), 1, fp);
                        //wpfile << x[i] << "," << y[i] << "," << z[i] << "," << w[i] << "\n";
                    }
                }

                fclose(fp);
            }
            else 
            {
                ROS_INFO("File(%s) open error.", fname);
            }         


        }



        /*if(save_wpmess != NULL)
        {
            ROS_INFO("Receive save_wpmess!!!");
        } else {
            ROS_ERROR_STREAM("Error receive save_wpmess!!!");
        }*/
        
        loop_rate.sleep();

        ros::spinOnce();

    }
}

void clickedPointCallback(const geometry_msgs::PointStamped& cp_msg)
{
    ///ROS_INFO("Receive pose===>"); 

    x[pnum] = cp_msg.point.x;
    y[pnum] = cp_msg.point.y;
    z[pnum] = cp_msg.point.z;

    if(pnum == 0)
    {
        ROS_INFO("num:%d (x:%f y:%f z:%f)", pnum, x[pnum], y[pnum], z[pnum]); 
    } 
    else if(pnum >= 1)
    {
        float diffx = x[pnum] - x[pnum-1];
        float diffy = y[pnum] - y[pnum-1];
        float difflen = sqrt(diffx*diffx + diffy*diffy);

        //float xl = diffx / difflen;
        //float yl = diffy / difflen;

        float oval = atan(diffy / diffx);
        //float oval = atan(diffy / diffx)*MATH_RAD2DEG;
        ang[pnum - 1] = oval;
        
        ROS_INFO("[1]num:%d (x:%f y:%f z:%f w:%f)", pnum-1, x[pnum-1], y[pnum-1], z[pnum-1], ang[pnum-1]); 
        ROS_INFO("[2]num:%d (x:%f y:%f z:%f w:%f)", pnum, x[pnum], y[pnum], z[pnum], ang[pnum]); 
    }

    pnum++;
}