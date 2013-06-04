// General include files
#include <stdio.h>
#include <math.h>
#include <vector>
// Player related files
#include <libplayerc++/playerc++.h>
#include "cmdline_parsing.h"
#include "common_functions.h"
// Local library files
#include "final.h"

using namespace PlayerCc;
using namespace std;

// This defines when we are "close enough" to the goal point
#define DIST_EPS    0.05

int main(int argc, char **argv)
{
    // Calls the command line parser
    parse_args(argc, argv);
    
    // Define the goal
    int goal_x = 0;
    int goal_y = 1;
    Point goal = Point(goal_x, goal_y);
    bool not_done = true;
    double dmin;
    
    try
    {
        // Initialize connection to player
        PlayerClient robot(gHostname, gPort);
        Position2dProxy pp(&robot, gIndex);
        LaserProxy lp(&robot, gIndex);
        int num_attempts = 20;
        if(!check_robot_connection(robot, pp, num_attempts))
        {
            exit(-2);
        }
        // Initialize speed and turnrate
        double speed = 0.0;
        double turnrate = 0.0;
        
        // Start main processing loop
        while(not_done)
        {
            // Read from the proxies
            robot.Read();
            Pose robot_pose = Pose(pp.GetXPos(), pp.GetYPos(), pp.GetYaw());
            
            // Query the laserproxy to gather the laser scanner data
            unsigned int n = lp.GetCount();
            vector<double> range_data(n);
            vector<double> bearing_data(n);
            for(uint i=0; i<n; i++) 
            {
                range_data[i] = lp.GetRange(i);
                bearing_data[i] = lp.GetBearing(i);
            }  

            // Assign the minimum distance the robot has come to the goal
            dmin = goal.distance_to(robot_pose.x, robot_pose.y);
            
            // Check to make sure the robot will not hit anything
            //for (uint i=0; i<n; i++)
           // {
                //if (lp.GetRange(i) < MIN_WALL_DIST)
               // {
                 //   printf("Impact detected at bearing %f! Abort!\n", 
                   //        lp.GetBearing(i));
                    //not_done = false;
               // }
           // }
            
            // Update movement
            if (not_done)
            {
                not_done = figure_out_movement(&speed, &turnrate, range_data, 
                                               bearing_data, n, goal_x, goal_y,
                                               robot_pose.x, robot_pose.y, 
                                               robot_pose.theta, dmin);
            } 
            else 
            {
                speed = 0.0;
                turnrate = 0.0;
            }
            pp.SetSpeed(speed, turnrate);
        }
    }
    catch(PlayerError e)
    {
        write_error_details_and_exit(argv[0], e);
    }
}
