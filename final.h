// This file contains a struct that holds laser scan data

// General include files
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <algorithm>

#define PI 3.14159265358979323846
#define MIN_TURN_RATE 0.3
#define SPEED_EPS 0.01
#define ANGLE_EPS 0.05
#define MIN_WALL_DIST 0.5
#define RANGE_DIFF 0.5
#define BEAR_DIFF 0.01
#define MAX_SPEED 0.3
#define DIST_EPS 0.05

// Include namespaces for convenience in code writing
using namespace std;

// declare state globals, for navigation
int state = 0;
bool first_pass_bf = true;
bool boundary_mode = false;
bool first_pass = true;
bool second_pass = false;
vector<double> node_x;
vector<double> node_y;
vector<double> range_x;
vector<double> range_y;
vector<double> range_nodes;
vector<vector<double> > nodes;
vector<vector<double> > nodes1;
vector<vector<double> > nodes2;
double node_closest_x;
double node_closest_y;
double goal_theta = 0;
double range_closest;
double robot_start_x;
double robot_start_y;
uint i;
// declare state constants
enum robot_states {
mtg,
bf
};

//Prototypes
int go_to_point(double goal_x, double goal_y,
                double robot_x, double robot_y, double robot_theta,
                double* r_dot, double* theta_dot);

vector<vector<double> > sense(vector<double> range_data, vector<double> bearing_data,
          double goal_x, double goal_y, double* speed, double* turnrate, double robot_x,
          double robot_y, double robot_theta);

bool figure_out_movement(double * speed, double * turnrate,
    vector<double> range_data, vector<double> bearing_data, unsigned int n,
    double goal_x, double goal_y, double robot_x, double robot_y, 
    double robot_theta, double dmin);


// This object stores the (x,y,theta) pose of a robot
struct Pose
{
    double x;
    double y;
    double theta;
    Pose() : x(0), y(0), theta(0)  {}
    Pose(const double x, const double y, const double theta) : x(x), y(y), theta(theta) {}
};
// This function prints out a Pose as x, y, theta
ostream& operator<<(ostream& os, const Pose& p)
{
    os << p.x << ", " << p.y << ", " << p.theta;
    return os;
}
// This function reads in a Pose given as x, y, theta
istream& operator>>(istream& s, Pose& p)
{
    double x = 0, y = 0, theta = 0;   // initialize components of vector to read in.
    char ch = 0;                    // characters read in from string

    if (!s)     {return s;}     // stream invalid, just return fail state

    // Read in each component and return if invalid syntax occurs
    s >> x >> ch;
    if (ch != ',') {s.clear(ios_base::failbit); return s;}  // no ',' between num
    s >> y >> ch;
    if (ch != ',') {s.clear(ios_base::failbit); return s;}  // no ',' between num
    s >> theta >> ch;

    // Everything valid, create Color
    p = Pose(x, y, theta);

    return s;
}

// This object stores the (x,y) coordinates of a point
struct Point
{
    double x;
    double y;
    Point() : x(0), y(0)  {}
    Point(const double x, const double y) : x(x), y(y)  {}
    double distance_to(const Point p2) const
    {
        return distance_to(p2.x, p2.y);
    }
    double distance_to(const double x2, const double y2) const
    {
        double dx = x - x2;
        double dy = y - y2;
        return sqrt(dx * dx + dy * dy);
    }
    double distance_to(const Pose p) const
    {
        return distance_to(p.x, p.y);
    }
    // This function returns the angle between two points.
    double angle_to(const double x2, const double y2) const
    {
        return atan((y - y2)/(x - x2));
    }
    // This function returns the angle between two points. When theta2 is given,
    // it interprets the point given as the location of the robot and gives the
    // relative angle toward *this.
    double angle_to(const double x2, const double y2, const double theta2) const
    {
        double dtheta = angle_to(x2, y2) - theta2;
        dtheta -= PI * (x2 > x);  // Correct on left half of plane
        dtheta += ((dtheta < -PI) - (dtheta > PI)) * 2 * PI; // Bound on [-PI, PI]
        return dtheta;
    }
    double angle_to(const Pose p) const
    {
        return angle_to(p.x, p.y, p.theta);
    }
};
// This function prints out a Point as [x, y]
ostream& operator<<(ostream& os, const Point& p)
{
    os << "[" << p.x << ", " << p.y << "]";
    return os;
}
// This function reads in a Point given as 2 doubles separated by whitespace
// It is assumed that there is a new line separator at the end
istream& operator>>(istream& s, Point& p)
{
    // initialize components of Point to read in
    double x = 0, y = 0;
    if (!s)     {return s;}     // stream invalid, just return fail state
    s >> x >> y;                // Get data from stream
    p = Point(x, y);            // Everything valid, create DoublePoint
    return s;                   // Return stream
}


/* This function implements a go to function based on inputs of where the robot
   destination is and the current pose. It returns the direction and speed to go
   by reference. */
int go_to_point(double goal_x, double goal_y,
                double robot_x, double robot_y, double robot_theta,
                double* r_dot, double* theta_dot)
{
    Point goal = Point(goal_x, goal_y); // Turn into point for convenience
    // Get distance and direction to goal
    double dr = goal.distance_to(robot_x, robot_y);
    double dtheta = goal.angle_to(robot_x, robot_y, robot_theta);
    // Don't move if more than 22.5 degrees off, otherwise scale speed with log
    // of angle but cap at dr to prevent overshoot
    *r_dot = (fabs(dtheta) < PI/8) * dr * -log(fabs(dtheta)) / 10.0;
    *r_dot = (dr < *r_dot) ? dr : *r_dot;
    *r_dot = (MAX_SPEED < *r_dot) ? MAX_SPEED : *r_dot;
    // Turn angle is simply direction offset
    *theta_dot = dtheta / 2;    // Slow rotation to prevent overshoot
    
    // Check that the turn rate is greater than the minimum if turning in place
    if (fabs(*theta_dot) < MIN_TURN_RATE && fabs(*r_dot) < SPEED_EPS &&
        fabs(*theta_dot) > ANGLE_EPS)
    {
        // Set to min turn rate if below
        *theta_dot = MIN_TURN_RATE * ((*theta_dot > 0.0) * 2 - 1);
    }
    return 0;   // Dummy return value
}

vector<vector<double> > sense(vector<double> range_data, vector<double> bearing_data,
          double goal_x, double goal_y, double* speed, double* turnrate, double robot_x,
          double robot_y, double robot_theta)
{
    Point goal = Point(goal_x, goal_y);
    node_x.clear();
    node_y.clear();
    range_nodes.clear();
    node_x.insert(node_x.begin(), robot_x);
    node_y.insert(node_y.begin(), robot_y);    
    range_nodes.insert(range_nodes.begin(), 0); 
      
    for (i = 1; i < range_data.size() - 1; i++)
    {
        // Look for discontinuities in the range data. Put those discontinuities into a vector of nodes.
        //if (range_data[i] < *max_element(range_data.begin(), range_data.end()) &&
                //(range_data[i + 1] == *max_element(range_data.begin(), range_data.end()) ||
                //range_data[i - 1] == *max_element(range_data.begin(), range_data.end())))
        if (fabs(range_data[i] - range_data[i + 1]) > RANGE_DIFF || 
                 fabs(range_data[i - 1] - range_data[i]) > RANGE_DIFF)
        {
            if (first_pass)
            {
                node_x.insert(node_x.end(), robot_x + range_data[i] * cosf(bearing_data[i]));
                node_y.insert(node_y.end(), robot_y + range_data[i] * sinf(bearing_data[i]));
            }
            else
            {
                node_x.insert(node_x.end(), robot_x - range_data[i] * cosf(bearing_data[i]));
                node_y.insert(node_y.end(), robot_y - range_data[i] * sinf(bearing_data[i]));
            }
            range_nodes.insert(range_nodes.end(), range_data[i]);
        }
        // Add node T, the "reflected" goal position, to the graph. T cannot be on the edge of an obstacle.
        if (fabs(bearing_data[i] - goal.angle_to(robot_x, robot_y, robot_theta)) < BEAR_DIFF &&
            range_data[i] >= goal.distance_to(robot_x, robot_y))
        {
            node_x.insert(node_x.end(), goal_x);
            node_y.insert(node_y.end(), goal_y);
            range_nodes.insert(range_nodes.end(), range_data[i]);
        }
        else if (fabs(bearing_data[i] - goal.angle_to(robot_x, robot_y, robot_theta)) < BEAR_DIFF &&
                range_data[i] == *max_element(range_data.begin(), range_data.end()))
        {
            if (first_pass)
            {
                node_x.insert(node_x.end(), robot_x + range_data[i] * cosf(bearing_data[i]));
                node_y.insert(node_y.end(), robot_y + range_data[i] * sinf(bearing_data[i]));
            }
            else
            {
                node_x.insert(node_x.end(), robot_x - range_data[i] * cosf(bearing_data[i]));
                node_y.insert(node_y.end(), robot_y - range_data[i] * sinf(bearing_data[i]));
            }
            range_nodes.insert(range_nodes.end(), range_data[i]);
        }
    }
    if (first_pass)
    {
        nodes1.push_back(node_x);
        nodes1.push_back(node_y);
        nodes1.push_back(range_nodes);
        cout << "First " << node_x.size() << endl;
        for (i = 0; i < node_x.size(); i++)
        {
            cout << "x: " << node_x[i] << " " << "y: " << node_y[i] << endl;
        }
        return nodes1;
    }
    else
    {
        nodes2.push_back(node_x);
        nodes2.push_back(node_y);
        nodes2.push_back(range_nodes);
        cout << "Second " << node_x.size() << endl;
        for (i = 0; i < node_x.size(); i++)
        {
            cout << "x: " << node_x[i] << " " << "y: " << node_y[i] << endl;
        }
        return nodes2;
    }
}     

bool figure_out_movement(double * speed, double * turnrate,
    vector<double> range_data, vector<double> bearing_data, unsigned int n,
    double goal_x, double goal_y, double robot_x, double robot_y, 
    double robot_theta, double dmin) {
    
    Point goal = Point(goal_x, goal_y);
    Point pose = Point(robot_x, robot_y);
    double dsmallest;
    bool not_done = true;
    
    // Act based on state
    switch(state) {
              
       case mtg :
       {
              // Sense to determine the linear tangent graph
              if (first_pass)
              {
                    nodes1 = sense(range_data, bearing_data, goal_x, goal_y, speed, 
                            turnrate, robot_x, robot_y, robot_theta);
                    dsmallest = 100;
                    for (i = 0; i < node_x.size(); i ++)
                    {
                        if (goal.distance_to(nodes1[0][i], nodes1[1][i]) < dsmallest)
                        {
                            dsmallest = goal.distance_to(nodes1[0][i], nodes1[1][i]);
                            node_closest_x = nodes1[0][i];
                            node_closest_y = nodes1[1][i];
                            range_closest = nodes1[2][i];
                        }
                    }
                    goal_theta = PI + robot_theta;
                    robot_start_x = robot_x;
                    robot_start_y = robot_y;
                    second_pass = true;
                    first_pass = false;
                    break;
              }
              else if (second_pass)
              {
                    // Turn 180 degrees to scan behind the robot
                    *turnrate = MIN_TURN_RATE;
                    *speed = 0;
                    if (fabs(robot_theta - goal_theta) > ANGLE_EPS)
                    {
                        break;
                    }
                    nodes2 = sense(range_data, bearing_data, goal_x, goal_y, speed, 
                            turnrate, robot_x, robot_y, robot_theta);
                    for (i = 0; i < node_x.size() + 1; i ++)
                    {
                        if (goal.distance_to(nodes2[0][i], nodes2[1][i]) < dsmallest)
                        {
                            dsmallest = goal.distance_to(nodes2[0][i], nodes2[1][i]);
                            node_closest_x = nodes2[0][i];
                            node_closest_y = nodes2[1][i];
                            range_closest = nodes2[2][i];
                        }
                    }
                    second_pass = false;
                    break;
              }
              else
              {
                    Point pose_start = Point(robot_start_x, robot_start_y);
                    // See if any nodes in the linear tangent graph can get you
                    // closer to the goal.
                    cout << "Closest " << node_closest_x << " " << node_closest_y << endl;
                    // If yes, go to that node.     
                    if (goal.distance_to(node_closest_x, node_closest_y) < dmin)
                    {
                        go_to_point(node_closest_x, node_closest_y, robot_x, robot_y, 
                                    robot_theta, speed, turnrate);
                        // Don't run into any walls
                        if (range_closest - pose_start.distance_to(node_closest_x, node_closest_y) > 
                            MIN_WALL_DIST)
                        {
                            if (pose.distance_to(node_closest_x, node_closest_y) > DIST_EPS)
                            {
                                break;
                            } 
                        }
                        else
                        {
                            if (pose.distance_to(node_closest_x, node_closest_y) > MIN_WALL_DIST)
                            {
                                break;
                            }
                        }    
                    }
                    // Goal reached
                    if (goal.distance_to(node_closest_x, node_closest_y) < DIST_EPS)
                    {
                        printf("Success! Goal reached.\n");
                        *speed = 0;
                        *turnrate = 0;
                        not_done = false;
                        break;
                    }
                    // If no nodes on the linear tangent graph can get you any closer
                    // to the goal, switch to boundary following mode
                    else if (goal.distance_to(node_closest_x, node_closest_y) == dmin)
                    {
                        state = bf;
                    }
                    else
                    {
                        first_pass = true;
                        second_pass = false;
                        break;
                    }
             } 
        }               
        case bf :
        {
              dsmallest = 100;
              for (i = 0; i < range_data.size(); i++)
              {
                   range_x[i] = range_data[i] * cos(bearing_data[i]);
                   range_y[i] = range_data[i] * sin(bearing_data[i]);
                   if (goal.distance_to(range_x[i], range_y[i]) < dmin)
                   {
                        dmin = goal.distance_to(range_x[i], range_y[i]);
                   }
              }
              if (first_pass)
              {
                   for(i = 0; i < nodes.size(); i++)
                   {
                        if (goal.distance_to(nodes[1][i], nodes[2][i]) < dsmallest)
                        {
                            dsmallest = goal.distance_to(nodes[1][i], nodes[2][i]);
                            node_closest_x = nodes[1][i];
                            node_closest_y = nodes[2][i];
                            Point node_closest = Point(node_closest_x, node_closest_y);
                        }
                    }

                    first_pass = false;
                    state = bf;
                    break;
              }
              else
              {
                  go_to_point(node_closest_x, node_closest_y, robot_x, robot_y,
                              robot_theta, speed, turnrate);
                  if (pose.distance_to(node_closest_x, node_closest_y) > DIST_EPS)
                  {
                        break;
                  }      
                  Point node_closest = Point(node_closest_x, node_closest_y);
                  if (goal.distance_to(node_closest_x, node_closest_y) < DIST_EPS)
                  {
                        printf("Success! Goal reached.\n");
                        *speed = 0;
                        *turnrate = 0;
                        break;
                  }
                  else if (node_closest.distance_to(goal_x, goal_y) <= dmin)
                  {
                        state = mtg;
                        break;
                  }
                  else
                  {
                        nodes = sense(range_data, bearing_data, goal_x, goal_y, 
                                      speed, turnrate, robot_x, robot_y, robot_theta);
                        Point node_start = Point(node_closest_x, node_closest_y);
                        Point node_follow = Point(node_closest_x, node_closest_y);
                        while (node_follow.distance_to(robot_x, robot_y) < DIST_EPS)
                        {
                                // Follow wall for a certain distance
                                node_follow = Point(node_closest_x, node_closest_y);
                                if (node_follow.x == node_start.x && node_follow.y == node_start.y)
                                {
                                        printf("Failure. No path found.\n");
                                        *speed = 0;
                                        *turnrate = 0;
                                        not_done = false;
                                        break;
                                }
                        }
                        if (goal.distance_to(node_closest_x, node_closest_y) < dmin)
                        {
                                state = mtg;
                                break;
                        }
                        break;
                  }
              }

            break;
            }
       // Something broke
       default : return false;
       }
       return not_done;
}  
