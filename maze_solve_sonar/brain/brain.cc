
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;
using std::vector;
using std::pair;

float goal_x = 19;
float goal_y = 0;
float x = -20;
float y = 0;
vector< pair <float, float> > vect;
bool last_turn = true;
bool short_turn = true;

/*
 * Takes the angle and velocity of the robot and returns the coordinates of the robot
 * in a float pair.
 */
pair <float, float> get_distance(float theta, float velocity)
{
    // get velocity and distance traveled this callback in x direction
    float x_vel = velocity * cos(theta);
    float x_dist = x_vel * 0.02;

    // get velocity and distance traveled this callback in y direction
    float y_vel = velocity * sin(theta);
    float y_dist = y_vel * 0.02;

    pair <float, float> coords = {x_dist, y_dist};

    return coords;
}

/*
 * Takes the robot, turning velocity, and straight velocity turns the robot towards the goal
 */
void turn_to_goal(Robot* robot, float turning_v, float velocity)
{
    float theta = robot->pos_t;

    // determine which way to turn based on direction of goal
    float rg_xdiff = x - goal_x;
    float rg_ydiff = y - goal_y;
    float g_angle = atan(rg_ydiff/rg_xdiff);
    float angle_diff = theta - g_angle;

    if (x > goal_x && y < 0) 
    {
	angle_diff = angle_diff - M_PI;
	cout << "hererere" << endl;
    }

    // logic for when beyond goal coordinate
    if ((x > goal_x) && y > 0)
    {
	if (theta > 0)
	{  
	    angle_diff = (theta - (M_PI - g_angle));
	    angle_diff = angle_diff < 0? angle_diff : -angle_diff;
	}
	else
	{
	    angle_diff = (theta + (g_angle));
	}

		cout << "here3" << endl;

    }

    cout << "theta: " << theta << endl;
    cout << "g_angle: " << g_angle << endl;
    cout << "theta diff: " << angle_diff  << endl;
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    // if less than a threshold, turn towards goal to the left
    if (angle_diff < -0.03)
    {
        cout << "turning left" << endl;
	robot->set_vel(-turning_v, turning_v);
	return;
    }
    // if more than a threshold, turn towards goal to the right
    if (angle_diff > 0.03)
    {
	cout << "turning right" << endl;
        robot->set_vel(turning_v,-turning_v);
        return;
    }
    // if within the threshold (very close to heading towards the goal), head straight
    else  
    {
	cout << "going straight" << endl;
	robot->set_vel(velocity, velocity);

        // pair of distance components in x and y
	pair distance_components = get_distance(theta, velocity);

        // robot moving straight so add to x and y
	// add x and y distances to previous x coordinate to update them
	x = x + distance_components.first;
	y = y + distance_components.second;

        return;
    }
}

void callback(Robot* robot)
{  
    // robot orientation	
    float theta = robot->pos_t; 

    // initializing estimated x and y component velocities 
    float x_vel = 0;
    float y_vel = 0;

    // initializing estimated x and y component distances
    float x_dist = 0;
    float y_dist = 0;

    // velocity to travel straight 
    float velocity = 2;

    // velocity to turn at (slow enough for less inconsistency)
    float turning_v = 1;

    // initializing coordinates to avoid seg fault
    pair<float, float> top;
    top.first = 0;
    top.second = 0;

    // if no vector yet, revert to top pair coordinates
    pair coord = vect.size() > 0? vect.back() : top;

    // previous coordinates pushed in past callback loop
    float prev_x = coord.first;
    float prev_y = coord.second;

    /* 
     * If the robot is very close to an obstacle base decision on which way to turn on last_action,
     * which is a value that represents which way the robot was turning when it was within this range
     * of an obstacle
     */
    if (robot->range < 0.45) {
	cout << "short turn: " << short_turn << endl;
	// set short turn to true (this range calls for a short range turn to be executed)
	short_turn = true;

	// once within a certain range of the goal, head towards it
	if ((x > goal_x - 2 && x < goal_x + 2) && (y > goal_y - 2 && y < goal_y + 2))
	{
	    turn_to_goal(robot, turning_v, velocity);
	    return;
	}

	if (last_turn == true) // turn left
	{
            robot->set_vel(-turning_v, turning_v);
	    return;
	}
	else // turn right
	{
            robot->set_vel(turning_v, -turning_v);
	    return;
	}
    }

    /*
     * Second range condition: medium range
     * If within a certain range, ramain on a tangent path to the obstacle
     */
    if (robot->range > 0.45 && robot->range < 1.5) {

	if ((x > goal_x - 2 && x < goal_x + 2) && (y > goal_y - 2 && y < goal_y + 2))
	{
	    cout << "here" << endl;
	    turn_to_goal(robot, turning_v, velocity);
	    return;
	}
	// add coordinate to vector of visited coordinates
	// to limit complexity, add one whenever traveled 1 unit    
	if (abs(x - prev_x) > 1 | abs(y - prev_y) > 1) 
	{
            pair <float, float> thisCoord;
	    thisCoord.first = x;
	    thisCoord.second = y;
	    vect.push_back(thisCoord);
	}
	   
        // set velocity of robot to go tangent along obstacle	
	robot->set_vel(velocity, velocity);

	// pair of distance components in x and y 
	pair distance_components = get_distance(theta, velocity);

	// robot moving straight so add to x and y 
	// add x and y distances to previous x coordinate to update them
	x = x + distance_components.first;
	y = y + distance_components.second;

	return;
    }

    /*
     * Third range condition: far range
     * If the range is far, then there are no obstacles in the immediate trajectory of the robot.
     * The robot will head towards the estimated direction of the goal using its estimated
     * coordinates and the coordinates of the goal.
     */
    if (robot->range >= 1.5)
    {	
	float xdif = abs(x - prev_x);
	float ydif = abs(y - prev_y);

	cout << "short turn: " << short_turn << endl;
	
        // if the last turn was a short turn (range < 0.4), then change the last_turn parameter so 
	// that the next short turn executed is the opposite.
	if (short_turn == true) 
	{
	    last_turn = !last_turn;
	    short_turn = false; // this turn is a long range turn so set short turn to false
	}



	// if far away enough from obstacle, start turning towards the goal.
	// because the coordinates are only pushed when traveling straight forward, xdif and ydif are
	// the distance from the last obstacle when the robot has just left its vicinity.
	if (xdif > 1.5 | ydif > 1.5)
        {
	    turn_to_goal(robot, turning_v, velocity);

/*	    // determine which way to turn based on direction of goal
	    float rg_xdiff = x - goal_x;
	    float rg_ydiff = y - goal_y;
	    float g_angle = atan(rg_ydiff/rg_xdiff);

	    if (x > goal_x) 
	    {
	        g_angle = g_angle + 2*M_PI;
	    }

	    // if less than a threshold, turn towards goal to the left
	    if (robot->pos_t - g_angle < -0.05)
	    {
		cout << "turning left" << endl;
		robot->set_vel(-turning_v, turning_v);
		return;
	    }
	    // if more than a threshold, turn towards goal to the right
	    if (robot->pos_t - g_angle > 0.05)
	    {
		cout << "turning right" << endl;
                robot->set_vel(turning_v,-turning_v);
		return;
	    }
	    // if within the threshold (very close to heading towards the goal), head straight
	    else  
	    {
		cout << "going straight" << endl;
	        robot->set_vel(velocity, velocity);

		// pair of distance components in x and y
	        pair distance_components = get_distance(theta, velocity);

		// robot moving straight so add to x and y
	        // add x and y distances to previous x coordinate to update them
	        x = x + distance_components.first;
	        y = y + distance_components.second;

		return;
	    }*/
        }
	else 
	{
            robot->set_vel(velocity, velocity);
	  
	    // robot going straight so add to x and y 
	    // distance pair in x and y 
	    pair distance_components = get_distance(theta, velocity);

	    x = x + distance_components.first;
	    y = y + distance_components.second;
	}
    }

    return;
}


int main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
