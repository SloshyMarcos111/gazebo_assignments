
#include <iostream>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;
using std::vector;
using std::pair;

float goal_x = 20;
float goal_y = 0;
float x = -20;
float y = 0;
vector< pair <float, float> > vect;
int last_action = 1;


// robot->position_t is the theta angle of robot
int swivel_detect(Robot* robot, float originalAngle) 
{
    int direction = 0;
    bool detected = false;

    // swivel right
    while (!detected || abs(robot->pos_t - originalAngle) < 0.30 )
    {
	cout << robot->pos_t << endl;
        robot->set_vel(1.0, -1.0);
	
	if (robot->range < 2.0) {
	    direction = 2; // wall on right so check left; should turn left until now
	    detected = true;
	}
    }
   
    detected = false;

    // swivel left
    while (!detected | abs(robot->pos_t - originalAngle) < 0.30 )
    {
	cout << robot->pos_t << endl;
        robot->set_vel(-1.0, 1.0);
	
	if (robot->range < 2.0) {
	    direction += 1; // if 1 then path open on right (turn right); if 3 then wall on either side (go straight)
	    detected = true;
	}
    }

    while (abs(robot->pos_t - originalAngle) < 0.05) 
    {
	cout << robot->pos_t << endl;
        robot->set_vel(-1.0, 1.0);
    }

    return direction; // default value;
}

void turn_robot(Robot* robot, int direction, float originalAngle)
{
    
    switch (direction) 
    {
        case 1:
		while (abs(robot->pos_t - originalAngle) < 0.05)
		{
		    robot->set_vel(1.0, -1.0);
		}

            break;
	case 3:
	    	while (abs(robot->pos_t - originalAngle) < 0.05)
		{
		    robot->set_vel(-1.0, 1.0);
		}
	    break;

	default:
	    break;
    }

    return;
}

// random number gen for turning when range is small
int roll12()
{
    return rand() % 2 + 1;
}

void
callback(Robot* robot)
{   
    float theta = robot->pos_t; 
    float x_vel = 0;
    float y_vel = 0;
    float x_dist = 0;
    float y_dist = 0;
    float velocity = 2;
    int direction = 0;
    float turning_v = 1;
    pair<float, float> top;
    top.first = 0;
    top.second = 0;
    pair coord = vect.size() > 0? vect.back() : top;
    float prev_x = coord.first;
    float prev_y = coord.second;

    cout << "prev x: " << prev_x << endl;
    cout << "prev y: " << prev_y << endl;

    if (robot->range < 0.4) {

	if (last_action == 1)
	{
	    
            robot->set_vel(-turning_v, turning_v);
	    return;
	}
	else
	{
            robot->set_vel(turning_v, -turning_v);
	    return;
	}
    }



	//for (int i = 0; i < vect.size() - 1; i++)
	//{
            /*pair search_coordinates = vect[i];
	    search_x = vect[i].first;
	    search_y = vect[i].second;

	    pair curr_coordinates = vect.back();
	    curr_x = curr_coordinates.first;
	    curr_y = curr_coordinates.second;*/


	    //if (search_x <= curr_x)
	   // {
	     //   if (search_y
//	    robot->set_vel(-turning_v, turning_v);


    // wall on the right
    if (robot->range > 0.4 && robot->range < 1.5) {

    	    
	// add wall to map if enough distance from previous entry
	if (abs(x - prev_x) > 1 | abs(y - prev_y) > 1) 
	{
            pair <float, float> thisCoord;
	    thisCoord.first = x;
	    thisCoord.second = y;
	    vect.push_back(thisCoord);
	}
	    
	robot->set_vel(velocity, velocity);

	// get velocity and distance traveled this callback in x direction
	x_vel = velocity * cos(theta);
	x_dist = x_vel * 0.02;

	// get velocity and distance traveled this callback in y direction
	y_vel = velocity * sin(theta);
	y_dist = y_vel * 0.02;

	// add x and y distances to previous x coordinate to update them
	x = x + x_dist;
	y = y + y_dist;

	return;
    }

    //direction = swivel_detect(robot, robot->pos_t);
    //turn_robot(robot, direction, robot->pos_t);

    if (robot->range >= 1.5)
    {	
	float xdif = abs(x - prev_x);
	float ydif = abs(y - prev_y);
	
        // clear last action
	last_action = last_action == 1? 0:1;

	if (xdif > 1.5 | ydif > 1.5)
        {
	    cout << "difference x: " << abs(x - prev_x) << endl;
	    cout << "difference y: " << abs(y - prev_y) << endl;

	    // determine which way to turn based on direction of goal
	    float rg_xdiff = x - goal_x;
	    float rg_ydiff = y - goal_y;
	    
	    float g_angle = atan(rg_ydiff/rg_xdiff);
	    cout << "g_angle: " << g_angle << endl;

	    cout << "difference in angles: " << robot->pos_t - g_angle << endl;
	    if (robot->pos_t - g_angle < -0.05)
	    {
		cout << "turning left" << endl;
		robot->set_vel(-turning_v, turning_v);
		return;
	    }
	    if (robot->pos_t - g_angle > 0.05)
	    {
		cout << "turning right" << endl;
                robot->set_vel(turning_v,-turning_v);
		return;
	    }
	    else  
	    {
		cout << "going straight" << endl;
	        robot->set_vel(velocity, velocity);
		// get velocity and distance traveled this callback in x direction
	        x_vel = velocity * cos(theta);
	        x_dist = x_vel * 0.02;

	        // get velocity and distance traveled this callback in y direction
	        y_vel = velocity * sin(theta);
	        y_dist = y_vel * 0.02;

	        // add x and y distances to previous x coordinate to update them
	        x = x + x_dist;
	        y = y + y_dist;

		return;
	    }
        }
	else 
	{
	    bool isx = abs(x - prev_x) > 1? true : false;
	    bool isy = abs(y - prev_x) > 1? true : false;

	    cout << "difference x > 1?: " << isx << endl;
	    cout << "difference y > 1?: " << isy << endl;
	    cout << "range: " << robot->range << endl;
            robot->set_vel(velocity, velocity);
	    
	    // get velocity and distance traveled this callback in x direction
	    x_vel = velocity * cos(theta);
	    x_dist = x_vel * 0.02;

	    // get velocity and distance traveled this callback in y direction
	    y_vel = velocity * sin(theta);
	    y_dist = y_vel * 0.02;

	    // add x and y distances to previous x coordinate to update them
	    x = x + x_dist;
	    y = y + y_dist;
	}
    }


    return;
}



int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    cout << "range: " << robot.range << endl;
    robot.do_stuff();

    return 0;
}
