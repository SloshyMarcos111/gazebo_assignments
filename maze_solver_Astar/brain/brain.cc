
#include <iostream>
#include <thread>
#include <math.h>

#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;
using std::pair;
using std::clamp;
using std::make_pair;
using std::vector;
using std::set;
using std::stack;


#define ROWS 480
#define COLS 480


typedef pair<int, int> Pair;
typedef pair <float, pair<int, int>> pPair;

pair <int, int> occupancy_grid[ROWS][COLS]; // 0.25 resolution
int path_grid [ROWS][COLS];
int b_grid[ROWS][COLS];
vector <pair<float, float>> coordinates; 
pair<float, float> prev_coord = make_pair(0, 0);  // vector of past robot x and y coords
int remain_count = 0;
float xsum = 0;
float ysum = 0;
float xavg = 0;
float yavg = 0;

int turn = 1;

bool traj_ready = false;

float goal_x = 20.0;
float goal_y = 0.0;

int source_x = 0;
int source_y = 0;

struct cell 
{ 
    // Row and Column index of its parent 
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
    int parent_i, parent_j; 
    // f = g + h 
    double f, g, h; 
}; 

/*
 * Initialize the occupancy grid
 */
void initialize_grid()
{
    for (int i = 0; i < ROWS; i++)
    {
	for (int j = 0; j < COLS; j++)
	{
            occupancy_grid[i][j].first = 0;
	    occupancy_grid[i][j].second = 0;
	}
    }
}

/*
 * Takes in an x and y coordinate local to the robot (measured from the robot's sensors)
 * and transforms them using a 3D transformation matrix into the global world coordinates
 */
float** transform_coordinates(float x_in, float y_in, float theta)
{
    float H [3][3];  // transformation matrix
    H[0][0] = cos(theta);
    H[0][1] = -sin(theta);
    H[0][2] = prev_coord.first;  // robot global x
    H[1][0] = sin(theta);
    H[1][1] = cos(theta);
    H[1][2] = prev_coord.second;  // robot global y
    H[2][0] = 0;
    H[2][1] = 0;
    H[2][2] = 1;

    float local_coord[3][1];  // local coordinates of hit 
    local_coord [0][0] = x_in;//hit.range*cos(hit.angle);  // x coordinate local to robot
    local_coord [1][0] = y_in;//hit.range*sin(hit.angle);  // y coordinate local to robot
    local_coord [2][0] = 1;

    float** world_coord = 0; // world coord frame matrix
    world_coord = new float*[3];

    for (int h = 0; h < 3; h++)
    {
	world_coord[h] = new float[1];

	for (int w = 0; w < 1; w++)
	{
            world_coord[h][w] = 0;
	}
    }

    world_coord[2][0] = 1;

    // multiply matrices
    for (int i = 0; i < 3; i++)
    {
	for(int j = 0; j < 3; j++)
	{
	    world_coord[i][0] += H[i][j] * local_coord[j][0];
	}
    }

    return world_coord;
}

float normalize(float x_in)
{
    // Normalize the coordinates to prepare for drawing
    float clamped_x = clamp(x_in, -27, 27);
    
    return (round(clamped_x/0.25))*0.25 + 27;
}

void get_distance(float theta, float velocity)
{
    // get velocity and distance traveled this callback in x direction
    float x_vel = velocity * cos(theta);
    float x_dist = x_vel * 0.02;

    // get velocity and distance traveled this callback in y direction
    float y_vel = velocity * sin(theta);
    float y_dist = y_vel * 0.02;

    prev_coord.first += x_dist;
    prev_coord.second += y_dist;
}

/*
* Decide which cells are occupied or not by comparing miss and hit values and applying thresholds that 
* represent the odds that there is a block given that most spaces are likely empty
*/
void fill_grid(float hit_range, float hit_angle, float theta) 
{
    // get the world coordinates of the hit
    float** world_coord = transform_coordinates(hit_range*cos(hit_angle), hit_range*sin(hit_angle), theta);
    
    // Normalize the coordinates to prepare for drawing
    float to_draw_x = normalize(world_coord[0][0]);
    float to_draw_y = normalize(world_coord[1][0]);

    float robot_x = normalize(prev_coord.first);
    float robot_y = normalize(prev_coord.second);

    for (int i = 0; i < ROWS; i++) 
    {
        for (int j = 0; j < COLS; j++)
        {
	    if (to_draw_x*4 == i && to_draw_y*4 == j)
	    {
	        occupancy_grid[i][j].first += 1;

	        if ((occupancy_grid[i][j].first > occupancy_grid[i][j].second - 5))
	        {
		    b_grid[i][j] = 0;
	        }
	    } 

	    if ((robot_x*4 == i && robot_y*4 == j))
	    {
	        occupancy_grid[i][j].second += 1;
	        if (occupancy_grid[i][j].second > occupancy_grid[i][j].first + 10)
	        { 
		    b_grid[i][j] = 1;
	        }
	    }
        }
    }
}

/**
 * Computes angle difference, accounting for the weirdness with the angle
 * system.
 *
 * Positive means turn CCW, negative means turn CW.
 */
float angle_diff(float from, float to) {
    float from_adj = from < 0 ? (2 * M_PI) + from : from;
    float to_adj = to < 0 ? (2 * M_PI) + to : to;
    float raw_delta = fmodf(to_adj - from_adj, 2 * M_PI);

    if (raw_delta < -M_PI) {
        return (2 * M_PI) + raw_delta;
    }

    if (raw_delta > M_PI) {
        return (-2 * M_PI) + raw_delta;
    }

    return raw_delta;
}

float get_relative_traj_angle(Robot* robot)
{
    // get trajectory angle	
    float traj_angle = 0;
    int x = source_x;
    int y = source_y;

    for (int i = x - 1; i <= x + 1; i++)
    {
	for (int j = y - 1; j <= y + 1; j++)
	{
            cout << path_grid[i][j] << " ";
	}
	cout << endl;
    }
    if (path_grid[x-1][y] == 1)
    {
	traj_angle = -3.142;
    }
   
    if (path_grid[x-1][y-1] == 1)
    {
	traj_angle = -2.356 ;
    }
    
    if (path_grid[x][y-1] == 1)
    {
	traj_angle = -1.571;
    }
   
    if (path_grid[x+1][y-1] == 1)
    {
	traj_angle = -0.7854;
    }
    
    if (path_grid[x+1][y] == 1)
    {
	traj_angle = 0;
    }
   
    if (path_grid[x+1][y+1] == 1)
    {
	traj_angle = 0.7854;
    }
    
    if (path_grid[x][y+1] == 1)
    {
	traj_angle = 1.571;
    }
    
    if (path_grid[x-1][y+1] == 1)
    {
	traj_angle = 2.356;
    }

    cout << "traj angle: " << traj_angle << endl;
    return traj_angle;
}

void trajectory_follow(Robot* robot)
{
    float traj_angle = get_relative_traj_angle(robot);
    float diff = angle_diff(robot->pos_t, traj_angle);

    turn = rand() % 2;
    cout << "diff: " << diff << endl;
    int push = 0;

    if ((diff <= -0.3) || (diff >= 0.3))
    {
	if (diff < 0)
	{
	    push = -1;
	}
	if (diff > 0)
	{
            push = 1;
	}

	robot->set_vel(-diff - push, diff + push);

	return;
    }

    robot->set_vel(3, 3);
    get_distance(robot->pos_t, 3);
    return;
}




void callback(Robot* robot)
{
    // average first 100 readings of x and y coordinates to initialize position
    if (remain_count < 50) 
    {
	remain_count++;
	robot->set_vel(0, 0);

	//remain in one spot and read position data and average it
	xsum += robot->pos_x;
	ysum += robot->pos_y;

        xavg = xsum / remain_count;
        yavg = ysum / remain_count;
        prev_coord.first = round(xavg);
        prev_coord.second = round(yavg);

	return;
    }

    if(!traj_ready)
    {
	robot->set_vel(0, 0);
    }

    if (remain_count >= 50 && traj_ready)
    {
	cout << "\n===" << endl;
	for (auto hit : robot->ranges) {
            if (hit.range < 100) {

	        fill_grid(hit.range, hit.angle, robot->pos_t);
            }

	    float range = hit.range;

	    if (isinf(hit.range))
	    {
	        range = 2.0; 
	    }

	    float** miss_coord;
	    float miss_x = 0;
	    float miss_y = 0;

	    for (float range_i = range*4 - 1; range_i >= 0; range_i--)
	    {
		miss_coord = transform_coordinates((range_i/4.0)*cos(hit.angle), (range_i/4.0)*sin(hit.angle), robot->pos_t);
	        miss_x = normalize(miss_coord[0][0]);
		miss_y = normalize(miss_coord[1][0]);

		for (int i = 0; i < ROWS; i++)
		{
	            for (int j = 0; j < COLS; j++)
	            {
		        if (miss_x*4 == i && miss_y*4 == j)
			{
		            occupancy_grid[i][j].second += 1;

		            if (occupancy_grid[i][j].second > occupancy_grid[i][j].first + 50)
		             {
				 b_grid[i][j] = 1;
		             }
			 }
		     }
		 }
	     }
	 }

	 if (robot->ranges.size() < 5) {
	     return;
	 }

	 /*
	  * robot wall follow
	  */
	  if (robot->ranges[3].range < 0.8 || robot->ranges[4].range < 0.8 || robot->ranges[2].range < 0.8)
	  {
              float traj_angle = get_relative_traj_angle(robot);
	      float diff = angle_diff(robot->pos_t, traj_angle);
	      cout << "diff wall: " << diff << endl;
	      int push = 0;
              cout << "turnnhere" << endl;
             if (turn == 1)
             {
		robot->set_vel(2, -2);

		turn = 1;

		return;
             }

	     if (turn == 0)
	     {
		 turn = 0;

                 robot->set_vel(-2, 2);
	         turn = 0;
                 return;
	     }
	  }

	  if (robot->ranges[5].range < 1.0  && robot->ranges[4].range > 0.5)
	  {
	      robot->set_vel(3, 3);

	      get_distance(robot->pos_t, 3);
	      return;
	  }

	  if (robot->ranges[1].range < 1.0  && robot->ranges[2].range > 0.5)
	  {
	      robot->set_vel(3, 3);

	      get_distance(robot->pos_t, 3);
	      return;
	  }

	  trajectory_follow(robot);

	  return;
    }
}

// A Utility Function to check whether given cell (row, col) 
// is a valid cell or not. 
bool isValid(int row, int col) 
{ 
    // Returns true if row number and column number 
    // is in range 
    return (row >= 0) && (row < ROWS) && (col >= 0) && (col < COLS); 
}

// A Utility Function to check whether the given cell is 
// blocked or not 
bool isUnBlocked(int grid[][COLS], int row, int col) 
{ 
    // Returns true if the cell is not blocked else false 
    if (grid[row][col] != 0) 
        return (true); 
    else
        return (false); 
}

bool isDestination(int x, int y)
{
    if (x == (goal_x+30)*4 && y == (goal_y+30)*4)
    {
	return true;
    }
    
    return false;
}

// A Utility Function to calculate the 'h' heuristics. 
double calculateHValue(int row, int col, Pair dest) 
{ 
    // Return using the distance formula 
    return ((double)sqrt ((row-dest.first)*(row-dest.first) 
                          + (col-dest.second)*(col-dest.second))); 
} 
/*
 * Calculates the heuristic cost of the node.
 * param x: the x coordinate of the node (1.25 would translate to 1.25 x 4 = 5 row on grid)
 * param y: the y coordinate of the node (1.25 would translate to 1.25 x 4 = 5 col on grid)
 * return: a float heuristic cost.
 */
float determineH(float x, float y)
{
    // using Euclidean distance as heuristc function
    return (sqrt((x - goal_x) * (x - goal_x) + (y - goal_y) * (y - goal_y)));
}

void tracePath(cell cellDetails[][COLS], Pair dest) 
{ 
    printf ("\nThe Path is "); 
    int row = dest.first; 
    int col = dest.second; 
  
    stack<Pair> Path; 
  
    while (!(cellDetails[row][col].parent_i == row 
             && cellDetails[row][col].parent_j == col )) 
    { 
        Path.push (make_pair (row, col)); 
        int temp_row = cellDetails[row][col].parent_i; 
        int temp_col = cellDetails[row][col].parent_j; 
        row = temp_row; 
        col = temp_col; 
    } 
  
    Path.push (make_pair (row, col)); 
    
    pair<int,int> p = Path.top(); 
    source_x = p.first;
    source_y = p.second;

    while (!Path.empty()) 
    { 
	p = Path.top(); 
	path_grid[p.first][p.second] = 1;
        Path.pop(); 

	int p_x = normalize(p.first);
	int p_y = normalize(p.second);

	viz_path(p_x, p_y);
    } 
  
    return; 
} 

void aStarSearch(int grid[][COLS], Pair src, Pair dest) 
{
    cout << "dest i: " << dest.first << endl << "dest j: " << dest.second << endl;
    cout << "source i: " << src.first << endl << "source j: " << src.second << endl;
    // If the source is out of range 
    if (isValid (src.first, src.second) == false) 
    { 
        printf ("Source is invalid\n"); 
        return; 
    } 
  
    // If the destination is out of range 
    if (isValid (dest.first, dest.second) == false) 
    { 
        printf ("Destination is invalid\n"); 
        return; 
    } 
  
    // If the destination cell is the same as source cell 
    if (isDestination(src.first, src.second) == true) 
    { 
        printf ("We are already at the destination\n"); 
        return; 
    } 
  
    // Create a closed list and initialise it to false which means 
    // that no cell has been included yet 
    // This closed list is implemented as a boolean 2D array 
    bool closedList[ROWS][COLS]; 
    memset(closedList, false, sizeof (closedList)); 
  
    // Declare a 2D array of structure to hold the details 
    //of that cell 
    cell cellDetails[ROWS][COLS]; 
  
    int i, j; 
  
    for (i=0; i<ROWS; i++) 
    { 
        for (j=0; j<COLS; j++) 
        { 
            cellDetails[i][j].f = FLT_MAX; 
            cellDetails[i][j].g = FLT_MAX; 
            cellDetails[i][j].h = FLT_MAX; 
            cellDetails[i][j].parent_i = -1; 
            cellDetails[i][j].parent_j = -1; 
        } 
    } 
  
    // Initialising the parameters of the starting node 
    i = src.first, j = src.second; 
    cellDetails[i][j].f = 0.0; 
    cellDetails[i][j].g = 0.0; 
    cellDetails[i][j].h = 0.0; 
    cellDetails[i][j].parent_i = i; 
    cellDetails[i][j].parent_j = j; 
  
    /* 
     Create an open list having information as- 
     <f, <i, j>> 
     where f = g + h, 
     and i, j are the row and column index of that cell 
     Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
     This open list is implenented as a set of pair of pair.*/
    set<pPair> openList; 
  
    // Put the starting cell on the open list and set its 
    // 'f' as 0 
    openList.insert(make_pair (0.0, make_pair (i, j))); 
  
    // We set this boolean value as false as initially 
    // the destination is not reached. 
    bool foundDest = false; 
  
    while (!openList.empty()) 
    { 
        pPair p = *openList.begin(); 
  
        // Remove this vertex from the open list 
        openList.erase(openList.begin()); 
  
        // Add this vertex to the closed list 
        i = p.second.first; 
        j = p.second.second; 
        closedList[i][j] = true; 
       
       /* 
        Generating all the 8 successor of this cell 
  
            N.W   N   N.E 
              \   |   / 
               \  |  / 
            W----Cell----E 
                 / | \ 
               /   |  \ 
            S.W    S   S.E 
  
        Cell-->Popped Cell (i, j) 
        N -->  North       (i-1, j) 
        S -->  South       (i+1, j) 
        E -->  East        (i, j+1) 
        W -->  West           (i, j-1) 
        N.E--> North-East  (i-1, j+1) 
        N.W--> North-West  (i-1, j-1) 
        S.E--> South-East  (i+1, j+1) 
        S.W--> South-West  (i+1, j-1)*/
  
        // To store the 'g', 'h' and 'f' of the 8 successors 
        double gNew, hNew, fNew; 
  
        //----------- 1st Successor (North) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid(i-1, j) == true) 
        { 
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i-1, j) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i-1][j].parent_i = i; 
                cellDetails[i-1][j].parent_j = j; 
                printf ("The destination cell is found\n"); 
                tracePath (cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i-1][j] == false && 
                     isUnBlocked(grid, i-1, j) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.0; 
                hNew = calculateHValue (i-1, j, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i-1][j].f == FLT_MAX || 
                        cellDetails[i-1][j].f > fNew) 
                { 
                    openList.insert( make_pair(fNew, 
                                               make_pair(i-1, j))); 
  
                    // Update the details of this cell 
                    cellDetails[i-1][j].f = fNew; 
                    cellDetails[i-1][j].g = gNew; 
                    cellDetails[i-1][j].h = hNew; 
                    cellDetails[i-1][j].parent_i = i; 
                    cellDetails[i-1][j].parent_j = j; 
                } 
            } 
        } 
  
        //----------- 2nd Successor (South) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid(i+1, j) == true) 
	{
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i+1, j) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i+1][j].parent_i = i; 
                cellDetails[i+1][j].parent_j = j; 
                printf("The destination cell is found\n"); 
                tracePath(cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i+1][j] == false && 
                     isUnBlocked(grid, i+1, j) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.0; 
                hNew = calculateHValue(i+1, j, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i+1][j].f == FLT_MAX || 
                        cellDetails[i+1][j].f > fNew) 
                { 
                    openList.insert( make_pair (fNew, make_pair (i+1, j))); 
                    // Update the details of this cell 
                    cellDetails[i+1][j].f = fNew; 
                    cellDetails[i+1][j].g = gNew; 
                    cellDetails[i+1][j].h = hNew; 
                    cellDetails[i+1][j].parent_i = i; 
                    cellDetails[i+1][j].parent_j = j; 
                } 
            } 
        } 
  
        //----------- 3rd Successor (East) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid (i, j+1) == true) 
        { 
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i, j+1) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i][j+1].parent_i = i; 
                cellDetails[i][j+1].parent_j = j; 
                printf("The destination cell is found\n"); 
                tracePath(cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
  
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i][j+1] == false && 
                     isUnBlocked (grid, i, j+1) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.0; 
                hNew = calculateHValue (i, j+1, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i][j+1].f == FLT_MAX || 
                        cellDetails[i][j+1].f > fNew) 
                { 
                    openList.insert( make_pair(fNew, 
                                        make_pair (i, j+1))); 
  
                    // Update the details of this cell 
                    cellDetails[i][j+1].f = fNew; 
                    cellDetails[i][j+1].g = gNew; 
                    cellDetails[i][j+1].h = hNew; 
                    cellDetails[i][j+1].parent_i = i; 
                    cellDetails[i][j+1].parent_j = j; 
                } 
            } 
        } 
  
        //----------- 4th Successor (West) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid(i, j-1) == true) 
        { 
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i, j-1) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i][j-1].parent_i = i; 
                cellDetails[i][j-1].parent_j = j; 
                printf("The destination cell is found\n"); 
                tracePath(cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
  
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i][j-1] == false && 
                     isUnBlocked(grid, i, j-1) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.0; 
                hNew = calculateHValue(i, j-1, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i][j-1].f == FLT_MAX || 
                        cellDetails[i][j-1].f > fNew) 
                { 
                    openList.insert( make_pair (fNew, 
                                          make_pair (i, j-1))); 
  
                    // Update the details of this cell 
                    cellDetails[i][j-1].f = fNew; 
                    cellDetails[i][j-1].g = gNew; 
                    cellDetails[i][j-1].h = hNew; 
                    cellDetails[i][j-1].parent_i = i; 
                    cellDetails[i][j-1].parent_j = j; 
                } 
            } 
        } 
  
        //----------- 5th Successor (North-East) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid(i-1, j+1) == true) 
        { 
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i-1, j+1) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i-1][j+1].parent_i = i; 
                cellDetails[i-1][j+1].parent_j = j; 
                printf ("The destination cell is found\n"); 
                tracePath (cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
  
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i-1][j+1] == false && 
                     isUnBlocked(grid, i-1, j+1) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.414; 
                hNew = calculateHValue(i-1, j+1, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i-1][j+1].f == FLT_MAX || 
                        cellDetails[i-1][j+1].f > fNew) 
                { 
                    openList.insert( make_pair (fNew,  
                                    make_pair(i-1, j+1))); 
  
                    // Update the details of this cell 
                    cellDetails[i-1][j+1].f = fNew; 
                    cellDetails[i-1][j+1].g = gNew; 
                    cellDetails[i-1][j+1].h = hNew; 
                    cellDetails[i-1][j+1].parent_i = i; 
                    cellDetails[i-1][j+1].parent_j = j; 
                } 
            } 
        } 
  
        //----------- 6th Successor (North-West) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid (i-1, j-1) == true) 
        { 
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination (i-1, j-1) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i-1][j-1].parent_i = i; 
                cellDetails[i-1][j-1].parent_j = j; 
                printf ("The destination cell is found\n"); 
                tracePath (cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
  
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i-1][j-1] == false && 
                     isUnBlocked(grid, i-1, j-1) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.414; 
                hNew = calculateHValue(i-1, j-1, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i-1][j-1].f == FLT_MAX || 
                        cellDetails[i-1][j-1].f > fNew) 
                { 
                    openList.insert( make_pair (fNew, make_pair (i-1, j-1))); 
                    // Update the details of this cell 
                    cellDetails[i-1][j-1].f = fNew; 
                    cellDetails[i-1][j-1].g = gNew; 
                    cellDetails[i-1][j-1].h = hNew; 
                    cellDetails[i-1][j-1].parent_i = i; 
                    cellDetails[i-1][j-1].parent_j = j; 
                } 
            } 
        } 
  
        //----------- 7th Successor (South-East) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid(i+1, j+1) == true) 
        { 
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i+1, j+1) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i+1][j+1].parent_i = i; 
                cellDetails[i+1][j+1].parent_j = j; 
                printf ("The destination cell is found\n"); 
                tracePath (cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
  
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i+1][j+1] == false && 
                     isUnBlocked(grid, i+1, j+1) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.414; 
                hNew = calculateHValue(i+1, j+1, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i+1][j+1].f == FLT_MAX || 
                        cellDetails[i+1][j+1].f > fNew) 
                { 
                    openList.insert(make_pair(fNew,  
                                        make_pair (i+1, j+1))); 
  
                    // Update the details of this cell 
                    cellDetails[i+1][j+1].f = fNew; 
                    cellDetails[i+1][j+1].g = gNew; 
                    cellDetails[i+1][j+1].h = hNew; 
                    cellDetails[i+1][j+1].parent_i = i; 
                    cellDetails[i+1][j+1].parent_j = j; 
                } 
            } 
        } 
  
        //----------- 8th Successor (South-West) ------------ 
  
        // Only process this cell if this is a valid one 
        if (isValid (i+1, j-1) == true) 
        { 
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i+1, j-1) == true) 
            { 
                // Set the Parent of the destination cell 
                cellDetails[i+1][j-1].parent_i = i; 
                cellDetails[i+1][j-1].parent_j = j; 
                printf("The destination cell is found\n"); 
                tracePath(cellDetails, dest); 
                foundDest = true; 
                return; 
            } 
  
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i+1][j-1] == false && 
                     isUnBlocked(grid, i+1, j-1) == true) 
            { 
                gNew = cellDetails[i][j].g + 1.414; 
                hNew = calculateHValue(i+1, j-1, dest); 
                fNew = gNew + hNew; 
  
                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i+1][j-1].f == FLT_MAX || 
                        cellDetails[i+1][j-1].f > fNew) 
                { 
                    openList.insert(make_pair(fNew,  
                                        make_pair(i+1, j-1))); 
  
                    // Update the details of this cell 
                    cellDetails[i+1][j-1].f = fNew; 
                    cellDetails[i+1][j-1].g = gNew; 
                    cellDetails[i+1][j-1].h = hNew; 
                    cellDetails[i+1][j-1].parent_i = i; 
                    cellDetails[i+1][j-1].parent_j = j; 
                } 
            } 
        } 
    } 
  
    // When the destination cell is not found and the open 
    // list is empty, then we conclude that we failed to 
    // reach the destiantion cell. This may happen when the 
    // there is no way to destination cell (due to blockages) 
    if (foundDest == false) 
        printf("Failed to find the Destination Cell\n"); 
  
    return; 
} 

void binary_grid()
{
    for (int i = 0; i < ROWS; i++)
    { 
	for (int j = 0; j < COLS; j++)
	{
            b_grid[i][j] = 2;
	    path_grid[i][j] == 0;
	}
    }
}

void reset_path()
{
    for (int i = 0; i < ROWS; i++)
    { 
	for (int j = 0; j < COLS; j++)
	{
     if (b_grid[i][j] == 0)
    {
	b_grid[i][j] = 0;
        viz_hit(normalize(i), normalize(j));
    }

    if (b_grid[i][j] == 1)
    { 
	viz_miss(normalize(i), normalize(j));
    }

    if (b_grid[i][j] == 2)
    {
	viz_unex(normalize(i), normalize(j));
    }

    if (path_grid[i][j] == 1)
    {
	path_grid[i][j] = 0;
	viz_unex(normalize(i), normalize(j));
    }

	    }
	}
    }

void astar_thread()
{
    while (true)
    {
	if (remain_count >= 50)
	{
	    traj_ready = false;
	    reset_path();

	    pair <int, int> normalized_coord;
	    normalized_coord.first = round((prev_coord.first + 27)*4);
	    normalized_coord.second = round((prev_coord.second + 27)*4);

	    pair <int, int> normalized_goal;
	    normalized_goal.first = (goal_x + 27)*4;
	    normalized_goal.second = (goal_y + 27)*4;

	    aStarSearch(b_grid, normalized_coord, normalized_goal);
	    cout << "x path: " <<  normalized_coord.first << endl << "y path: " << normalized_coord.second << endl;

	    cout << " the b: " << b_grid[normalized_coord.first][normalized_coord.second] << endl;
	    traj_ready = true;

	    sleep(5);
	}
    }
}

void robot_thread(Robot* robot)
{
    robot->do_stuff();
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    initialize_grid();
    binary_grid();
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    std::thread astarthr(astar_thread);

    return viz_run(argc, argv);
}
