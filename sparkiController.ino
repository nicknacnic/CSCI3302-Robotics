//Robot side code for project: Teach robot to love
//made use of lab6 code
#include <sparki.h>
#define TRUE 1
#define FALSE 0
#define DISTANCE_MARGIN 0.02 // 2cm of tolerance
#define HEADING_MARGIN 0.061 // ~3.5 degrees of tolerance
#define ROBOT_SPEED 0.0278
#define CYCLE_TIME .100
#define AXLE_DIAMETER 0.0865
#define M_PI 3.14159
#define WHEEL_RADIUS 0.03
#define FWD 1
#define NONE 0
#define BCK -1
#define NUM_X_CELLS 4
#define NUM_Y_CELLS 4
#define MAP_SIZE_X 0.6
#define MAP_SIZE_Y 0.42
#define BIG_NUMBER 255
/**********************************
*          MACHINE STATES         *
**********************************/
#define STATE_START 0
#define STATE_HAS_PATH 1
#define STATE_SEEKING_POSE 2
#define STATE_FINISH 3
#define STATE_WAIT_FOR_COMMAND 4
#define STATE_GRIPPER_CLOSE 5

short prev_holder[NUM_X_CELLS*NUM_Y_CELLS];
// IK/Odometry Variables
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0., phi_l = 0., phi_r = 0.;
// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;
bool world_map[NUM_Y_CELLS][NUM_X_CELLS];
bool goal_changed = TRUE; // Track if the goal coordinates have been changed since we last path planned.
int goal_i = 0;
int goal_j = 0;
int source_i = 0;
int source_j = 0;
short *prev = NULL;
short *path = NULL;
long program_start_time = 0; // Track time since controller began
/**********************************
*        PROJECT VARIABLES          *
**********************************/
int machine_state;
int curr_i;
int curr_j;
float curr_x;
float curr_y;
int curr_index;
int dest_theta;
bool final_path_contains_source = FALSE;
int goal_index;
String holding;

void setup()
{
  // IK Setup
  sparki.gripperOpen();
  delay(5000);
  sparki.gripperStop();
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  // Dijkstra Setup
  for (int j = 0; j < NUM_Y_CELLS; ++j)
  {
    for (int i = 0; i < NUM_X_CELLS; ++i)
    {
      world_map[j][i] = 1;
    }
  }
  program_start_time = millis();
  machine_state = STATE_WAIT_FOR_COMMAND;
}

/*****************************
*    IK Helper Functions     *
*****************************/
void set_pose_destination(float x, float y, float t)
{
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
}

float to_radians(double deg)
{
  return  deg * 3.1415/180.;
}

float to_degrees(double rad)
{
  return  rad * 180 / 3.1415;
}

void updateOdometry()
{
  pose_x += cos(pose_theta) * CYCLE_TIME * ROBOT_SPEED 
  * (float(left_wheel_rotating)*left_speed_pct 
    + float(right_wheel_rotating)*right_speed_pct)/2.;
  pose_y += sin(pose_theta) * CYCLE_TIME * ROBOT_SPEED 
  * (float(left_wheel_rotating)*left_speed_pct 
    + float(right_wheel_rotating)*right_speed_pct)/2.;
  pose_theta += (float(right_wheel_rotating)*right_speed_pct 
         - float(left_wheel_rotating)*left_speed_pct)  
      * CYCLE_TIME * ROBOT_SPEED / AXLE_DIAMETER;
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
}

/*****************************
* Core IK Functions         *
*****************************/
void moveStop()
{
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  left_speed_pct = 0.0;
  right_speed_pct = 0.0;
  sparki.moveStop();
}
void compute_IK_errors()
{
  // Distance, Bearing, and Heading error
  d_err = sqrt( (dest_pose_x-pose_x)*(dest_pose_x-pose_x) + (dest_pose_y-pose_y)*(dest_pose_y-pose_y) );
  b_err = atan2( (dest_pose_y - pose_y), (dest_pose_x - pose_x) ) - pose_theta;
  h_err = dest_pose_theta - pose_theta;

  if (b_err <= -M_PI) b_err += 2.*M_PI;
  else if (b_err > M_PI) b_err -= 2.*M_PI;

  if (h_err <= -M_PI) h_err += 2.*M_PI;
  else if (h_err > M_PI) h_err -= 2.*M_PI;  
}
void compute_IK_wheel_rotations()
{
float dTheta, dX;
  if (d_err > DISTANCE_MARGIN)
  { // Get reasonably close before considering heading error
    dTheta = b_err;
  }
  else
  {
    dTheta = h_err;
    dX = 0.; // Force 0-update for dX since we're supposedly at the goal position anyway
  }  

  dX  = 0.1 * min(M_PI, d_err); // De-prioritize distance error to help avoid paths through unintended grid cells

  phi_l = (dX  - (dTheta*AXLE_DIAMETER/2.)) / WHEEL_RADIUS;
  phi_r = (dX  + (dTheta*AXLE_DIAMETER/2.)) / WHEEL_RADIUS;
}

void set_IK_motor_rotations()
{
  float wheel_rotation_normalizer = max(abs(phi_l), abs(phi_r));
  if (wheel_rotation_normalizer == 0)
  {
    moveStop();
    return;
  }
  left_speed_pct = abs(phi_l) / wheel_rotation_normalizer;
  right_speed_pct = abs(phi_r) / wheel_rotation_normalizer;

  // Figure out which direction the wheels need to rotate
  if (phi_l <= -to_radians(1))
  {
    left_dir = DIR_CW;
    left_wheel_rotating = BCK;
  }
  else if (phi_l >= to_radians(1))
  {
    left_dir = DIR_CCW;
    left_wheel_rotating = FWD;
  }
  else
  {
    left_speed_pct = 0.;
    left_wheel_rotating = 0;
  }

  if (phi_r <= -to_radians(1))
  {
    right_dir = DIR_CCW;
    right_wheel_rotating = BCK;
  }
  else if (phi_r >= to_radians(1))
  {
    right_dir = DIR_CW;
    right_wheel_rotating = FWD;
  }
  else
  {
    right_speed_pct = 0.;
    right_wheel_rotating = 0;
  }
  sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100));
  sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100));
}

bool is_robot_at_IK_destination_pose()
{
  return (d_err <= DISTANCE_MARGIN && abs(h_err) < HEADING_MARGIN);
}



/*****************************
* Dijkstra Helper Functions *
*****************************/

// Return 1 if there are entries in range [0,inf) in arr
// otherwise return 0, signifying empty queue
bool is_not_empty(short *arr, int len)
{
  for (int i=0; i < len; ++i)
  {
    if (arr[i] >= 0)
    {
      return TRUE;
    }
  }
  return FALSE;
}

int get_min_index(short *arr, int len)
{
  int min_idx=0;
  for (int i=0;i < len; ++i)
  {
    if (arr[min_idx] < 0 || (arr[i] < arr[min_idx] && arr[i] >= 0))
    {
      min_idx = i;
    }
  }
  if (arr[min_idx] == -1)
    return -1; 
  return min_idx;
}


/**********************************
* Coordinate Transform Functions *
**********************************/

// Returns 0 if something went wrong -- assume invalid i and j values being set
bool vertex_index_to_ij_coordinates(int v_idx, int *i, int *j)
{
  *i = v_idx % NUM_X_CELLS;
  *j = v_idx / NUM_X_CELLS;

  if (*i < 0 || *j < 0 || *i >= NUM_X_CELLS || *j >= NUM_Y_CELLS)
    return FALSE;
  return TRUE;
}

int ij_coordinates_to_vertex_index(int i, int j)
{
  return j*NUM_X_CELLS + i;  
}

// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. x and y values are the middle of cell (i,j)
bool ij_coordinates_to_xy_coordinates(int i, int j, float *x, float *y)
{
  if (i < 0 || j < 0 || i >= NUM_X_CELLS || j >= NUM_Y_CELLS)
    return FALSE;

  *x = (i+0.5)*(MAP_SIZE_X/NUM_X_CELLS);
  *y = (j+0.5)*(MAP_SIZE_Y/NUM_Y_CELLS);
  return TRUE;  
}

// Returns 0 if something went wrong -- assume invalid x and y values are being set
// Returns 1 otherwise. x and y values are the middle of cell (i,j)
bool xy_coordinates_to_ij_coordinates(float x, float y, int *i, int *j)
{
  if (x < 0 || y < 0 || x >= MAP_SIZE_X || y >= MAP_SIZE_Y) return FALSE;

  *i = int((x/MAP_SIZE_X) * NUM_X_CELLS);
  *j = int((y/MAP_SIZE_Y) * NUM_Y_CELLS);

  return TRUE;  
}


/**********************************
*     HELPER FUNCTIONS            *
**********************************/
float calculateNextTheta(int NextVertex, int NextNextVertex)
{
  int next_i;
  int next_j;
  float next_x;
  float next_y;
  int nextnext_i;
  int nextnext_j;
  float nextnext_x;
  float nextnext_y;
  vertex_index_to_ij_coordinates(NextNextVertex, &nextnext_i, &nextnext_j);
  ij_coordinates_to_xy_coordinates(nextnext_i, nextnext_j, &nextnext_x, &nextnext_y);
  vertex_index_to_ij_coordinates(NextVertex, &next_i, &next_j);
  ij_coordinates_to_xy_coordinates(next_i, next_j, &next_x, &next_y);
  float avg_theta = atan2( abs(next_y - nextnext_y), abs(next_x - nextnext_x));
  return avg_theta;
}

/**********************************
*      Core Dijkstra Functions   *
**********************************/


// Returns the cost of moving from vertex_source to vertex_dest
byte get_travel_cost(int vertex_source, int vertex_dest)
{
  bool are_neighboring = 1;

  int s_i, s_j; // Source i,j
  int d_i, d_j; // Dest i,j

  if (!vertex_index_to_ij_coordinates(vertex_source, &s_i, &s_j))
    return BIG_NUMBER;
  if (!vertex_index_to_ij_coordinates(vertex_dest, &d_i, &d_j))
    return BIG_NUMBER;

  are_neighboring = (abs(s_i - d_i) + abs(s_j - d_j) <= 1); // 4-Connected world

  // TODO: Add your code to incorporate world_map here

  if(world_map[d_j][d_i] == 0)
    return BIG_NUMBER;

  if (are_neighboring)
    return 1;
  else
    return BIG_NUMBER;
}

short *run_dijkstra(bool world_map[NUM_Y_CELLS][NUM_X_CELLS], int source_vertex)
{
  int cur_vertex = -1;
  int cur_cost = -1;
  int min_index = -1;
  int travel_cost = -1;
  byte dist[NUM_X_CELLS*NUM_Y_CELLS];
  short Q_cost[NUM_Y_CELLS*NUM_X_CELLS];
  short *prev = prev_holder;

  // Initialize our variables
  for (int j = 0; j < NUM_Y_CELLS; ++j)
  {
    for (int i = 0; i < NUM_X_CELLS; ++i)
    {
      Q_cost[j*NUM_X_CELLS + i] = BIG_NUMBER;
      dist[j*NUM_X_CELLS + i] = BIG_NUMBER;
      prev[j*NUM_X_CELLS + i] = -1;
    }
  }
  // Initialize source vertex's distance to 0
  dist[source_vertex] = 0;
  Q_cost[source_vertex] = 0;

  while (is_not_empty(Q_cost,NUM_X_CELLS*NUM_Y_CELLS) == 1)
  {
    min_index = get_min_index(Q_cost, NUM_X_CELLS*NUM_Y_CELLS);
    if (min_index < 0)
    {
      break; // Queue is empty!
    }
    cur_vertex = min_index; // Vertex ID is same as array indices
    cur_cost = Q_cost[cur_vertex]; // Current best cost for reaching vertex
    Q_cost[cur_vertex] = -1; // Remove cur_vertex from the queue.

    // Iterate through all of node's neighbors and see if cur_vertex provides a shorter
    // path to the neighbor than whatever route may exist currently.
    for (byte neighbor_idx = 0; neighbor_idx < NUM_X_CELLS*NUM_Y_CELLS; ++neighbor_idx)
    {
      if (Q_cost[neighbor_idx] == -1) continue; // Skip neighboring nodes that are not in the Queue

      short alt = -1;
      travel_cost = get_travel_cost(cur_vertex, neighbor_idx);
      if (travel_cost == BIG_NUMBER)
      {
        continue; // Nodes are not neighbors/cannot traverse... skip!
      }
      alt = dist[cur_vertex] + travel_cost;
      if (alt < dist[neighbor_idx])
      {
        // New shortest path to node at neighbor_idx found!
        dist[neighbor_idx] = alt;
        if (Q_cost[neighbor_idx] > 0)
        Q_cost[neighbor_idx] = alt;
        prev[neighbor_idx] = cur_vertex;
      }
    }
  }
  return prev;
}

short *reconstruct_path(short *prev, int source_vertex, int dest_vertex)
{
  short path[NUM_X_CELLS * NUM_Y_CELLS]; // Max-length that path could be
  short *final_path = NULL;
  int last_idx = 0;

  // After using last_idx, increment 1
  path[last_idx++] = dest_vertex; // Start at goal, work backwards
  short last_vertex = prev[dest_vertex];
  // Last(Previous) vertex would not be -1 unless it is has been visited
  while (last_vertex != -1)
  {
    path[last_idx++] = last_vertex;
    last_vertex = prev[last_vertex];
  }

  final_path = new short[last_idx+1];
  for (int i = 0; i < last_idx; ++i)
  {
    final_path[i] = path[last_idx-1-i]; // Reverse path so it goes source->dest
  }
  final_path[last_idx] = -1; // End-of-array marker
  if(final_path[0] == ij_coordinates_to_vertex_index(source_i,source_j))
  {
    final_path_contains_source = TRUE;
  }
  goal_changed = FALSE;

  return final_path;
}
void displayOdometry()
{
  sparki.clearLCD();
  sparki.print("Goal index is: ");
  sparki.print(goal_index);
  sparki.updateLCD();
}

void loop ()
{
  unsigned long begin_time = millis();
  unsigned long end_time = 0;

  updateOdometry();
  displayOdometry();
  switch(machine_state)
  { 
    case STATE_WAIT_FOR_COMMAND:
    //This case is for the sparki to wait for computer side input
    //Once computer side decided grid id for the object, it should send the id through port
     
     //If serial port received something
     if (Serial.available())
      {
        //Put grid id into holding
        int inByte = Serial.read();
        holding += (char)inByte;
        if(holding == "one"){
          goal_index = 1;
        }
        else if(holding == "two"){
          goal_index = 2;
        }
        else if(holding == "three"){
          goal_index = 3;
        }
        else if(holding == "four+"){
          goal_index = 4;
        }
        else if(holding == "five"){
          goal_index = 5;
        }
        else if(holding == "six"){
          goal_index = 6;
        }
        else if(holding == "seven"){
          goal_index = 7;
        }
        else if(holding == "eight"){
          goal_index = 8;
        }
        else if(holding == "nine"){
          goal_index = 9;
        }
        else if(holding == "ten"){
          goal_index = 10;
        }
        else if(holding == "eleven"){
          goal_index = 11;
        }
        else if(holding == "twelve"){
          goal_index = 12;
        }
        else if(holding == "thirteen"){
          goal_index = 13;
        }
        else if(holding == "fourteen"){
          goal_index = 14;
        }
        else if(holding == "fifteen"){
          goal_index = 15;
        }
      }
      //if we do received a valid grid id
      if(goal_index != 0 && goal_index < 16){
        //Translate the grid id into i,j coordinate
        vertex_index_to_ij_coordinates(goal_index, &goal_i, &goal_j);
        machine_state = STATE_START;
      }
      break;
  
    case STATE_START:
    //Construct a path to the given grid id
      if(path != NULL)
      {
        path = NULL;
        delete path;
      }

      xy_coordinates_to_ij_coordinates(pose_x, pose_y, &source_i, &source_j);
      prev = run_dijkstra(world_map, ij_coordinates_to_vertex_index(source_i, source_j)); 
      path = reconstruct_path(prev, ij_coordinates_to_vertex_index(source_i, source_j), ij_coordinates_to_vertex_index(goal_i, goal_j));

      if(final_path_contains_source)
      {
        curr_index = 0;
        machine_state = STATE_HAS_PATH;
      }
      else if(!final_path_contains_source)
      {
        sparki.clearLCD();
        sparki.println("ERROR: CANNOT REACH GOAL");
        sparki.updateLCD();
        delay(2000);
        machine_state = STATE_FINISH;
      }
      break;

    case STATE_HAS_PATH:
      if (goal_changed)
      {
        machine_state = STATE_START;
      }
      else if (path[curr_index] == -1)
      {
        moveStop();
        machine_state = STATE_GRIPPER_CLOSE;
      }
      else
      {
        if (path[curr_index+1] == -1)
        {
          dest_theta = 0;
          machine_state = STATE_SEEKING_POSE;
        }
        dest_theta = calculateNextTheta(path[curr_index],path[curr_index+1]);
        vertex_index_to_ij_coordinates(path[curr_index], &curr_i, &curr_j);
        ij_coordinates_to_xy_coordinates(curr_i, curr_j, &curr_x, &curr_y);
        set_pose_destination(curr_x, curr_y, dest_theta);
        machine_state = STATE_SEEKING_POSE;
      }
        break;
  

    case STATE_SEEKING_POSE:
      /*****************************************/
      // Use inverse kinematics functions to   //
      // navigate to the assigned goal pose    //
      //        the final path array           //
      /*****************************************/
      if(goal_changed)
      {
        machine_state = STATE_START;
      }
      else
      {
        compute_IK_errors();
        compute_IK_wheel_rotations();
        set_IK_motor_rotations();
        if (is_robot_at_IK_destination_pose())
        {
          moveStop();
          curr_index++;
          machine_state = STATE_HAS_PATH;
        }
      }
      break;

    case STATE_GRIPPER_CLOSE:
    //Once the sparki reach the goal position
    //Capture the object
      sparki.gripperClose();
      delay(5000);
      sparki.gripperStop(); 
      sparki.clearLCD();
      sparki.print("The object is here!");
      sparki.updateLCD();
      delay(2000);
      machine_state = STATE_FINISH;
      break;
      
    case STATE_FINISH:
      /*****************************************/
      // This is the final state,used only to  //
      // stop sparki and tell the user the lab //
      //          has fully completed          //
      /*****************************************/
      delete path;
      goal_index = 0;
      path=NULL;
      sparki.clearLCD();
      sparki.print("Project finished!");
      sparki.updateLCD();
      delay(2000);
      break;
  }

  end_time = millis();
  if (end_time - begin_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - (end_time - begin_time)); // each loop takes CYCLE_TIME ms
  else
    delay(10); // Accept some error
}
