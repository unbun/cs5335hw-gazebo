
#include <iostream>
#include <stdio.h>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

const double straight_deadpan_p = 0.005;
const double straight_deadpan_n = -0.005;

// this isn't exactly how these constants work w/ pid, but 
// they were inspired by pid concepts
float err_avoid_i;
const double ki_avoid = 1.1f;
const double kff_t = 0.1;

float err_goal_i;
const double ki_goal = 1.1f;

void
callback(Robot* robot)
{

    // Check If the robot has reached the goal
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    // Steer the robot to point towards the goal (on the global y-axis)
    double y_offset = (goal_y - robot->pos_y);
    double x_offset = goal_x - robot->pos_x;
    double goal_t = atan2(y_offset, x_offset);
    double turn_error = (goal_t - robot->pos_t);

    bool front_46_hit = false;
    bool right_30_hit = false;
    bool left_30_hit = false;


    for (LaserHit hit : robot->hits) {
        float t_deg = hit.angle * (180.0/M_PI);

        if (hit.range <= 5.0){
            if(t_deg < 23 && t_deg > -23) {
                front_46_hit = true;
            }

            if(t_deg >=23 && t_deg < 53) {
                left_30_hit = true;
            }

            if(t_deg <= -23 && t_deg > -52) {
                right_30_hit = true;
            }
        }
    }

    if(front_46_hit && !right_30_hit) {
        // turn right
        printf("\n avoid right\n");
        robot->set_vel(6.0);
        robot->set_turn(0.08);

    } else if (front_46_hit && !left_30_hit) {
        // turn left
        printf("\n avoid left\n");
        robot->set_vel(6.0);
        robot->set_turn(-0.08);

    } else if (front_46_hit){
        // turn right _hard_
        printf("\n really avoid right\n");
        robot->set_vel(6.0);
        robot->set_turn(0.2);

    } else if(turn_error > 0.01){
        // point to the target (to the left)
        printf("\n steer left\n");
        robot->set_vel(6.0);
        robot->set_turn(-0.08);

    } else if(turn_error < 0.01){
        // point to the target (to the right)
        printf("\n steer right\n");
        robot->set_vel(6.0);
        robot-> set_turn(0.08);

    }
   
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
