
#include <iostream>
#include <stdio.h>
#include <math.h>

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

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

    // Determine how to steer the robot to point towards the goal
    double y_offset = (goal_y - robot->pos_y);
    double x_offset = goal_x - robot->pos_x;
    double goal_t = atan2(y_offset, x_offset);
    double turn_error = (goal_t - robot->pos_t);

    // Determine how to avoid obstacles with the lidar sensor
    bool front_46_hit = false; // 45, but even
    bool right_30_hit = false;
    bool left_30_hit = false;

    for (LaserHit hit : robot->hits) {
        float t_deg = hit.angle * (180.0 / M_PI); // less mental math for myself 

        // since we're going faster and aren't tuning, we should check a bit further a head of ourselves
        // to start turning as soon as we can
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

    // If we need to avoid something, we should do that, otherwise try to best to get to the target
    // 
    // Uses a bunch of constants/magic numbers. Ideally, these would actually be calc-ed with a 
    // tuned PID controller based on rolling error... but that feels like a lot for assignment 1

    if(front_46_hit && !right_30_hit) {
        // turn right
        // printf("\n avoid right\n");
        robot->set_vel(9.0);
        robot->set_turn(0.12);

    } else if (front_46_hit && !left_30_hit) {
        // turn left
        // printf("\n avoid left\n");
        robot->set_vel(9.0);
        robot->set_turn(-0.12);

    } else if (front_46_hit){
        // turn right _hard_
        // printf("\n really avoid right\n");
        robot->set_vel(9.0);
        robot->set_turn(0.24);

    } else if(turn_error > 0.01){
        // point to the target (to the left)
        // printf("\n steer left\n");
        robot->set_vel(9.0);
        robot->set_turn(-0.12);

    } else if(turn_error < 0.01){
        // point to the target (to the right)
        // printf("\n steer right\n");
        robot->set_vel(9.0);
        robot-> set_turn(0.12);

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
