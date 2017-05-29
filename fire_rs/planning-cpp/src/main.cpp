#include <iostream>

#include "dubins.h"
#include "trajectory.h"
#include "local_search.h"
#include <stdio.h>
#include <math.h>
#include <cassert>

int printConfiguration(double q[3], double x, void* user_data) {
    printf("%f,%f,%f,%f\n", q[0], q[1], q[2], x);
    return 0;
}

int main()
{
    srand(time(0));

    UAV uav(1,1);

    double q0[] = { 0,0,0 };
    double q1[] = { 4, 0, 0 };
    DubinsPath path;
    dubins_init( q0, q1, 1.0, &path);

    printf("%f \n", dubins_path_length(&path));

    std::vector<Segment> waypoints = { Segment(Waypoint(0,0,0)), Waypoint(4, 0, 0) };
    Trajectory traj(uav, waypoints);
    printf("%f\n", traj.length());

    Trajectory t(uav);
    auto t2 = t.with_additional_segment(0, Segment(Waypoint(0, 0, 0)));
    assert(t2.length() == 0);
    auto t3 = t2.with_additional_segment(1, Segment(Waypoint(4, 0, 0)));
    assert(t3.length() == traj.length());
    auto t4 = t3.without_segment(1);
    assert(t4.length() == t2.length());

    printf("%f\n", uav.travel_distance(Waypoint(0,0,0), Waypoint(10,10,0)));
    printf("%f\n", uav.travel_distance(Waypoint(0,0,0), Waypoint(10,10,0)));

    std::vector<Segment> wps = { Waypoint(4,3,1.4), Segment(Waypoint(0,0,1)), Waypoint(10,10,0), Waypoint(4, 0, 0) };
    Trajectory t_init(uav, wps);
    printf("Before: %f\n", t_init.length());
    OrientationChangeNeighborhood gen1;
    TwoOrientationChangeNeighborhood gen2;
    Trajectory t_final = first_improvement_search(t_init, gen2, 20000);
    printf("After:  %f\n %s", t_final.length(), t_final.to_string().c_str());



//    printf("#x,y,theta,t\n");
//    dubins_path_sample_many( &path, printConfiguration, 0.1, NULL);

//    for(int i=0 ; i< 100 ; i++) {
//        q1[2] = 0.1 * i;
//        dubins_init(q0, q1, 1.0, &path);
//        printf("%f \n", dubins_path_length(&path));
//    }

    return 0;
}