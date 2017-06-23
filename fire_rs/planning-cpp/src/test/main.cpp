#include <iostream>

#include "../ext/dubins.h"
#include "../trajectory.h"
#include "../raster.h"
#include "../visibility.h"
#include "../planning.h"
#include "../neighborhoods/dubins_optimization.h"

UAV uav(10., 32.*M_PI/180);

double drand(double min, double max) {
    const double base = (double) rand() / RAND_MAX;
    return min + base * (max-min);
}

void test_single_point_to_observe() {

    // all points ignited at time 0, except ont at time 100
    Raster ignitions(100, 100, 0, 0, 25);
    ignitions.set(10, 10, 100);

    // only interested in the point ignited at time 100
    Visibility visibility(ignitions, 90, 110);
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 150) };
    Plan p(confs, visibility);


    vector<shared_ptr<Neighborhood>> neighborhoods;
    neighborhoods.push_back(make_shared<OneInsertNbhd>());

    VariableNeighborhoodSearch vns(neighborhoods);

    auto res = vns.search(p, 0, 1);
    ASSERT(res.final_plan)
    Plan solution = res.final();

    cout << "SUCCESS" << endl;
}

void test_many_points_to_observe() {
    // all points ignited at time 0, except ont at time 100
    Raster ignitions(10, 10, 0, 0, 25);

    // only interested in the point ignited at time 100
    Visibility visibility(ignitions, 0, 110);
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 150) };
    Plan p(confs, visibility);


    vector<shared_ptr<Neighborhood>> neighborhoods;
    neighborhoods.push_back(make_shared<OneInsertNbhd>());

    VariableNeighborhoodSearch vns(neighborhoods);

    auto res = vns.search(p, 0, 1);
    ASSERT(res.final_plan)
    Plan solution = res.final();

    cout << "SUCCESS" << endl;
}

void test_segment_rotation() {
    for(size_t i=0; i<100; i++) {
        Waypoint wp(drand(-100000, 10000), drand(-100000, 100000), drand(-10*M_PI, 10*M_PI));
        Segment seg(wp, drand(0, 1000));

        Segment seg_rotated = rotate_on_visibility_center(seg, uav, drand(-10*M_PI, 10*M_PI));
        Segment seg_back = rotate_on_visibility_center(seg_rotated, uav, wp.dir);
        ASSERT(seg == seg_back)
    }

    // simple UAV to simplify reasoning
    UAV uav(1., 32.*M_PI/180);
    uav.view_width = 1;
    uav.view_depth = 1;

    Waypoint wp(0, 0, 0);
    Segment seg(wp, 0);

    Segment seg_rotated = rotate_on_visibility_center(seg, uav, M_PI/2);
    ASSERT(ALMOST_EQUAL(seg_rotated.start.x, 0.5))
    ASSERT(ALMOST_EQUAL(seg_rotated.start.y, -0.5))
    ASSERT(ALMOST_EQUAL(seg_rotated.start.dir, M_PI/2))
    Segment seg_back = rotate_on_visibility_center(seg_rotated, uav, wp.dir);
    ASSERT(seg == seg_back)
}

int main()
{
    srand(time(0));

    test_segment_rotation();
    test_single_point_to_observe();
    test_many_points_to_observe();

    return 0;
}