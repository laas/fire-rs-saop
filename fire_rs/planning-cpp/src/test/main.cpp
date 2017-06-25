#include <iostream>

#include "../ext/dubins.h"
#include "../trajectory.h"
#include "../raster.h"
#include "../visibility.h"
#include "../planning.h"
#include "../neighborhoods/dubins_optimization.h"
#include "../fire_data.h"

UAV uav(10., 32.*M_PI/180);

void test_single_point_to_observe() {
    // all points ignited at time 0, except ont at time 100
    Raster ignitions(100, 100, 0, 0, 25);
    ignitions.set(10, 10, 100);

    // only interested in the point ignited at time 100
    Visibility visibility(ignitions, 90, 110);
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 150) };
    Plan p(confs, visibility);


    DefaultVnsSearch vns;

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

    DefaultVnsSearch vns;

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

void test_projection_on_firefront() {
    // uniform propagation along the y axis
    {
        Raster ignitions(100, 100, 0, 0, 1);
        for (size_t x = 0; x < 100; x++) {
            for (size_t y = 0; y < 100; y++) {
                ignitions.set(x, y, y);
            }
        }

        FireData fd(ignitions);
        auto res = fd.project_on_fire_front(Cell{1, 1}, 50.5);
        ASSERT(res && res->y == 50)


        auto res_back = fd.project_on_fire_front(Cell{79, 1}, 50.5);
        ASSERT(res_back && res_back->y == 50)
    }

    // uniform propagation along the x axis
    {
        Raster ignitions(10, 10, 0, 0, 1);
        for (size_t x = 0; x < 10; x++) {
            for (size_t y = 0; y < 10; y++) {
                ignitions.set(x, y, x);
            }
        }

        FireData fd(ignitions);
        auto res = fd.project_on_fire_front(Cell{1, 1}, 5.5);
        ASSERT(res && res->x == 5)


        auto res_back = fd.project_on_fire_front(Cell{7, 1}, 5.5);
        ASSERT(res_back && res_back->x == 5)
    }
    // circular propagation center on (50,50)
    {
        auto dist = [](size_t x, size_t y) { return sqrt(pow((double) x - 50., 2.) + pow((double) y - 50., 2.)); };
        Raster ignitions(100, 100, 0, 0, 1);
        for (size_t x = 0; x < 100; x++) {
            for (size_t y = 0; y < 100; y++) {
                ignitions.set(x, y, dist(x, y));

            }
        }
        FireData fd(ignitions);
        for(size_t i=0; i<100; i++) {
            const size_t x = rand(0, 100);
            const size_t y = rand(0, 100);
            auto res = fd.project_on_fire_front(Cell{x, y}, 25);
            ASSERT(res && abs(dist(res->x, res->y) - 25) <1.5)
        }
    }
}

int main()
{
    srand(time(0));

    test_segment_rotation();
    test_single_point_to_observe();
    test_many_points_to_observe();
    test_projection_on_firefront();
    return 0;
}