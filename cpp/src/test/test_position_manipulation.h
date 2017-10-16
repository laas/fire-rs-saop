#include <iostream>

#include "../ext/dubins.h"
#include "../trajectory.h"
#include "../raster.h"
#include "../planning.h"
#include "../uav.h"
#include "../neighborhoods/dubins_optimization.h"
#include "../fire_data.h"

UAV uav(10., 32.*M_PI/180, 0.1);

void test_single_point_to_observe() {
    // all points ignited at time 0, except ont at time 100
    DRaster ignitions(100, 100, 0, 0, 25);
    ignitions.set(10, 10, 100);

    auto fd = make_shared<FireData>(ignitions,ignitions);

    // only interested in the point ignited at time 100
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 100) };
    Plan p(confs, fd, TimeWindow{90, 110});


    DefaultVnsSearch vns;

    auto res = vns.search(p, 0, 1);
    ASSERT(res.final_plan)
    Plan solution = res.final();

    cout << "SUCCESS" << endl;
}

void test_many_points_to_observe() {
    // circular firedata spread
    DRaster ignitions(100, 100, 0, 0, 1);
    for (size_t x = 0; x < 100; x++) {
        for (size_t y = 0; y < 100; y++) {
            ignitions.set(x, y, sqrt(pow((double) x-50,2) + pow((double) y-50, 2)));
        }
    }

    auto fd = make_shared<FireData>(ignitions, ignitions);
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 10) };
    Plan p(confs, fd, TimeWindow{0, 110});

    DefaultVnsSearch vns;

    auto res = vns.search(p, 0, 1);
    ASSERT(res.final_plan)
    Plan solution = res.final();

    cout << "SUCCESS" << endl;
}

void test_many_points_to_observe_with_start_end_positions() {
    Waypoint3d start(5,5,0,0);
    Waypoint3d end(11,11,0,0);

    // circular firedata spread
    DRaster ignitions(100, 100, 0, 0, 1);
    for (size_t x = 0; x < 100; x++) {
        for (size_t y = 0; y < 100; y++) {
            ignitions.set(x, y, sqrt(pow((double) x-50,2) + pow((double) y-50, 2)));
        }
    }

    auto fd = make_shared<FireData>(ignitions, ignitions);
    vector<TrajectoryConfig> confs { TrajectoryConfig(
            uav,
            start,
            end,
            10) };
    Plan p(confs, fd, TimeWindow{0, 110});

    DefaultVnsSearch vns;

    auto res = vns.search(p, 0, 1);
    ASSERT(res.final_plan)
    Plan solution = res.final();

    auto& traj = solution.trajectories[0];
//    ASSERT(traj[0] == start);
//    ASSERT(traj[traj.size()-1] == end);
    ASSERT(traj.first_modifiable() == 1);
    ASSERT(traj.last_modifiable() == traj.size()-2);

    cout << "SUCCESS" << endl;
}


void test_segment_rotation() {
    for(size_t i=0; i<100; i++) {
        Waypoint wp(drand(-100000, 10000), drand(-100000, 100000), drand(-10*M_PI, 10*M_PI));
        Segment seg(wp, drand(0, 1000));

        Segment seg_rotated = uav.rotate_on_visibility_center(seg, drand(-10*M_PI, 10*M_PI));
        Segment seg_back = uav.rotate_on_visibility_center(seg_rotated, wp.dir);
        ASSERT(seg == seg_back)
    }
}

void test_projection_on_firefront() {
    // uniform propagation along the y axis
    {
        DRaster ignitions(100, 100, 0, 0, 1);
        for (size_t x = 0; x < 100; x++) {
            for (size_t y = 0; y < 100; y++) {
                ignitions.set(x, y, y);
            }
        }

        FireData fd(ignitions, ignitions);
        auto res = fd.project_on_fire_front(Cell{1, 1}, 50.5);
        ASSERT(res && res->y == 50)


        auto res_back = fd.project_on_fire_front(Cell{79, 1}, 50.5);
        ASSERT(res_back && res_back->y == 50)
    }

    // uniform propagation along the x axis
    {
        DRaster ignitions(10, 10, 0, 0, 1);
        for (size_t x = 0; x < 10; x++) {
            for (size_t y = 0; y < 10; y++) {
                ignitions.set(x, y, x);
            }
        }

        FireData fd(ignitions, ignitions);
        auto res = fd.project_on_fire_front(Cell{1, 1}, 5.5);
        ASSERT(res && res->x == 5)


        auto res_back = fd.project_on_fire_front(Cell{7, 1}, 5.5);
        ASSERT(res_back && res_back->x == 5)
    }
    // circular propagation center on (50,50)
    {
        auto dist = [](size_t x, size_t y) { return sqrt(pow((double) x - 50., 2.) + pow((double) y - 50., 2.)); };
        DRaster ignitions(100, 100, 0, 0, 1);
        for (size_t x = 0; x < 100; x++) {
            for (size_t y = 0; y < 100; y++) {
                ignitions.set(x, y, dist(x, y));

            }
        }
        FireData fd(ignitions, ignitions);
        for(size_t i=0; i<100; i++) {
            const size_t x = rand(0, 100);
            const size_t y = rand(0, 100);
            auto res = fd.project_on_fire_front(Cell{x, y}, 25);
            ASSERT(res && abs(dist(res->x, res->y) - 25) <1.5)
        }
    }
}

void test_trajectory_as_waypoints() {
    Trajectory traj((TrajectoryConfig(uav)));
    traj.sampled(2);
}

void all_position_manipulation()
{
    srand(time(0));

    test_trajectory_as_waypoints();
    test_segment_rotation();
    test_single_point_to_observe();
    test_many_points_to_observe();
    test_many_points_to_observe_with_start_end_positions();
    test_projection_on_firefront();
}