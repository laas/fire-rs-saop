#include <iostream>

#include "../ext/dubins.h"
#include "../trajectory.h"
#include "../raster.h"
#include "../visibility.h"
#include "../planning.h"

int printConfiguration(double q[3], double x, void* user_data) {
    printf("%f,%f,%f,%f\n", q[0], q[1], q[2], x);
    return 0;
}

UAV uav(10., 32.*M_PI/180);

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

int main()
{
    srand(time(0));



    double q0[] = { 0,0,0 };
    double q1[] = { 4, 0, 0 };
    DubinsPath path;
    dubins_init( q0, q1, 1.0, &path);

    printf("%f \n", dubins_path_length(&path));

    test_single_point_to_observe();
    test_many_points_to_observe();

    return 0;
}