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

int main()
{
    srand(time(0));

    UAV uav(18., 32.*M_PI/180);

    double q0[] = { 0,0,0 };
    double q1[] = { 4, 0, 0 };
    DubinsPath path;
    dubins_init( q0, q1, 1.0, &path);

    printf("%f \n", dubins_path_length(&path));

    Raster ignitions(100, 100, 0, 0, 25);
    Visibility visibility(ignitions, 0, 100);
    vector<TrajectoryConfig> confs { TrajectoryConfig(uav, 200) };
    Plan p(confs, visibility);


    vector<shared_ptr<Neighborhood>> neighborhoods;
    neighborhoods.push_back(make_shared<OneInsertNbhd>());

    VariableNeighborhoodSearch vns(neighborhoods);

    auto res = vns.search(p, 0, 1);


    return 0;
}