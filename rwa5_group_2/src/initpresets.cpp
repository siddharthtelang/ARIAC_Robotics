#include "rwa_implementation.h"

#define GET_VARIABLE_NAME(Variable) (#Variable)

void RWAImplementation::initPresetLocs()
{
    conveyor_belt.gantry = {0, 3, PI / 2};
    conveyor_belt.left_arm = {-0.2, -PI / 4, PI / 2, -PI / 4, PI / 2 - 0.2, 0};
    conveyor_belt.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    conveyor_belt.name = GET_VARIABLE_NAME(conveyor_belt);

    // joint positions to go to start location
    start_a.gantry = {0, 0, 0};
    start_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    start_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    start_a.name = GET_VARIABLE_NAME(start_a);

    // joint positions to go to bin3
    bin3_a.gantry = {4.0, -1.1, 0.};
    bin3_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin3_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    bin3_a.name = GET_VARIABLE_NAME(bin3_a);

    // joint positions to go to bin11 (and all 8 bins in the entire grouping)
    bin11_a.gantry = {4.0, -1.1, 0.}; // the exact same as bin311_a
    bin11_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    bin11_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    bin11_a.name = GET_VARIABLE_NAME(bin11_a);

    // // rename this to general far pick
    // shelf11_south_far.gantry = {-13.52-0.172656, 1.96, -PI/2}; // WORKS FOR LEFT SHELF PULLEY NOT RIGHT
    // shelf11_south_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    // shelf11_south_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    // shelf11_south_far.name = GET_VARIABLE_NAME(shelf11_south_far);

    // rename this to general far pick
    shelf11_south_far.gantry = {-13.52-0.172656, 1.96, -PI/2}; // WORKS FOR LEFT SHELF PULLEY NOT RIGHT
    // shelf11_south_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf11_south_far.left_arm = {0.00, -2.06, 1.62, -2.73, -PI/2, 0.0};     // lower for better pick part speed
    // shelf11_south_far.left_arm = {0.00, -2.06, 1.61, -2.73, -PI/2, 0.0};     // lower for better pick part speed 1.62>1.61
    shelf11_south_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf11_south_far.name = GET_VARIABLE_NAME(shelf11_south_far);


    ///////////// Shelf Preset Locations: arranged in rows from Northernmost (world +y direction) row to Southernmost (world -y direction) row,
    ////////////                          each row is arranged from left (-x) to right (+x). This section is for the shelves only.

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 1
    bottom_left_staging_a.gantry = {-14.22, -6.75, 0.00};
    bottom_left_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    bottom_left_staging_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    bottom_left_staging_a.name = GET_VARIABLE_NAME(bottom_left_staging_a);

    waitpoint_best_north_fromNorth.gantry = {-11.4, -6.9, 0.88}; // waitpoint_best_north_fromNorth
    waitpoint_best_north_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_north_fromNorth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    waitpoint_best_north_fromNorth.name = GET_VARIABLE_NAME(waitpoint_best_north_fromNorth);

    waitpoint_best_north_fromSouth.gantry = {-11.4, -6.9, 0.88-PI}; // waitpoint_best_north_fromSouth
    waitpoint_best_north_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_north_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    waitpoint_best_north_fromSouth.name = GET_VARIABLE_NAME(waitpoint_best_north_fromSouth);

    // joint positions to go to agv1
    agv1_staging_a.gantry = {0.6, -6.9, 0.00};
    agv1_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    agv1_staging_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    agv1_staging_a.name = GET_VARIABLE_NAME(agv1_staging_a);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 2
    shelf5_a.gantry = {-14.42, -4.30, 0.00}; // todo why is this -14.42 instead of -14.22
    shelf5_a.left_arm = {-1.76, -1.00, 1.86, -.85, -.20, 0.0};
    shelf5_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf5_a.name = GET_VARIABLE_NAME(shelf5_a);

    shelf5_fromNorth_near.gantry = {-14.22, -4.30, 0.88}; // shelf5_fromNorth_near
    shelf5_fromNorth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf5_fromNorth_near.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf5_fromNorth_near.name = GET_VARIABLE_NAME(shelf5_fromNorth_near);


    shelf5_fromNorth_far.gantry = {-13.52-0.172656, -4.30, -PI/2}; // shelf5_fromNorth_far
    shelf5_fromNorth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf5_fromNorth_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf5_fromNorth_far.name = GET_VARIABLE_NAME(shelf5_fromNorth_far);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 3
    shelf8_a.gantry = {-14.22, -1.5, 0.00};
    shelf8_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf8_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf8_a.name = GET_VARIABLE_NAME(shelf8_a);


    shelf5_fromSouth_near.gantry = {-14.22, -1.5, 0.88-PI}; // shelf5_fromSouth_near
    shelf5_fromSouth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf5_fromSouth_near.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf5_fromSouth_near.name = GET_VARIABLE_NAME(shelf5_fromSouth_near);


    shelf5_fromSouth_far.gantry = {-13.52-0.172656, -1.5, -PI/2}; // shelf5_fromSouth_far
    shelf5_fromSouth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf5_fromSouth_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf5_fromSouth_far.name = GET_VARIABLE_NAME(shelf5_fromSouth_far);


    shelf8_fromNorth_near.gantry = {-14.22, -1.5, 0.88}; // shelf8_fromNorth_near
    shelf8_fromNorth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf8_fromNorth_near.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf8_fromNorth_near.name = GET_VARIABLE_NAME(shelf8_fromNorth_near);


    shelf8_fromNorth_far.gantry = {-13.52-0.172656, -1.5, -PI/2}; // shelf8_fromNorth_far
    shelf8_fromNorth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf8_fromNorth_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf8_fromNorth_far.name = GET_VARIABLE_NAME(shelf8_fromNorth_far);



    mid_5_8_intersection_fromNorth.gantry = {-11.4, -1.5, 0.88}; // Intersection!
    mid_5_8_intersection_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_intersection_fromNorth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_5_8_intersection_fromNorth.name = GET_VARIABLE_NAME(mid_5_8_intersection_fromNorth);

    mid_5_8_intersection_fromSouth.gantry = {-11.4, -1.5, 0.88-PI}; // mid_5_8_intersection_fromSouth
    mid_5_8_intersection_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_intersection_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_5_8_intersection_fromSouth.name = GET_VARIABLE_NAME(mid_5_8_intersection_fromSouth);

    mid_5_8_staging_a.gantry = {0.0, -1.5, 0.00};
    mid_5_8_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_5_8_staging_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_5_8_staging_a.name = GET_VARIABLE_NAME(mid_5_8_staging_a);


    mid_5_8_staging_fromNorth.gantry = {0.0, -1.5, 0.88}; // mid_5_8_staging_fromNorth
    mid_5_8_staging_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_staging_fromNorth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_5_8_staging_fromNorth.name = GET_VARIABLE_NAME(mid_5_8_staging_fromNorth);

    mid_5_8_staging_fromSouth.gantry = {0.0, -1.5, 0.88-PI}; // mid_5_8_staging_fromSouth
    mid_5_8_staging_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_5_8_staging_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_5_8_staging_fromSouth.name = GET_VARIABLE_NAME(mid_5_8_staging_fromSouth);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 4
    shelf11_a.gantry = {-14.22, 1.5, 0.00};
    shelf11_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    shelf11_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf11_a.name = GET_VARIABLE_NAME(shelf11_a);

    shelf8_fromSouth_near.gantry = {-14.22, 1.5, 0.88-PI}; // shelf8_fromSouth_near
    shelf8_fromSouth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf8_fromSouth_near.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf8_fromSouth_near.name = GET_VARIABLE_NAME(shelf8_fromSouth_near);

    // shelf8_fromSouth_far.gantry = {-13.52-0.172656, 1.5, PI/2}; // shelf8_fromSouth_far
    // shelf8_fromSouth_far.gantry = {-13.52-0.172656   -0.1-0.1   , 1.7+0.1, PI/2}; // shelf8_fromSouth_far //tuned for kick
    // shelf8_fromSouth_far.gantry = {-13.52-0.172656   -0.1-0.1   , 1.7, PI/2}; // shelf8_fromSouth_far //tuned for kick
    shelf8_fromSouth_far.gantry = {-13.52-0.172656, 1.7, PI/2}; // shelf8_fromSouth_far //tuned for kick
    // shelf8_fromSouth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf8_fromSouth_far.left_arm = {0.0, -3.58, 2.87, -4.65, -PI/2, 0.00}; // stowed fast
    // shelf8_fromSouth_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf8_fromSouth_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf8_fromSouth_far.name = GET_VARIABLE_NAME(shelf8_fromSouth_far);

    shelf11_fromNorth_near.gantry = {-14.22, 1.5, 0.88}; // shelf11_fromNorth_near
    shelf11_fromNorth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf11_fromNorth_near.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf11_fromNorth_near.name = GET_VARIABLE_NAME(shelf11_fromNorth_near);

    shelf11_fromNorth_far.gantry = {-13.52-0.172656, 1.5, -PI/2}; // shelf11_fromNorth_far
    shelf11_fromNorth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf11_fromNorth_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf11_fromNorth_far.name = GET_VARIABLE_NAME(shelf11_fromNorth_far);


    mid_8_11_intersection_fromNorth.gantry = {-11.4, 1.5, 0.88}; // mid_8_11_intersection_fromNorth
    mid_8_11_intersection_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_intersection_fromNorth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_8_11_intersection_fromNorth.name = GET_VARIABLE_NAME(mid_8_11_intersection_fromNorth);

    // mid_8_11_intersection_fromSouth.gantry = {-11.4, 1.5, 0.88-PI}; // mid_8_11_intersection_fromSouth
    mid_8_11_intersection_fromSouth.gantry = {-11.4, 1.5, PI/2}; // mid_8_11_intersection_fromSouth
    // mid_8_11_intersection_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_intersection_fromSouth.left_arm = {0.0, -3.58, 2.87, -4.65, -PI/2, 0.00}; // stowed fast
    // mid_8_11_intersection_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_8_11_intersection_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_8_11_intersection_fromSouth.name = GET_VARIABLE_NAME(mid_8_11_intersection_fromSouth);

    mid_8_11_staging_a.gantry = {0.0, 1.5, 0.00};
    mid_8_11_staging_a.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    mid_8_11_staging_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_8_11_staging_a.name = GET_VARIABLE_NAME(mid_8_11_staging_a);


    mid_8_11_staging_fromNorth.gantry = {0.0, 1.5, 0.88}; // mid_8_11_staging_fromNorth
    mid_8_11_staging_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_staging_fromNorth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_8_11_staging_fromNorth.name = GET_VARIABLE_NAME(mid_8_11_staging_fromNorth);

    mid_8_11_staging_fromSouth.gantry = {0.0, 1.5, 0.88-PI}; // mid_8_11_staging_fromSouth
    mid_8_11_staging_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    mid_8_11_staging_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    mid_8_11_staging_fromSouth.name = GET_VARIABLE_NAME(mid_8_11_staging_fromSouth);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 5

    shelf11_fromSouth_near.gantry = {-14.22, 4.30, 0.88-PI}; // shelf11_fromSouth_near
    shelf11_fromSouth_near.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    shelf11_fromSouth_near.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf11_fromSouth_near.name = GET_VARIABLE_NAME(shelf11_fromSouth_near);

    shelf11_fromSouth_far.gantry = {-13.52-0.172656, 4.30, -PI/2}; // shelf11_fromSouth_far
    shelf11_fromSouth_far.left_arm = {0.00, -3.25, 2.09, -2.02, -PI/2, 0.0};     // try to raise arm a little, use exact pi! better picks
    shelf11_fromSouth_far.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    shelf11_fromSouth_far.name = GET_VARIABLE_NAME(shelf11_fromSouth_far);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////// Row 6

    southwest_corner_staging.gantry = {-14.22, 6.9, PI}; // southwest_corner_staging
    southwest_corner_staging.left_arm = {-PI / 2, -1.01, 2.09, -1.13, 0.00, 0.00};
    southwest_corner_staging.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    southwest_corner_staging.name = GET_VARIABLE_NAME(southwest_corner_staging);


    waitpoint_best_south_fromNorth.gantry = {-11.4, 6.9, 0.88}; // waitpoint_best_south_fromNorth
    waitpoint_best_south_fromNorth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_south_fromNorth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    waitpoint_best_south_fromNorth.name = GET_VARIABLE_NAME(waitpoint_best_south_fromNorth);

    // waitpoint_best_south_fromSouth.gantry = {-11.4, 6.9, 0.88-PI}; // waitpoint_best_south_fromSouth
    // waitpoint_best_south_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    // waitpoint_best_south_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    // waitpoint_best_south_fromSouth.name = GET_VARIABLE_NAME(waitpoint_best_south_fromSouth);

    waitpoint_best_south_fromSouth.gantry = {-11.4, 6.9, PI/2}; // waitpoint_best_south_fromSouth
    // waitpoint_best_south_fromSouth.gantry = {-11.4, 6.9, 0.88-PI}; // waitpoint_best_south_fromSouth
    // waitpoint_best_south_fromSouth.left_arm = {-PI / 2, -1.01, 2.76, -1.13, 0.00, 0.00}; // left elbow bent more
    waitpoint_best_south_fromSouth.left_arm = {0.0, -3.58, 2.87, -4.65, -PI/2, 0.00}; // left elbow bent more
    waitpoint_best_south_fromSouth.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    waitpoint_best_south_fromSouth.name = GET_VARIABLE_NAME(waitpoint_best_south_fromSouth);

    // joint positions to go to agv2
    agv2_a.gantry = {0.6, 6.9, PI};
    agv2_a.left_arm = {0.0, -PI / 4, PI / 2, -PI / 4, PI / 2, 0};
    agv2_a.right_arm = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0}; // vertical up
    agv2_a.name = GET_VARIABLE_NAME(agv2_a);

    //////////// End Shelf Preset Locations
    ////////////
    

    cam_to_y_coordinate = {
        {0, 1.750927},
        {7, 1.698910},
        {9, 3.006604},
        {12, 3.006604},

        {6, 0.000000},
        {16, 0.000000},

        {8, -3.024268},
        {11, -3.024268},

        {1, -1.794935},
        {2, -1.806997},

        {3, 6.582773},
        {4, -7.175601},
        {5, 4.2},
        {10, -3.024268},
        {13, 3.78},
        {14, -3.024268},
        {15, 3.78},
        {16, 0.000000},
    };



    cam_to_presetlocation = {
        {0, bin3_a},
        {7, bin3_a},
        {9, shelf5_a},
        {12, shelf5_a},

        {6, shelf8_a},
        {16, shelf8_a},

        {8, shelf11_a},
        {11, shelf11_a},

        {1, bin11_a},
        {2, bin11_a},
    };

    cam_to_shelf_string = {
        // {0, "bin3_a"},
        // {7, "bin3_a"},
        {9, "shelf5"},
        {12, "shelf5"},
        {6, "shelf8"},
        {16, "shelf8"},
        {8, "shelf11"},
        {11, "shelf11"},
        // {1, "bin11_a"},
        // {2, "bin11_a"},
    };

    agv_to_camera = {
        {"agv1", 3},
        {"agv2", 4}};

    // vector of some of the locations, when gantry moves anywhere, it first searches through these possible locations,
    // and tries to find the closest one to it. That approximate location then serves as the beginning location for any move lookup.
    preset_locations_list_ = {start_a, bin3_a, agv2_a, agv1_staging_a,
                              bottom_left_staging_a, shelf8_a, shelf11_a, bin11_a, shelf5_a, shelf8_fromSouth_far,
                              shelf11_fromSouth_near, shelf8_fromSouth_near, shelf11_fromNorth_near, shelf8_fromNorth_near, shelf5_fromNorth_near, shelf5_fromSouth_near, // working for near picks! 0500
                              shelf11_fromSouth_far, shelf8_fromSouth_far, shelf11_fromNorth_far, shelf8_fromNorth_far, shelf5_fromNorth_far, shelf5_fromSouth_far,
                              }; // do not have mid_xyz anything here for now

    preset_locations_list_simple_ = {bin3_a, agv2_a, agv1_staging_a,
                              bottom_left_staging_a, shelf8_a, shelf11_a, bin11_a, shelf5_a, shelf8_fromSouth_far}; // do not have mid_xyz anything here for now

    wait_preset_locations_list_ = {waitpoint_best_south_fromSouth.name, waitpoint_best_south_fromNorth.name, waitpoint_best_north_fromSouth.name, waitpoint_best_north_fromNorth.name};

    PathingLookupDictionary = {
        {{"start_a", "bin3_a"}, std::vector<PresetLocation>{start_a, bin3_a}},
        {{"start_a", "agv2_a"}, std::vector<PresetLocation>{start_a, agv2_a}},
        {{"start_a", "agv1_staging_a"}, std::vector<PresetLocation>{start_a, agv1_staging_a}},
        {{"start_a", "bottom_left_staging_a"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a}},
        {{"start_a", "shelf8_a"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_a, shelf8_a}},
        {{"start_a", "shelf11_a"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_a, shelf11_a}},
        {{"start_a", "bin11_a"}, std::vector<PresetLocation>{start_a, bin11_a}},
        // {{"start_a", "start_a"}, std::vector<PresetLocation>{start_a, start_a}},
        {{"start_a", "start_a"}, std::vector<PresetLocation>{bin11_a,start_a, start_a, start_a, start_a, start_a, start_a, start_a}}, // GO to start_a, do not path simplify.

        {{"bin3_a", "start_a"}, std::vector<PresetLocation>{bin3_a, start_a}},
        {{"agv2_a", "start_a"}, std::vector<PresetLocation>{agv2_a, start_a}},
        {{"agv1_staging_a", "start_a"}, std::vector<PresetLocation>{agv1_staging_a, start_a}},
        {{"bottom_left_staging_a", "start_a"}, std::vector<PresetLocation>{bottom_left_staging_a, agv1_staging_a, start_a}},
        {{"shelf5_a", "start_a"}, std::vector<PresetLocation>{bottom_left_staging_a, agv1_staging_a, start_a}}, // added for shelf 5 bumping into shelves, go direct
        {{"shelf8_a", "start_a"}, std::vector<PresetLocation>{shelf8_a, mid_5_8_staging_a, start_a}},
        {{"shelf11_a", "start_a"}, std::vector<PresetLocation>{shelf11_a, shelf11_a, shelf11_a, mid_8_11_staging_a, start_a}}, //GO to shelf 11a!
        {{"bin11_a", "start_a"}, std::vector<PresetLocation>{bin11_a, start_a}},

        {{"agv1_staging_a", "bin3_a"}, std::vector<PresetLocation>{start_a, bin3_a}}, // go to start_a first /////////// this block: do not go to first point first
        {{"agv1_staging_a", "agv2_a"}, std::vector<PresetLocation>{agv2_a}},
        {{"agv1_staging_a", "agv1_staging_a"}, std::vector<PresetLocation>{agv1_staging_a}}, // go to itself
        {{"agv1_staging_a", "bottom_left_staging_a"}, std::vector<PresetLocation>{agv1_staging_a, bottom_left_staging_a}},
        {{"agv1_staging_a", "shelf8_a"}, std::vector<PresetLocation>{mid_5_8_staging_a, shelf8_a}},
        {{"agv1_staging_a", "shelf11_a"}, std::vector<PresetLocation>{mid_8_11_staging_a, shelf11_a}},
        {{"agv1_staging_a", "bin11_a"}, std::vector<PresetLocation>{start_a, bin11_a}}, // go to start_a first

        {{"agv2_a", "bin3_a"}, std::vector<PresetLocation>{start_a, bin3_a}}, // go to start_a first /////////////// this block: do not go to first point first
        {{"agv2_a", "agv2_a"}, std::vector<PresetLocation>{agv2_a}},          // go to itself
        {{"agv2_a", "agv1_staging_a"}, std::vector<PresetLocation>{agv1_staging_a}},
        {{"agv2_a", "bottom_left_staging_a"}, std::vector<PresetLocation>{agv1_staging_a, bottom_left_staging_a}},
        {{"agv2_a", "southwest_corner_staging"}, std::vector<PresetLocation>{agv2_a, southwest_corner_staging}}, // added
        {{"agv2_a", "shelf8_a"}, std::vector<PresetLocation>{mid_5_8_staging_a, shelf8_a}},
        {{"agv2_a", "shelf11_a"}, std::vector<PresetLocation>{mid_8_11_staging_a, shelf11_a}},
        {{"agv2_a", "bin11_a"}, std::vector<PresetLocation>{start_a, bin11_a}}, // go to start_a first

        {{"bin3_a", "bin11_a"}, std::vector<PresetLocation>{bin3_a, bin11_a}},
        {{"bin3_a", "bin3_a"}, std::vector<PresetLocation>{bin3_a, bin3_a}},

        ////////////////////////////////////////////////////////////////////////////////////////////////// Row by row, left to right
        {{"start_a", "waitpoint_best_north_fromNorth"}, std::vector<PresetLocation>{start_a, agv1_staging_a, waitpoint_best_north_fromNorth}},
        {{"start_a", "waitpoint_best_north_fromSouth"}, std::vector<PresetLocation>{start_a, agv1_staging_a, waitpoint_best_north_fromSouth}},

        {{"start_a", "shelf5_fromNorth_near"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_near}},
        {{"start_a", "shelf5_fromNorth_far"}, std::vector<PresetLocation>{start_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_far}},

        {{"start_a", "shelf5_fromSouth_near"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near}},
        {{"start_a", "shelf5_fromSouth_far"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near, shelf5_fromSouth_far}},
        {{"start_a", "shelf8_fromNorth_near"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near}},
        {{"start_a", "shelf8_fromNorth_far"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near, shelf8_fromNorth_far}},
        // {{"start_a", "mid_5_8_intersection_fromNorth"}, std::vector<PresetLocation>{start_a, 222, mid_5_8_intersection_fromNorth}},
        // {{"start_a", "mid_5_8_intersection_fromSouth"}, std::vector<PresetLocation>{start_a, 222, mid_5_8_intersection_fromSouth}},

        {{"start_a", "mid_5_8_staging_fromNorth"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromNorth}},
        {{"start_a", "mid_5_8_staging_fromSouth"}, std::vector<PresetLocation>{start_a, mid_5_8_staging_fromSouth}},

        {{"start_a", "shelf8_fromSouth_near"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near}},
        {{"start_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        {{"start_a", "shelf11_fromNorth_near"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near}},
        {{"start_a", "shelf11_fromNorth_far"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near, shelf11_fromNorth_far}},
        // {{"start_a", "mid_8_11_intersection_fromNorth"}, std::vector<PresetLocation>{start_a, 222, mid_8_11_intersection_fromNorth}},
        // {{"start_a", "mid_8_11_intersection_fromSouth"}, std::vector<PresetLocation>{start_a, 222, mid_8_11_intersection_fromSouth}},
        {{"start_a", "mid_8_11_staging_fromNorth"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromNorth}},
        {{"start_a", "mid_8_11_staging_fromSouth"}, std::vector<PresetLocation>{start_a, mid_8_11_staging_fromSouth}},

        {{"start_a", "shelf11_fromSouth_near"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_near}},
        {{"start_a", "shelf11_fromSouth_far"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_far}},
        
        {{"start_a", "southwest_corner_staging"}, std::vector<PresetLocation>{start_a, agv2_a, southwest_corner_staging}},
        {{"start_a", "waitpoint_best_south_fromNorth"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromNorth}},
        {{"start_a", "waitpoint_best_south_fromSouth"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromSouth}},


        ////////////////////////////////////////////////////////////////////////////////////////////////// shelf8_fromSouth_far (organize this)
        // {{"shelf8_fromSouth_far", "start_a"}, std::vector<PresetLocation>{shelf8_fromSouth_far, shelf8_fromSouth_near, mid_8_11_intersection_fromSouth, waitpoint_best_south_fromSouth, agv2_a, start_a}},

        ////////////////////////////////////////////////////////////////////////////////////////////////// Row by row, left to right REVERSED, NEAR

        {{"shelf11_fromSouth_near", "start_a"}, std::vector<PresetLocation>{shelf11_fromSouth_near, southwest_corner_staging, agv2_a, start_a}},
        {{"shelf8_fromSouth_near", "start_a"}, std::vector<PresetLocation>{shelf8_fromSouth_near, mid_8_11_staging_fromSouth, start_a}},
        {{"shelf11_fromNorth_near", "start_a"}, std::vector<PresetLocation>{shelf11_fromNorth_near, shelf11_fromNorth_near, mid_8_11_staging_fromNorth, start_a}},
        {{"shelf8_fromNorth_near", "start_a"}, std::vector<PresetLocation>{shelf8_fromNorth_near, mid_5_8_staging_fromNorth, start_a}},
        {{"shelf5_fromNorth_near", "start_a"}, std::vector<PresetLocation>{shelf5_fromNorth_near, bottom_left_staging_a, agv1_staging_a, start_a}},
        {{"shelf5_fromSouth_near", "start_a"}, std::vector<PresetLocation>{shelf5_fromSouth_near, mid_5_8_staging_fromSouth, start_a}},

        ////////////////////////////////////////////////////////////////////////////////////////////////// AGV to near or far shelf picks directly

        {{"agv1_staging_a", "shelf5_fromNorth_near"}, std::vector<PresetLocation>{agv1_staging_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_near}},
        {{"agv1_staging_a", "shelf5_fromNorth_far"}, std::vector<PresetLocation>{agv1_staging_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_far}},
        {{"agv1_staging_a", "shelf5_fromSouth_near"}, std::vector<PresetLocation>{agv1_staging_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near}},
        {{"agv1_staging_a", "shelf5_fromSouth_far"}, std::vector<PresetLocation>{agv1_staging_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near, shelf5_fromSouth_far}},
        {{"agv1_staging_a", "shelf8_fromNorth_near"}, std::vector<PresetLocation>{agv1_staging_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near}},
        {{"agv1_staging_a", "shelf8_fromNorth_far"}, std::vector<PresetLocation>{agv1_staging_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near, shelf8_fromNorth_far}},
        {{"agv1_staging_a", "shelf8_fromSouth_near"}, std::vector<PresetLocation>{agv1_staging_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near}},
        {{"agv1_staging_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{agv1_staging_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        {{"agv1_staging_a", "shelf11_fromNorth_near"}, std::vector<PresetLocation>{agv1_staging_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near}},
        {{"agv1_staging_a", "shelf11_fromNorth_far"}, std::vector<PresetLocation>{agv1_staging_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near, shelf11_fromNorth_far}},
        {{"agv1_staging_a", "shelf11_fromSouth_near"}, std::vector<PresetLocation>{agv1_staging_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_near}},
        {{"agv1_staging_a", "shelf11_fromSouth_far"}, std::vector<PresetLocation>{agv1_staging_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_far}},
        {{"agv1_staging_a", "southwest_corner_staging"}, std::vector<PresetLocation>{agv1_staging_a, agv2_a, southwest_corner_staging}},

        {{"agv2_a", "shelf5_fromNorth_near"}, std::vector<PresetLocation>{agv2_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_near}},
        {{"agv2_a", "shelf5_fromNorth_far"}, std::vector<PresetLocation>{agv2_a, agv1_staging_a, bottom_left_staging_a, shelf5_fromNorth_far}},
        {{"agv2_a", "shelf5_fromSouth_near"}, std::vector<PresetLocation>{agv2_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near}},
        {{"agv2_a", "shelf5_fromSouth_far"}, std::vector<PresetLocation>{agv2_a, mid_5_8_staging_fromSouth, shelf5_fromSouth_near, shelf5_fromSouth_far}},
        {{"agv2_a", "shelf8_fromNorth_near"}, std::vector<PresetLocation>{agv2_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near}},
        {{"agv2_a", "shelf8_fromNorth_far"}, std::vector<PresetLocation>{agv2_a, mid_5_8_staging_fromNorth, shelf8_fromNorth_near, shelf8_fromNorth_far}},
        {{"agv2_a", "shelf8_fromSouth_near"}, std::vector<PresetLocation>{agv2_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near}},
        {{"agv2_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{agv2_a, mid_8_11_staging_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        {{"agv2_a", "shelf11_fromNorth_near"}, std::vector<PresetLocation>{agv2_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near}},
        {{"agv2_a", "shelf11_fromNorth_far"}, std::vector<PresetLocation>{agv2_a, mid_8_11_staging_fromNorth, shelf11_fromNorth_near, shelf11_fromNorth_far}},
        {{"agv2_a", "shelf11_fromSouth_near"}, std::vector<PresetLocation>{agv2_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_near}},
        {{"agv2_a", "shelf11_fromSouth_far"}, std::vector<PresetLocation>{agv2_a, agv2_a, southwest_corner_staging, shelf11_fromSouth_far}},
        {{"agv2_a", "southwest_corner_staging"}, std::vector<PresetLocation>{agv2_a, agv2_a, southwest_corner_staging}},

        ////////////////////////////////////////////////////////////////////////////////////////////////// Row by row, left to right REVERSED, FAR
        {{"shelf11_fromSouth_far", "start_a"}, std::vector<PresetLocation>{shelf11_fromSouth_far, southwest_corner_staging, agv2_a, start_a}}, // only this one, do not go back to near position first
        {{"shelf8_fromSouth_far", "start_a"}, std::vector<PresetLocation>{shelf8_fromSouth_far, shelf8_fromSouth_near, mid_8_11_staging_fromSouth, start_a}},
        {{"shelf11_fromNorth_far", "start_a"}, std::vector<PresetLocation>{shelf11_fromNorth_far, shelf11_fromNorth_near, shelf11_fromNorth_near, mid_8_11_staging_fromNorth, start_a}},
        {{"shelf8_fromNorth_far", "start_a"}, std::vector<PresetLocation>{shelf8_fromNorth_far, shelf8_fromNorth_near, mid_5_8_staging_fromNorth, start_a}},
        {{"shelf5_fromNorth_far", "start_a"}, std::vector<PresetLocation>{shelf5_fromNorth_far, shelf5_fromNorth_near, bottom_left_staging_a, agv1_staging_a, start_a}},
        {{"shelf5_fromSouth_far", "start_a"}, std::vector<PresetLocation>{shelf5_fromSouth_far, shelf5_fromSouth_near, mid_5_8_staging_fromSouth, start_a}},




    };

    WaitPathingLookupDictionary = {
        // {{"start_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromSouth, mid_8_11_intersection_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        {{"start_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{start_a, agv2_a, waitpoint_best_south_fromSouth, mid_8_11_intersection_fromSouth, shelf8_fromSouth_far}},
        // {{"shelf8_fromSouth_far", "start_a"}, std::vector<PresetLocation>{shelf8_fromSouth_far, shelf8_fromSouth_near, mid_8_11_intersection_fromSouth, waitpoint_best_south_fromSouth, agv2_a, start_a}},
        {{"shelf8_fromSouth_far", "start_a"}, std::vector<PresetLocation>{shelf8_fromSouth_far, mid_8_11_intersection_fromSouth, waitpoint_best_south_fromSouth, agv2_a, start_a}},


        {{"agv1_staging_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{agv1_staging_a, agv2_a, waitpoint_best_south_fromSouth, mid_8_11_intersection_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        {{"agv2_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{agv2_a, waitpoint_best_south_fromSouth, mid_8_11_intersection_fromSouth, shelf8_fromSouth_near, shelf8_fromSouth_far}},
        {{"agv2_a", "shelf8_fromSouth_far"}, std::vector<PresetLocation>{agv2_a, waitpoint_best_south_fromSouth, mid_8_11_intersection_fromSouth, shelf8_fromSouth_far}},


    };
}