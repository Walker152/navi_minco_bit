#include "bt_manager/utils/nav_zone.hpp"

namespace Sentry_BT
{
  std::vector<Point2D> nav_points = {
    //   {3.0, 3.0},  //HOME
    //   {8.5, 8.5},  // BONUS
    //   {15.7, 11.0}  // OUTPOST

      // for test
      // {1.8, 5.6},  //HOME
      // {5.6, 3.8},  //BONUS
      // {7.2, 6.0}   //OUTPOST

    // for rmul
      {1.2, 7.2},  //HOME
      {6.4, 4.4},  // BONUS
<<<<<<< HEAD
      {7.0, 1.7}  // OUTPOST
=======
      {7.5, 6.8}  // OUTPOST
>>>>>>> origin/master
  };

  std::vector<Sentry_BT::Point2D> patrol_points_normal = {
    // for rmuc
    //   {14.2, 11.5},
    //   {17.5, 8.2},
    //   {16.0, 4.2},
    //   {12.4, 5.1},
    //   {12.0, 8.6},
    //   {14.0, 12.0}
    // for test
      // {5.6, 6.0},
      // {4.5, 5.1},
      // {5.5, 3.5}

    // for rmul
     // {7.2, 5.3},
      {6.4, 4.4},
      // {5.5, 5.3}
      // {7.3, 4.5}
  };

  std::vector<int> patrol_points_milliseconds = {
      10000,
      // 5000
    //   6000,
    //   2000,
    //   2000,
  };

  std::vector<Sentry_BT::Point2D> patrol_points_attack = {
    //   {15.2, 11.5},
    //   {12.8, -0.1},
    //   {11.5, -3.8}

    // for test
    // {5.6, 6.0},
    // {4.5, 5.1},
    // {5.5, 3.5}

    // for rmul
      // {7.2, 5.3},
      {6.4, 4.4}, 
      // {5.5, 5.3}
      // {7.3, 4.5}
  };

