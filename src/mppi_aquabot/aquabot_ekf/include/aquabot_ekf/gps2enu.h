#ifndef GPS2ENU_H
#define GPS2ENU_H

#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;

namespace aquabot_ekf
{

struct toENU
{
  double X0,Y0,Z0;
  int zone{-1};

  inline bool isInit() const
  {
    return zone > 0;
  }

  void setReference(double latitude, double longitude, double Z0 = 2.)
  {
    auto northp{true};
    GeographicLib::UTMUPS::Forward(latitude, longitude,
                                   zone, northp, X0,Y0);
    this->Z0 = Z0 - 1.62;
  }

  inline void transform(double latitude, double longitude, double altitude,
                        Point &p) const
  {
    bool northp{true};
    auto zone{this->zone};
    // this will throw if we move too far from the initial UTM zone
    GeographicLib::UTMUPS::Forward(latitude, longitude,
                                   zone, northp, p.x, p.y, this->zone);

    p.x -= X0;
    p.y -= Y0;
    p.z = altitude-Z0;
  }
};

}

#endif // GPS2ENU_H
