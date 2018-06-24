#include "Debug.h"

#include <math.h>


void WGStoLatLonHeight(WGS84 in, double& lat, double& lon, double& height)
{
    // WGS ellipsoid params
    double a = 6378137;
    double f = 1/298.257;
    double e = sqrt(2*f - pow(f, 2));

    // From equation 4.A.3,
    double lambda = atan2(in.y, in.x);
    double p = sqrt(pow(in.x, 2) + pow(in.y, 2));

    // initial value of phi assuming h = 0;
    double h = 0;
    double phi = atan2(in.z, p*(1 - pow(e, 2)));
    double N = a / pow((1 - pow(e*sin(phi), 2)), 0.5);
    double delta_h = 1000000;
    while (delta_h > 0.01)
    {
        double prev_h = h;
        phi = atan2(in.z, p*(1 - pow(e, 2)*(N/(N+h))));
        N = a / pow((1 - pow(e*sin(phi), 2)), 0.5);
        h = p/cos(phi)-N;
        delta_h = abs(h-prev_h);
    }

    lat = phi*180.0/M_PI;
    lon = lambda*180.0/M_PI;
    height = h;
}
