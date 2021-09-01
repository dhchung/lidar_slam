#define UTM_ZON 52
#define Wa 6378137.0
#define Wb 6356752.314245
#define We 0.081819190842965
#define Weps 0.006739496742333
#include <tf/tf.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <core_msgs/string_w_header.h>

class GPSModule{
public:
    GPSModule();
    ~GPSModule();

    void GPS_DD2DM(const double dlat_dd, const double dlon_dd, double &dlat_dm, double &dlon_dm);
    double GPS_DM2DD(double dm);

    void ToUtm(double dlat, double dlon, double &rutmx, double &rutmy);
    nav_msgs::Odometry gps2odom(long double time, std::vector<double> gps_data);

    long double ros_time;
    // std::vector<std::string> gps_values;

    std::vector<double> gps_values;

    bool GNGGA_data;
    bool GNRMC_data;

    nav_msgs::Odometry gps_odom;
    bool cb_gps(core_msgs::string_w_header msg);

    std::pair<std::string, std::map<std::string, double>> StringToData(std::string & gps_data);

};