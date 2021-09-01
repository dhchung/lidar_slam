#include "gps_module.h"

GPSModule::GPSModule() {
    ros_time = 0.0f;
    gps_values.clear();
    GNGGA_data = false;
    GNRMC_data = false;
}

GPSModule::~GPSModule() {

}

bool GPSModule::cb_gps(core_msgs::string_w_header msg){
    bool return_value = false;

    std::string data = msg.data;
    double time = msg.header.stamp.toSec();

    std::pair<std::string, std::map<std::string, double>> values = StringToData(data);

    if(values.first.compare("$GNRMC") == 0) {
        gps_values.clear();
        gps_values.resize(10);
        GNRMC_data = true;
        GNGGA_data = false;

        gps_values[1] = values.second["latitude"]; //latitude
        gps_values[2] = values.second["longitude"]; //longitude
        gps_values[5] = values.second["signal_type"]; //signal_type
        gps_values[8] = values.second["SOG"]; //SOG
        gps_values[9] = values.second["COG"]; //COG
    }

    if(values.first.compare("$GNGGA") == 0) {
        if(GNRMC_data) {
            GNGGA_data = true;
            gps_values[0] = values.second["gps_time"]; //gps_time
            gps_values[3] = values.second["altitude"]; //altitude
            gps_values[6] = values.second["Fix"]; //Fix
            gps_values[7] = values.second["HDOP"]; //HDOP
        } else {
            return false;
        }
    }

    if(values.first.compare("$GNHDT") == 0) {
        if(GNGGA_data && GNRMC_data) {
            ros_time = time;

            gps_values[4] = values.second["yaw"]; //yaw
            GNGGA_data = false;
            GNRMC_data = false;

            gps_odom = gps2odom(time, gps_values);
            return true;
        } else {
            GNGGA_data = false;
            GNRMC_data = false;

            return false;
        }
    }
}

nav_msgs::Odometry GPSModule::gps2odom(long double time, std::vector<double> gps_data){
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp.fromSec(time);

    double latitude = gps_data[1];
    double longitude = gps_data[2];
    double altitude = gps_data[3];
    double yaw = gps_data[4];

    printf("latitude in MINE : %.16f\n", latitude);
    printf("longitude in MINE : %.16f\n", longitude);

    double utmx;
    double utmy;
    ToUtm(latitude, longitude, utmx, utmy);

    // odom.pose.pose.position.x = utmx - 3986631.0f;
    // odom.pose.pose.position.y = utmy - 534042.0f;
    // odom.pose.pose.position.z = 0.0f;

    odom.pose.pose.position.x = utmx;
    odom.pose.pose.position.y = utmy;
    odom.pose.pose.position.z = altitude;

    tf::Quaternion Q;
    double yaw_rad = (yaw/180.0f)*M_PI;
    // Q.setEuler(0, 0, yaw/180.0f*M_PI);
    Q.setRPY(0, 0, yaw_rad);

    odom.pose.pose.orientation.x = Q.x();
    odom.pose.pose.orientation.y = Q.y();
    odom.pose.pose.orientation.z = Q.z();
    odom.pose.pose.orientation.w = Q.w();

    return odom;
}



void GPSModule::GPS_DD2DM(const double dlat_dd, const double dlon_dd, double &dlat_dm, double &dlon_dm)
{
	double dLat_deg, dLon_deg;
	double dLat_degdec, dLon_degdec;

	dLat_deg = floor(dlat_dd);
	dLon_deg = floor(dlon_dd);

	dLat_degdec = (dlat_dd - dLat_deg) * 60.0;
	dLon_degdec = (dlon_dd - dLon_deg) * 60.0;

	dLat_deg = dLat_deg * 100.0;
	dLon_deg = dLon_deg * 100.0;

	dlat_dm = dLat_deg + dLat_degdec;
	dlon_dm = dLon_deg + dLon_degdec;
}

double GPSModule::GPS_DM2DD(double dm)
{
	double deg;
	double degdec;
	double min;

	// ddxx.xxxxxx , dddxx.xxxxx
	deg = floor(dm / 100.0);
	// xxmm.mmmmmm , xxxmm.mmmmm
	min = dm - (deg * 100.0);

	degdec = min / 60.0;

	double dd = deg + degdec;

    return dd;
}


void GPSModule::ToUtm(double dlat, double dlon, double &rutmx, double &rutmy)
{
	double lat, lon;

	// coordinates in radians
	lat = dlat * M_PI / 180;
	lon = dlon * M_PI / 180;

	// UTM parameters
	double lon0_f = floor(dlon / 6) * 6 + 3; // reference longitude in degrees
	double lon0 = lon0_f * M_PI / 180;		 // in radians
	double k0 = 0.9996;						 // scale on central meridian

	int FE = 500000;				// false easting
	int FN = (dlat < 0) * 10000000; // false northing

	// Equations parameters
	// N: radius of curvature of the earth perpendicular to meridian plane
	// Also, distance from point to polar axis
	double WN = Wa / sqrt(1 - pow(We, 2) * pow(sin(lat), 2));
	double WT = pow(tan(lat), 2);
	double WC = (pow(We, 2) / (1 - pow(We, 2))) * pow(cos(lat), 2);
	double WLA = (lon - lon0) * cos(lat);
	// M: true distance along the central meridian from the equator to lat
	double WM = Wa * ((1 - pow(We, 2) / 4 - 3 * pow(We, 4) / 64 - 5 * pow(We, 6) / 256) * lat - (3 * pow(We, 2) / 8 + 3 * pow(We, 4) / 32 + 45 * pow(We, 6) / 1024) * sin(2 * lat) + (15 * pow(We, 4) / 256 + 45 * pow(We, 6) / 1024) * sin(4 * lat) - (35 * pow(We, 6) / 3072) * sin(6 * lat));

	// northing
	// M(lat0) = 0 so not used in following formula
	rutmx = (FN + k0 * WM + k0 * WN * tan(lat) * (pow(WLA, 2) / 2 + (5 - WT + 9 * WC + 4 * pow(WC, 2)) * pow(WLA, 4) / 24 + (61 - 58 * WT + pow(WT, 2) + 600 * WC - 330 * Weps) * pow(WLA, 6) / 720));

	// easting
	rutmy = (FE + k0 * WN * (WLA + (1 - WT + WC) * pow(WLA, 3) / 6 + (5 - 18 * WT + pow(WT, 2) + 72 * WC - 58 * Weps) * pow(WLA, 5) / 120));
}


std::pair<std::string, std::map<std::string, double>> GPSModule::StringToData(std::string & gps_data){
    std::pair<std::string, std::map<std::string, double>> result;

    std::string phrase;
    std::stringstream ss(gps_data);
    std::vector<std::string> row;
    int idx = 0;
    std::string gps_data_type;
    while(std::getline(ss, phrase, ',')) {
        if(phrase.compare("")==0) {
            ++idx;
            continue;
        }
        if(idx==0) {
            gps_data_type = phrase;
            result.first = phrase;
            ++idx;
            continue;
        }
        if(gps_data_type.compare("$GNGGA")==0) {
            switch (idx) {
                case 1:
                    result.second["gps_time"] = std::stod(phrase);
                    break;
                case 2:
                    result.second["latitude"] = GPS_DM2DD(std::stod(phrase));
                    break;
                case 3:
                    if(phrase.compare("S")==0) {
                        result.second["latitude"] = -result.second["latitude"];
                    }
                    break;
                case 4:
                    result.second["longitude"] = GPS_DM2DD(std::stod(phrase));
                    break;
                case 5:
                    if(phrase.compare("W")==0) {
                        result.second["longitude"] = -result.second["longitude"];
                    }
                    break;
                case 6:
                    result.second["fix"] = atoi(phrase.c_str());
                    break;
                case 8:
                    result.second["HDOP"] = std::stod(phrase);
                    break;
                case 9:
                    result.second["altitude"] = -std::stod(phrase);
                    break;
                default:
                    break;
            }
            ++idx;
            continue;
        } else if(gps_data_type.compare("$GNRMC")==0) {
            switch (idx) {
                case 1:
                    result.second["gps_time"] = std::stod(phrase);
                    break;
                case 2:
                    if(phrase.compare("A")==0) {
                        result.second["good_signal"] = 1.0;
                    } else {
                        result.second["good_signal"] = 0.0;
                    }
                    break;
                case 3:
                    result.second["latitude"] = GPS_DM2DD(std::stod(phrase));
                    break;
                case 4:
                    if(phrase.compare("S")==0) {
                        result.second["latitude"] = -result.second["latitude"];
                    }
                    break;
                case 5:
                    result.second["longitude"] = GPS_DM2DD(std::stod(phrase));
                    break;
                case 6:
                    if(phrase.compare("W")==0) {
                        result.second["longitude"] = -result.second["longitude"];
                    }
                    break;
                case 7:
                    result.second["SOG"] = std::stod(phrase);
                    break;
                case 8:
                    result.second["COG"] = std::stod(phrase);
                    break;
                default:
                    break;
            }
            ++idx;
            continue;

        } else if(gps_data_type.compare("$GNHDT")==0) {
            if(idx==1) {
                result.second["yaw"] = std::stod(phrase);
            }
            ++idx;
            continue;
        }



        // if(phrase.compare("")==0) {
        //     continue;
        // }
    }

    return result;
}