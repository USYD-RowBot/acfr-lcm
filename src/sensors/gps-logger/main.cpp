#include <iostream>
#include <ctime>
#include <fstream>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>

#include <lcm/lcm-cpp.hpp>

#include "perls-common/getopt.h"
#include "perls-common/units.h"

#include "perls-lcmtypes++/senlcm/gpsd3_t.hpp"

/*  TODO:
 *  improve parsing error handling
*/

using boost::posix_time::operator <<;

const double MINUTES_TO_DEGREE = 1.0/60; 
const int64_t HOUR_IN_DAY = 24;
const int64_t SEC_IN_HOUR = 3600;
const int64_t MICROSEC_IN_SEC = 1e6;

class GPSLogger
{
  public:
    GPSLogger (const std::string&, const std::string&, const std::string);
    ~GPSLogger () { _gpslog.close ();}

    void parse_log ();

  private:
    std::ifstream _gpslog;
    lcm::LogFile _lcmlog;
    std::string _channel;
    
    boost::posix_time::ptime _gpstime;

    senlcm::gpsd3_t _gpsd;

    void add_log_event ();
};

GPSLogger::GPSLogger (const std::string &infile, const std::string &outfile, const std::string channel)
    : _lcmlog (outfile.c_str (), "w"), _channel (channel)
{
    std::cout << "opening " << infile << std::endl;
    _gpslog.open (infile.c_str ());
    if (!_gpslog.good ()) {
        std::cerr << "gpslog not good!" << std::endl;
        exit (EXIT_FAILURE);
    }

    if (!_lcmlog.good ()) {
        std::cerr << "lcmlog not good!" << std::endl;
        exit (EXIT_FAILURE);
    }

    senlcm::gpsd3_t tmp = {0};
    _gpsd = tmp;

    _gpstime = boost::posix_time::neg_infin;
}

int update_time (boost::posix_time::ptime &btime, std::string hhmmss)
{
    boost::gregorian::date date = btime.date ();

    if (date.is_neg_infinity ()) // date not set yet
        return 1;

    // format is hhmmss.sss
    boost::posix_time::time_duration duration;
    try {
        int hrs = boost::lexical_cast<int>(hhmmss.substr (0,2)); //hh
        int min = boost::lexical_cast<int>(hhmmss.substr (2,2)); //mm
        int sec = boost::lexical_cast<int>(hhmmss.substr (4,2)); //ss
        int fsc = boost::lexical_cast<int>(hhmmss.substr (7,3)); //sss
        duration =  boost::posix_time::time_duration (boost::posix_time::time_duration (hrs, min, sec, 0) + boost::posix_time::milliseconds (fsc));
    }
    catch (boost::bad_lexical_cast &e) {
        std::cerr << e.what () << std::endl;
        return 1;
    }

    btime = boost::posix_time::ptime (date, duration);
    return 0;
}

int update_date (boost::posix_time::ptime &btime, std::string ddmmyy)
{
    boost::posix_time::time_duration tod = btime.time_of_day ();
    if (tod.is_neg_infinity ())
        tod = boost::posix_time::time_duration (0,0,0);

    std::string dd = ddmmyy.substr (0,2);
    std::string mm = ddmmyy.substr (2,2);
    std::string yy = ddmmyy.substr (4,2);
    yy = "20" + yy;
    boost::gregorian::date date = boost::gregorian::from_undelimited_string (yy+mm+dd);

    btime = boost::posix_time::ptime (date, tod);
    return 0;
}

int64_t ptime_to_utime (const boost::posix_time::ptime btime)
{
    boost::gregorian::date_duration date_diff = btime.date() - boost::gregorian::date(1970,1,1);
    boost::posix_time::time_duration tod = btime.time_of_day();

    return
        static_cast<int64_t>(date_diff.days())*HOUR_IN_DAY*SEC_IN_HOUR*MICROSEC_IN_SEC + 
        static_cast<int64_t>(tod.total_seconds())*MICROSEC_IN_SEC +
        static_cast<int64_t>(tod.fractional_seconds()) /
        (boost::posix_time::time_duration::ticks_per_second() / MICROSEC_IN_SEC);        
}

double nmeall_to_rad (std::string latlon)
{
    if (latlon.size() == 9)     // check whether lat or lon by size of string
        latlon = "0" + latlon;  // if latitude, prepend with 0
    double ll_deg = boost::lexical_cast<double>(latlon.substr (0,3));
    double ll_min = boost::lexical_cast<double>(latlon.substr (3,7));
    return (ll_deg + ll_min*MINUTES_TO_DEGREE)*UNITS_DEGREE_TO_RADIAN;
}

void GPSLogger::add_log_event ()
{
    lcm::LogEvent *event = new lcm::LogEvent ();
    event->timestamp = ptime_to_utime (_gpstime);
    event->channel = _channel;
    int gpsd_size = _gpsd.getEncodedSize ();
    uint8_t *buf = new uint8_t[gpsd_size];
    _gpsd.encode (buf, 0, gpsd_size);
    event->datalen = gpsd_size;
    event->data = buf;

    _lcmlog.writeEvent (event);
    delete buf;
    delete event;
}


void GPSLogger::parse_log ()
{
    std::string line;
    while (_gpslog.good ()) {
        std::getline (_gpslog, line);
        std::cout << line << std::endl;

        std::vector<std::string> nmea;
        boost::split (nmea, line, boost::is_any_of(",*"));
        if (nmea[0] == "$GPRMC") {
            if (update_date (_gpstime, nmea[9])==1) continue;
            if (update_time (_gpstime, nmea[1])==1) continue;
            _gpsd.utime = ptime_to_utime (_gpstime);
            _gpsd.fix.utime = ptime_to_utime (_gpstime);
            _gpsd.fix.latitude = (nmea[4]=="N" ? 1 : -1)*nmeall_to_rad (nmea[3]);
            _gpsd.fix.longitude = (nmea[6]=="E" ? 1 : -1)*nmeall_to_rad (nmea[5]);
            _gpsd.fix.track = (boost::lexical_cast<double>(nmea[8]))*UNITS_DEGREE_TO_RADIAN;
            _gpsd.fix.speed = (boost::lexical_cast<double>(nmea[7]))*UNITS_KNOT_TO_METER_PER_SEC;
            add_log_event ();
        }
        else if (nmea[0] == "$GPGGA") {
            if (update_time (_gpstime, nmea[1])==1) continue;
            _gpsd.utime = ptime_to_utime (_gpstime);
            _gpsd.fix.utime = ptime_to_utime (_gpstime);
            _gpsd.fix.latitude = (nmea[3]=="N" ? 1 : -1)*nmeall_to_rad (nmea[2]);
            _gpsd.fix.longitude = (nmea[5]=="E" ? 1 : -1)*nmeall_to_rad (nmea[4]);
            _gpsd.status = boost::lexical_cast<int16_t>(nmea[6]);
            _gpsd.dop.hdop = boost::lexical_cast<double>(nmea[8]);
            _gpsd.fix.altitude = boost::lexical_cast<double>(nmea[9]);
            _gpsd.geoidal_separation = boost::lexical_cast<double>(nmea[11]);
            add_log_event ();
        }
        else if (nmea[0] == "$GPGSA") {
            _gpsd.fix.mode = boost::lexical_cast<int16_t>(nmea[2]);
            _gpsd.dop.pdop = boost::lexical_cast<double>(nmea[15]);
            _gpsd.dop.hdop = boost::lexical_cast<double>(nmea[16]);
            _gpsd.dop.vdop = boost::lexical_cast<double>(nmea[17]);
            
            std::vector<int16_t> svs_used;
            for (unsigned int ctr=3; ctr<=14; ctr++) {
                if (!nmea[ctr].empty ())
                    svs_used.push_back (boost::lexical_cast<int16_t>(nmea[ctr]));
            }
            _gpsd.satellites_used = svs_used.size ();
            _gpsd.used = svs_used;
            svs_used.clear ();
        }
    }
}

int main (int argc, char **argv)
{
    getopt_t *gopt = getopt_create ();
    getopt_add_description (gopt, "parse gps nmea log file to lcm stream");
    getopt_add_help (gopt, 0);
    getopt_add_string (gopt, 'c', "channel", "GPSLOG", "lcm-channel");
    getopt_add_string (gopt, 'o', "output", "lcmlog-gpslog", "lcmlog-output");

    if (!getopt_parse (gopt, argc, argv, 1) || gopt->extraargs->len!=1) {
        getopt_do_usage (gopt, "/path/to/gpslog.csv");
        exit (EXIT_FAILURE);
    }
    std::string gpslog = static_cast<char*>(g_ptr_array_index (gopt->extraargs, 0));
    std::string lcmlog = getopt_get_string (gopt, "output");
    std::string channel = getopt_get_string (gopt, "channel");

    GPSLogger log (gpslog, lcmlog, channel);
    log.parse_log ();
    std::cout << "all done" << std::endl;

    exit (EXIT_SUCCESS);
}

