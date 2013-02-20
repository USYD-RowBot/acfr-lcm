
#define TIXML_USE_STL

//#include <tinyxml.h>
#include <libxml++/libxml++.h>
#include <string>
#include "mission_primatives.hpp"

#ifndef MISSION_HPP
#define MISSION_HPP

class raw_command
{
    public:
        Glib::ustring command;
        Glib::ustring value;
};
    

class mission
{
    public:
        mission();
        mission(string filename);
        ~mission();
        
        int load(string filename);
        
        goal_point get_next();
        goal_point get_index(int index);
        int dump();
        
        // the list of goal points
        list<goal_point> goal_points;
        
    private:
        string filename;
        int load();
        
        // helper functions
        int grid_to_points();
        int get_xyzrph(const xmlpp::Element* element, double &x, double &y, double &z, double &r, double &p, double &h);
        int get_velocity(const xmlpp::Element* element, double &vx, double &vy, double &vz);
        int get_timeout(const xmlpp::Element* element, double &t);
        int get_common(xmlpp::Node *node, mission_primative &primative);
        int get_command(const xmlpp::Element* element, list<mission_command> &commands);
        int get_single_value(const xmlpp::Element* element, string tag, double &val);
        int get_depth_mode(const xmlpp::Element* element, depth_mode_t &dm);
        int get_single_value(const xmlpp::Element* element, string tag, Glib::ustring &val);
        
        // parsers
        int parse_line(xmlpp::Node *node);
        int parse_grid(xmlpp::Node *node);
        int parse_goto(xmlpp::Node *node);
        int parse_hold(xmlpp::Node *node);
        int parse_command(xmlpp::Node *node);
        int parse_zambonie(xmlpp::Node *node);
        
        // Conversion
        int primatives_to_points();
        int grid_to_points(mission_grid &grid);
        int line_to_points(mission_line &line);
        
};


#endif
