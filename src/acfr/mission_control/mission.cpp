/*
 *  Load and parse a mission file into a set of goal points.
 *
 *  Christian Lees
 *  ACFR
 *  28/11/12
 */
 
 
 #include "mission.hpp"
 
 mission::mission()
 {
 }
 
 mission::mission(string _filename)
 {
    filename = _filename;
    load();
 }
 
 mission::~mission()
 {
 }
 
 // This function is used for if you want to load a mission after the mission
 // object has been created.  A public function that just sets the file name
 // and then calls the private load function
 int mission::load(string _filename)
 {
    filename = _filename;
    return load();    
 }
 
 // Load a mission file and create a list of points
 int mission::load()
 {
    // empty out the list
    goal_points.clear();
    
    
    xmlpp::DomParser parser;
    parser.parse_file(filename);
    if(parser)
    {
        //Walk the tree:
        const xmlpp::Node* root_node = parser.get_document()->get_root_node(); //deleted by DomParser.
        xmlpp::Node::NodeList list = root_node->get_children();
        
        if(root_node->get_name() != "mission")
        {
            cerr << "root tag is not mission" << endl;
            return 0;
        }
        
        xmlpp::Node::NodeList xml_other;
        xmlpp::Node::NodeList xml_primitives;
        for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
        {
            if((*iter)->get_name().lowercase() == "primitive")
                xml_primitives.push_back(*iter);
            else if((*iter)->get_name() != "text")
                xml_other.push_back(*iter);
        }
        
        // extract the description, globals and location
        
        
        // extract the actual mission
        for(xmlpp::Node::NodeList::iterator iter = xml_primitives.begin(); iter != xml_primitives.end(); ++iter)
        {
            xmlpp::Node::NodeList primitive_list = (*iter)->get_children();
            
            
            for(xmlpp::Node::NodeList::iterator i = primitive_list.begin(); i != primitive_list.end(); ++i)
            {
                if((*i)->get_name().lowercase() == "goto")
                    parse_goto(*i);
                else if((*i)->get_name().lowercase() == "grid")
                    parse_grid(*i);
                else if((*i)->get_name().lowercase() == "line")
                    parse_line(*i);
                else if((*i)->get_name().lowercase() == "command")
                    parse_command(*i);
                else if((*i)->get_name().lowercase() == "hold")
                    parse_hold(*i);
                else if((*i)->get_name().lowercase() == "zambonie")
                    parse_zambonie(*i);
                else if((*i)->get_name().lowercase() != "text")
                    cerr << "Unknown mission primitive " << (*i)->get_name() << endl;
            }      
                
        }
    }
    
}


// Read and XML node that contains an location such as point, start or end
int mission::get_xyzrph(const xmlpp::Element* element, double &x, double &y, double &z, double &r, double &p, double &h)
{

    // set them to non values first so we can error check
    x = NAN;
    y = NAN;
    z = NAN;
    r = 0;  // These need to default to 0 otherwise things get broken
    p = 0;  // we will have to check else where that if the heading is 0
    h = 0;  // then we decide what it really is
    
    // loop through the list and fill in the data
    const xmlpp::Element::AttributeList& attributes = element->get_attributes();
    for(xmlpp::Element::AttributeList::const_iterator i = attributes.begin(); i != attributes.end(); ++i)
    {
        if((*i)->get_name().lowercase() == "x")
            x = atof((*i)->get_value().c_str());
        else if((*i)->get_name().lowercase() == "y")
            y = atof((*i)->get_value().c_str());
        else if((*i)->get_name().lowercase() == "z")
            z = atof((*i)->get_value().c_str());
        else if((*i)->get_name().lowercase() == "r")
            r = atof((*i)->get_value().c_str()) / 180 * M_PI;
        else if((*i)->get_name().lowercase() == "p")
            p = atof((*i)->get_value().c_str()) / 180 * M_PI;
        else if((*i)->get_name().lowercase() == "h")
            h = atof((*i)->get_value().c_str()) / 180 * M_PI;
    }

    // here we only check x, y amd z as r, p and h are optional
    if((x != x) || (y != y) || (z != z))
    {
        cerr << "Invalid or missing position value, line " << element->get_line() << endl;
        return 0;
    }
    else
        return 1;
}

// As we support 3D velocities we need to read in three of them
// at least one needs to be valid
int mission::get_velocity(const xmlpp::Element* element, double &vx, double &vy, double &vz)
{
    vx = NAN;
    vy = NAN;
    vz = NAN;

    // loop through the list and fill in the data
    const xmlpp::Element::AttributeList& attributes = element->get_attributes();
    for(xmlpp::Element::AttributeList::const_iterator i = attributes.begin(); i != attributes.end(); ++i)
    {
        if((*i)->get_name().lowercase() == "x")
            vx = atof((*i)->get_value().c_str());
        else if((*i)->get_name().lowercase() == "y")
            vy = atof((*i)->get_value().c_str());
        else if((*i)->get_name().lowercase() == "z")
            vz = atof((*i)->get_value().c_str());
    }

    // check to make sure at least one is valid
    if((vx != vx) && (vy != vy) && (vz != vz))
    {
        cerr << "Invalid or missing velocity value, line " << element->get_line() << endl;
        return 0;
    }
    else
        return 1;
}

// Read a timeout value
int mission::get_timeout(const xmlpp::Element* element, double &t)
{

    t = NAN;
    const xmlpp::Element::AttributeList& attributes = element->get_attributes();
    for(xmlpp::Element::AttributeList::const_iterator i = attributes.begin(); i != attributes.end(); ++i)
    {
        if((*i)->get_name().lowercase() == "t")
            t = atof((*i)->get_value().c_str());
    }
    
    if(t != t)
    {
        cerr << "Invalid or missing timeout value, line " << element->get_line() << endl;
        return 0;
    }
    else
        return 1;
}

int mission::get_single_value(const xmlpp::Element* element, string tag, double &val)
{
    val = NAN;
    const xmlpp::Element::AttributeList& attributes = element->get_attributes();
    for(xmlpp::Element::AttributeList::const_iterator i = attributes.begin(); i != attributes.end(); ++i)
    {
        if((*i)->get_name().lowercase() == tag)
            val = atof((*i)->get_value().c_str());
    }
    
    if(val != val)
    {
        cerr << "Invalid or missing " << tag << " value, line " << element->get_line() << endl;
        return 0;
    }
    else
        return 1;
}

// read a single string value
int mission::get_single_value(const xmlpp::Element* element, string tag, Glib::ustring &val)
{

    const xmlpp::Element::AttributeList& attributes = element->get_attributes();
    for(xmlpp::Element::AttributeList::const_iterator i = attributes.begin(); i != attributes.end(); ++i)
    {
        if((*i)->get_name().lowercase() == tag)
        {
        
            val = (*i)->get_value();
            return 1;
        }
        else
            return 0;
    }
    // if we end up here then the tag wasn't there
    return 0;
}
    


// Read the depth mode variable, if its not set then we are in depth mode
int mission::get_depth_mode(const xmlpp::Element* element, depth_mode_t &dm)
{
    Glib::ustring mode;
    const xmlpp::Element::AttributeList& attributes = element->get_attributes();
    for(xmlpp::Element::AttributeList::const_iterator i = attributes.begin(); i != attributes.end(); ++i)
    {
        if((*i)->get_name().lowercase() == "mode")
            mode = (*i)->get_value();
    }
    
    if(mode.size() == 0)
    {
        cerr << "Invalid or missing depth mode value, line " << element->get_line() << endl;
        return 0;
    }
    else
    {
        if(mode.lowercase() == "altitude")
            dm = DEPTH_MODE_ALTITUDE;
    }
    return 1;
}

// Read the command element and create a list of command that need to be executed
// at the way point
int mission::get_command(const xmlpp::Element* element, list<mission_command> &commands)
{
    list<raw_command> cs;
    Glib::ustring device;

    // Get the device name in question and create a list of commands
    const xmlpp::Element::AttributeList& attributes = element->get_attributes();
    for(xmlpp::Element::AttributeList::const_iterator i = attributes.begin(); i != attributes.end(); ++i)
    {
        if(element)
        {
            if((*i)->get_name().lowercase() == "device")
                device = (*i)->get_value();
            else
            {        
                // get the rest
                raw_command c;
                c.command = (*i)->get_name();
                c.value = (*i)->get_value();
                cs.push_back(c);
            }
        }
    }
    // Lets put the command all together
    for(list<raw_command>::iterator i = cs.begin(); i != cs.end(); ++i)
    {
        mission_command mc;
        if(device.lowercase() == "camera")
            mc.device = CAMERA;
        else if(device.lowercase() == "dvl")
            mc.device = CAMERA;
            
        // Camera specific
        if(mc.device == CAMERA)
        {
            if((*i).command.lowercase() == "freq")
            {
                mc.command = CAMERA_RATE;
                mc.value_double = atof((*i).value.c_str());
            }            
            else if((*i).command.lowercase() == "time")
            {
                mc.command = CAMERA_STROBE_DURATION;
                mc.value_double = atof((*i).value.c_str());
            }            
            else if((*i).command.lowercase() == "onoff")
            {
                mc.command = ON_OFF;
                if((*i).value.lowercase() == "on")
                    mc.value_int = 1;
                else
                    mc.value_int = 0;
            }
        }
        
        // DVL Specific
        // TODO
        
        commands.push_back(mc);
     
    }
            
            
}

// Get all the primitive attributes that are common to all primitives
int mission::get_common(xmlpp::Node *node, mission_primitive &primitive)
{
    // Set some defaults
    primitive.depth_mode = DEPTH_MODE_DEPTH;

    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)   
    {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element)
        {
            if(element->get_name().lowercase() == "timeout")
            {
                if(!get_timeout(element, primitive.timeout))
                    return 0;
            }
            else if(element->get_name().lowercase() == "command")
            {
                if(!get_command(element, primitive.commands))
                    return 0;
            }
            else if(element->get_name().lowercase() == "depth")
            {
                if(!get_depth_mode(element, primitive.depth_mode))
                    return 0;
            }

        }
    }
    
    return 1;
}

int mission::parse_line(xmlpp::Node *node)
{
    double x, y, z, r, p, h;
    
    mission_line ml;
    
    // a line consist of two way points
    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)   
    {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element)
        {
            if(element->get_name().lowercase() == "start")
            {
                if(!get_xyzrph(element, x, y, z, r, p, h))
                    return 0;
                ml.start.setPosition(x, y, z);
                ml.start.setRollPitchYawRad(r, p, h);
            }
            else if(element->get_name().lowercase() == "end")
            {
                if(!get_xyzrph(element, x, y, z, r, p, h))
                    return 0;
                ml.end.setPosition(x, y, z);
                ml.end.setRollPitchYawRad(r, p, h);
            }
            else if(element->get_name().lowercase() == "start_velolcity")
            {
                if(!get_velocity(element, ml.start_vel[0], ml.start_vel[1], ml.start_vel[2])) 
                    return 0;
            }
            else if(element->get_name().lowercase() == "end_velolcity	")
            {
                if(!get_velocity(element, ml.start_vel[0], ml.start_vel[1], ml.start_vel[2])) 
                    return 0;
            }
        }
    }
    
    // get the timeout, tags, commands etc
    get_common(node, ml);
    
    // convert to points
    ml.to_points(goal_points);
}
    
int mission::parse_goto(xmlpp::Node *node)
{
    // a goto is a single way point
    double x, y, z, r, p, h;
    
    mission_goto mg;

    // a line consist of two way points
    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)   
    {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element)
        {
            if(element->get_name().lowercase() == "end")
            {
                if(!get_xyzrph(element, x, y, z, r, p, h))
                    return 0;
                mg.end.setPosition(x, y, z);
                mg.end.setRollPitchYawRad(r, p, h);
            }
            else if(element->get_name().lowercase() == "velocity")
            {
                if(!get_velocity(element, mg.vel[0], mg.vel[1], mg.vel[2])) 
                    return 0;
            }
        }
    }
    
    // get the timeout, tags, commands etc
    get_common(node, mg);
    mg.to_points(goal_points);
}

int mission::parse_grid(xmlpp::Node *node)
{
    // generate a grid pattern
    
    mission_grid mg;
    double x, y, z, r, p, h;
    double l, w, rot, num_lines;
    
    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)   
    {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element)
        {
            if(element->get_name().lowercase() == "center")
            {
                if(!get_xyzrph(element, x, y, z, r, p, h))
                    return 0;
                mg.center.setPosition(x, y, z);
                mg.center.setRollPitchYawRad(r, p, h);
            }
            if(element->get_name().lowercase() == "width")
            {
                if(!get_single_value(element, "w", w))
                    return 0;
                
                mg.width = w;
            }
            if(element->get_name().lowercase() == "length")
            {
                if(!get_single_value(element, "l", l))
                    return 0;
                mg.length = l;
            }
            if(element->get_name().lowercase() == "lines")
            {
                if(!get_single_value(element, "lines", num_lines))
                    return 0;
                mg.number_lines = num_lines;
            }
            if(element->get_name().lowercase() == "rotation")
            {
                if(!get_single_value(element, "r", rot))
                    return 0;
                mg.rotation = rot / 180 * M_PI;
            }
            if(element->get_name().lowercase() == "velocity")
            {
                if(!get_velocity(element, mg.velocity, w, l)) 
                    return 0;
            }
        }
    }
    get_common(node, mg);
    mg.to_points(goal_points);
    return 1;


}

int mission::parse_zambonie(xmlpp::Node *node)
{
    // generate a grid pattern
    
    mission_zambonie mz;
    double x, y, z, r, p, h;
    double l, w, rot, s;
    
    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)   
    {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element)
        {
            if(element->get_name().lowercase() == "center")
            {
                if(!get_xyzrph(element, x, y, z, r, p, h))
                    return 0;
                mz.center.setPosition(x, y, z);
                mz.center.setRollPitchYawRad(r, p, h);
            }
            if(element->get_name().lowercase() == "width")
            {
                if(!get_single_value(element, "w", w))
                    return 0;
                
                mz.width = w;
            }
            if(element->get_name().lowercase() == "length")
            {
                if(!get_single_value(element, "l", l))
                    return 0;
                mz.length = l;
            }
            if(element->get_name().lowercase() == "spacing")
            {
                if(!get_single_value(element, "s", s))
                    return 0;
                mz.spacing = s;
            }
            if(element->get_name().lowercase() == "rotation")
            {
                if(!get_single_value(element, "r", rot))
                    return 0;
                mz.rotation = rot / 180 * M_PI;
            }
            if(element->get_name().lowercase() == "velocity")
            {
                if(!get_velocity(element, mz.velocity, w, l)) 
                    return 0;
            }
            // the default direction in cw
            mz.direction = mission_primitive::direction_cw;
            if(element->get_name().lowercase() == "direction")
            {
                Glib::ustring str;
                if(get_single_value(element, "d", str)) 
                {
                    if(str.lowercase() == "cw")
                        mz.direction = mission_primitive::direction_cw;
                    else if(str.lowercase() == "ccw")
                        mz.direction = mission_primitive::direction_ccw;
                }
                
            }
        }
    }
    get_common(node, mz);
    mz.to_points(goal_points);
    
    return 1;
}
   
int mission::parse_hold(xmlpp::Node *node)
{
    // this is a special case of a goto with a time that we will stay there
    mission_hold mh;
    double x, y, z, r, p, h;
    double t;
    
    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)   
    {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element)
        {
            if(element->get_name().lowercase() == "pos")
            {
                if(!get_xyzrph(element, x, y, z, r, p, h))
                    return 0;
                mh.position.setPosition(x, y, z);
                mh.position.setRollPitchYawRad(r, p, h);
            }
            if(element->get_name().lowercase() == "time")
            {
                if(!get_single_value(element, "t", t))
                    return 0;
                mh.time = t;
            }
        }
    }
    get_common(node, mh);
    mh.to_points(goal_points);
    return 1;
            
}

int mission::parse_command(xmlpp::Node *node)
{
    // no motion command, just a command to control something on the auv
    // we will just use the mission_primitive base class
    mission_com mc;
    
    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)   
    {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element)
        {
            if(element->get_name().lowercase() == "command")
            {
                if(!get_command(element, mc.commands))
                    return 0;
            }
        }
    }
    get_common(node, mc);
    mc.to_points(goal_points);
    return 1;
    
}

int mission::dump()
{
    for(list<goal_point>::iterator i=goal_points.begin(); i != goal_points.end(); ++i)
    {
        if((*i).type == GOAL)
            cout << (*i) << endl;

    }
}    

    
