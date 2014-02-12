/*
 *  Load and parse a mission file into a set of goal points.
 *
 *  Christian Lees
 *  ACFR
 *  28/11/12
 */
 
 #include "mission.hpp"
 
 Mission::Mission()
 {
 }
 
 Mission::Mission(string _filename)
 {
    filename = _filename;
    load();
 }
 
 Mission::~Mission()
 {
 }
 
 // This function is used for if you want to load a mission after the mission
 // object has been created.  A public function that just sets the file name
 // and then calls the private load function
 int Mission::load(string _filename)
 {
    filename = _filename;
    return load();    
 }
 
 // Load a mission file and create a list of points
 int Mission::load()
 {
    // empty out the list
    waypoints.clear();
    
    
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
        xmlpp::Node::NodeList xml_globals;
        for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
        {
            if((*iter)->get_name().lowercase() == "primitive")
                xml_primitives.push_back(*iter);
            else if((*iter)->get_name().lowercase() == "global")
                xml_globals.push_back(*iter);            
            else if((*iter)->get_name() != "text")
                xml_other.push_back(*iter);
        }
        
        // extract the description, globals and location
        parseGlobals(xml_globals);
        
        // extract the actual mission
        for(xmlpp::Node::NodeList::iterator iter = xml_primitives.begin(); iter != xml_primitives.end(); ++iter)
        {
            xmlpp::Node::NodeList primitive_list = (*iter)->get_children();
            
            
            for(xmlpp::Node::NodeList::iterator i = primitive_list.begin(); i != primitive_list.end(); ++i)
            {
                Glib::ustring tag = (*i)->get_name().lowercase(); 
                if (tag == "goto" || tag == "gotoandcircle" || tag == "leg"
						|| tag == "zambonie" || tag == "spiral"
						|| tag == "spiral_inward")
					parsePrimitive(*i);
				else if((*i)->get_name().lowercase() != "text")
                    cerr << "Unknown mission primitive " << (*i)->get_name() << endl;
            }      
                
        }
    }
    return 1;
    
}

// Read the global variables
int Mission::parseGlobals(xmlpp::Node::NodeList &globals) {
    // Using this method allows us to have more then one global section
    // they are all processed at the start regardless of where in the file they occur
    double x;
    missionTimeout = NAN;
    
    for(xmlpp::Node::NodeList::iterator iter = globals.begin(); iter != globals.end(); ++iter) {
        xmlpp::Node::NodeList globalList = (*iter)->get_children();
        
        for(xmlpp::Node::NodeList::iterator i = globalList.begin(); i != globalList.end(); ++i) {
            const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*i);
            if(!element)
                continue;

            if(element->get_name().lowercase() == "drop_distance")
            {
                if(!getSingleValue(element, "m", dropDistance))
                    return 0;
            }
            
            else if(element->get_name().lowercase() == "drop_angle")
            {
                if(getSingleValue(element, "deg", x))
                    dropAngleRad = x / 180 * M_PI;
                else if(getSingleValue(element, "rad", x))
                    dropAngleRad = x;
                else  
                    return 0;
            }
            
            else if(element->get_name().lowercase() == "turn_radius")
            {
            	if(!getSingleValue(element, "m", turnRadius))
            		return 0;
            }

            else if(element->get_name().lowercase() == "mission_timeout")
            {
                if(!getSingleValue(element, "t", missionTimeout)) {
                    cerr << "No mission_timeout variable set in global section." << endl;
                    return 0;
                }
            }
            
            else if(element->get_name().lowercase() == "location")
            {
                if(!getSingleValue(element, "lat", originLat) || !getSingleValue(element, "lon", originLon)) {
                    cerr << "Origin not set, set the location variable." << endl;
                    return 0;            
                }
            }
            
        } 
        if(missionTimeout != missionTimeout) {   
            cerr << "No mission_timeout variable set in global section." << endl;
            return 0;
        }
    }
    return 1;    

}
// Read and XML node that contains an location such as point, start or end
int Mission::getXYZ(const xmlpp::Element* element, double &x, double &y, double &z)
{

    // set them to non values first so we can error check
    x = NAN;
    y = NAN;
    z = NAN;
    
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
int Mission::getVelocity(const xmlpp::Element* element, double &vx, double &vy, double &vz)
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


int Mission::getSingleValue(const xmlpp::Element* element, string tag, double &val)
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
int Mission::getSingleValue(const xmlpp::Element* element, string tag, Glib::ustring &val)
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
int Mission::getDepthMode(const xmlpp::Element* element)
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
            depthMode = DEPTH_MODE_ALTITUDE;
        if(mode.lowercase() == "depth")
            depthMode = DEPTH_MODE_DEPTH;
    }
    return 1;
}

// Read the command element and create a list of command that need to be executed
// at the way point
int Mission::getCommand(const xmlpp::Element* element)
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
        MissionCommand mc;
        if(device.lowercase() == "camera")
            mc.device = CAMERA;
        else if(device.lowercase() == "dvl")
            mc.device = DVL;
            
        // Camera specific
        if(mc.device == CAMERA)
        {
            if((*i).command.lowercase() == "freq")
            {
                mc.command = CAMERA_FREQ;
                mc.valueDouble = atof((*i).value.c_str());
            }            
            else if((*i).command.lowercase() == "width")
            {
                mc.command = CAMERA_WIDTH;
                mc.valueDouble = atof((*i).value.c_str());
            }            
            else if((*i).command.lowercase() == "onoff")
            {
		if ((*i).value.lowercase() == "start")
		{
                	mc.command = CAMERA_START;
		}
		else
		{
			mc.command = CAMERA_STOP;
		}
            }
        }
        
        // DVL Specific
        // TODO
        
        commands.push_back(mc);
     
    }
            
    return 1;        
}


int Mission::parsePrimitive(xmlpp::Node *node)
{
    // generate a grid pattern
    
    MissionPrimitive *mp;
    
    double x, y, z;
    double l, w, rot, s;
    
    Glib::ustring primitiveType = node->get_name().lowercase();
    
    if(primitiveType == "zambonie")
        mp = new ZamboniePath();
    else if(primitiveType == "spiral_inward")
        mp = new SpiralInwardPath();
    else if(primitiveType == "spiral")
        mp = new SpiralPath();
    else if(primitiveType == "goto")
        mp = new GotoPath();
    else if(primitiveType == "leg")
        mp = new LegPath();
    else if(primitiveType == "gotoandcircle")
    	mp = new GotoAndCirclePath();
    else 
        return 0;    
        
    // set from globals
    mp->setDropDist(dropDistance);
    mp->setDropAngleRad(dropAngleRad);     
    
    xmlpp::Node::NodeList list = node->get_children();     
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter) {
        const xmlpp::Element* element = dynamic_cast<const xmlpp::Element*>(*iter);
        if(element) {
            if(element->get_name().lowercase() == "position") {
                if(!getXYZ(element, x, y, z))
                    return 0;
                mp->setPosition(x, y, z);
                
            }
            if(element->get_name().lowercase() == "width")
            {
                if(!getSingleValue(element, "m", w))
                    return 0;
                
                mp->setWidth(w);
            }
            if(element->get_name().lowercase() == "length")
            {
                if(!getSingleValue(element, "m", l))
                    return 0;
                mp->setLength(l);
            }
            if(element->get_name().lowercase() == "spacing")
            {
                if(!getSingleValue(element, "m", s))
                    return 0;
                mp->setPathOffset(s);
            }
            if (element->get_name().lowercase() == "heading"
					|| element->get_name().lowercase() == "rotation")
            {
                if(getSingleValue(element, "deg", rot))
                    mp->setHeadingDeg(rot);
                else if(getSingleValue(element, "rad", rot))
                    mp->setHeadingRad(rot);
                else
				{
					cout << "Could not set heading/rotation" << endl;
                    return 0;
				}
            }
            if(element->get_name().lowercase() == "velocity")
            {
                if(!getVelocity(element, x, y, z)) 
                    return 0;
                mp->setVelocity(x);
            }
            // the default direction in cw
            if(element->get_name().lowercase() == "direction")
            {
                Glib::ustring str;
                if(getSingleValue(element, "d", str)) 
                {
                    if(str.lowercase() == "cw")
                        mp->setDirectionCW();
                    else if(str.lowercase() == "ccw")
                        mp->setDirectionCCW();
                }
                
            }
            if(element->get_name().lowercase() == "overlap")
            {
                if(getSingleValue(element, "m", x))
                    mp->setCenterOverlap(x);
                else if(getSingleValue(element, "deg", x))
                    mp->setCenterOverlapDeg(x);
                else
                    return 0;
            }
            if(element->get_name().lowercase() == "loops")
            {
                if(!getSingleValue(element, "n", x))
                    return 0;
                mp->setNumLoops(x);
            }
            if(element->get_name().lowercase() == "min_width_length")
            {
                if(!getSingleValue(element, "m", x))
                    return 0;
                mp->setMinWidthLength(x);
            }
            if(element->get_name().lowercase() == "pitch")
            {
                if(getSingleValue(element, "deg", rot))
                    mp->setPitchDeg(rot);
                else if(getSingleValue(element, "rad", rot))
                    mp->setPitchRad(rot);
                else  
                    return 0;
            }
            if(element->get_name().lowercase() == "altitude_change")
            {
                if(!getSingleValue(element, "m", x))
                    return 0;
                mp->setAltitudeChange(x);
            }
            if(element->get_name().lowercase() == "drop_distance")
            {
                if(!getSingleValue(element, "m", x))
                    return 0;
                mp->setDropDist(x);
                cout << "Drop distance = " << x << endl;    
            }
            
                
            
            if(element->get_name().lowercase() == "drop_angle")
            {
                if(getSingleValue(element, "deg", x))
                    mp->setDropAngleDeg(x);
                else if(getSingleValue(element, "rad", x))
                    mp->setDropAngleRad(x);
                else  
                    return 0;
            }
        
        
            if(element->get_name().lowercase() == "timeout")
            {
                if(!getSingleValue(element, "t", timeout))
                    return 0;
            }
            else if(element->get_name().lowercase() == "command")
            {
                if(!getCommand(element))
                    return 0;
            }
            else if(element->get_name().lowercase() == "depth")
            {
                if(!getDepthMode(element))
                    return 0;
            }
        
        }
        
        
        
    }

    mp->generatePath(timeout, depthMode, commands, waypoints.size());

    waypoints.splice(waypoints.end(), mp->getPath());   
    
    return 1;
}


int Mission::dump()
{
    for(list<waypoint>::iterator i=waypoints.begin(); i != waypoints.end(); ++i)
    {
     //   if((*i).goalType == GOAL)
            cout << (*i) << endl;

    }
    return 1;
}    

void Mission::dumpMatlab(string filename) {
    ofstream fout(filename.c_str(), ios::out);
    fout << "path = [" << endl;
	list<waypoint>::iterator it;
	for( it = waypoints.begin(); it != waypoints.end(); it++ ) {
		fout << (*it).pose.getX() << ", " << (*it).pose.getY() << ", " << (*it).pose.getZ() << "; " << endl;
	}
	fout << "];" << "\n";
//	fout << "figure; clf; grid on; axis equal; hold all;" << "\n";
//	fout << "plot3d( path', 'c' )" << endl;
//	for( it = waypoints.begin(); it != waypoints.end(); it++ ) {
//		fout << "pose = [" << (*it).pose.getX() << " " << (*it).pose.getX() + 0.2 * cos((*it).pose.getYawRad()) << "; "
//				<< (*it).pose.getY() << " " << (*it).pose.getY() + 0.2 * sin((*it).pose.getYawRad()) << "; "
//				<< (*it).pose.getZ() << " " << (*it).pose.getZ() << "]; " << endl;
//		//fout << "plot3d( path(" << i + 1 << ",:)', 'b-o' ); " << "\n";
//		fout << "plot3d( pose, 'm-' );" << endl;
//	}
//	fout << "scatter3( path(:,1), path(:,2), path(:,3), 20 )" << endl;
//	fout << "xlabel( 'x [m]' ); ylabel( 'y [m]' );" << "\n";
//	fout << "view(0,90)" << endl;

	//system("matlab matlab_plot.m");
}
