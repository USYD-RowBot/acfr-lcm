#include <iostream>

#include "perls-owtt-nav/index.h"

using perls::operator<<;

int main (int argc, char *argv[])
{
    perls::Index index;
    std::cout << "created index" << std::endl 
        << index << std::endl;

    index.add ("base");
    index.add ("extras");
    std::cout << "added base and extras" << std::endl
       << index << std::endl;

    index("base").add ("xy", 2);
    std::cout << "added xy of size 2 to base" << std::endl 
        << index << std::endl;

    std::cout << "index[\"base\"] = " << index["base"] << std::endl;
    std::cout << "index(\"base\"):" << std::endl 
        << index("base") << std::endl;
    std::cout << "index(\"base\")[\"xy\"] = " << index("base")["xy"] << std::endl;

    index("extras").add ("delayed");
    perls::Index& delayed_index = index("extras")("delayed");
    std::cout << "added delayed" << std::endl
        << index("extras")("delayed") << std::endl;

    delayed_index.add ("D1",2);
    std::cout << "added extras->delayed->D1" << std::endl 
        << index("extras") << std::endl
        << index("extras")("delayed") << std::endl
        << delayed_index("D1") << std::endl;
    std::cout << "index(\"extras\")(\"delayed\")[\"D1\"] = " << delayed_index["D1"] << std::endl;

    delayed_index.insert ("D1","D2",3);
    std::cout << "inserted D2 before D1" << std::endl;
    std::cout << index << std::endl;
    std::cout << index("extras")("delayed") << std::endl;
    std::cout << delayed_index("D1") << std::endl;
    std::cout << delayed_index("D2") << std::endl;
    std::cout << "index[\"base\"] = " << index["base"] << std::endl;
    std::cout << "index[\"extras\"] = " << index["extras"] << std::endl;
    std::cout << "index(\"extras\")[\"delayed\"] = " << index("extras")["delayed"] << std::endl;
    std::cout << "index(\"extras\")(\"delayed\")[\"D1\"] = " << index("extras")("delayed")["D1"] << std::endl;
    std::cout << "index(\"extras\")(\"delayed\")[\"D2\"] = " << index("extras")("delayed")["D2"] << std::endl;

    std::cout << "index[\"all\"] = " << index["all"].size () << std::endl;

    return 0;
}
