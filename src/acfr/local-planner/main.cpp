#include "local_planner.hpp"

int main(int argc, char **argv)
{
	// install the signal handler

	LocalPlanner *lp = new LocalPlanner();
	if (!lp->loadConfig(basename(argv[0])))
	{
		cerr << "Failed loading config" << endl;
		return 0;
	}

    // this is a blocking call that only returns when
    // the program signals an exit condition
	lp->process();

	delete lp;

	return 0;
}

