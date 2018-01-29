#include <unistd.h>
#include "follow_planner2D.hpp"

void
print_help (int exval, char **argv)
{
    printf("Usage:%s [-h] [-n VEHICLE_NAME]\n\n", argv[0]);

    printf("  -h                               print this help and exit\n");
    printf("  -n VEHICLE_NAME                  set the vehicle_name\n");
    exit (exval);
}

void
parse_args (int argc, char **argv, FollowPlanner2D *fp)
{
    int opt;

    while ((opt = getopt (argc, argv, "hn:")) != -1)
    {
        switch(opt)
        {
        case 'h':
            print_help (0, argv);
            break;
        case 'n':
            fp->setVehicleName((char*)optarg);
            break;
         }
    }
}

int main(int argc, char **argv)
{
	// install the signal handler

	FollowPlanner2D *fp = new FollowPlanner2D();
        parse_args(argc, argv, fp);
        fp->subscribeChannels();

	if (!fp->loadConfig(basename(argv[0])))
	{
		cerr << "Failed loading config" << endl;
		return 0;
	}

    // this is a blocking call that only returns when
    // the program signals an exit condition
	fp->process();

	delete fp;

	return 0;
}

