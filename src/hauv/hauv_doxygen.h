/**
 * @page HAUV Hull Inspection Readme page

 This page is README for hull inspection project

<HR>

How to run seserver + rtvan from the same machine:
This is the set up we used in 2011 ONR demo in Panama city, FL
2011.07.18 Ayoung Kim

0) Installation
You need both seserver and rtvan installed properly on your machine.
Note that rtvan (perls code) is based on 10.04 LTS, and thus uses opencv 2.0,
whereas seserver (hauv-project code) uses opencv 2.2.0.

You need to install two different opencv on your machine to run them together.
(In this README, 10.04 LTS system is assumed for perls code user)

On perls side:
- Install opencv 2.0 from synaptic
- Check the opencv version
  <code>
  \verbatim
   $ pkg-config opencv --libs  # --> You should see -lcv -lhighgui -lcvaux -lml -lcxcore \endverbatim
  </code>
- Compile perls code with SIFTGPU option "on"
  (or you need to have the siftgpu-server on the network)
- Detailed compile instruction for perls is omitted. (See INSTALL in perls)

On hauv-project side:
- Download "OpenCV-2.2.0.tar.bz2" from
  http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.2/
- Install opencv 2.2 in user specified folder. Simply you can do:
  (http://opencv.willowgarage.com/wiki/InstallGuide)
  <code>
  \verbatim
   $ cd OpenCV-2.2.0
   $ mkdir release; cd release
   $ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-2.2.0 -D BUILD_PYTHON_SUPPORT=ON ..
   $ make
   (* If you encounter error from matchers.cpp line 45 <Eigen/Array>, change the line to <Eigen/Core>)
   $ sudo make install
   (* Note: the default is /usr/local you may specify your own name for it instead of opencv-2.2.0) 
   $ sudo cp /usr/local/opencv-2.2.0/lib/pkgconfig/opencv.pc /usr/local/lib/pkgconfig/opencv2.2.pc
   ( This will allow us to get gcc flags for opencv 2.2 using `pkg-config --cflags opencv2.2` )  \endverbatim
  </code>
- As root, append the following line to the file /etc/ld.so.conf:
  <code>
  \verbatim
   /usr/local/opencv-2.2.0/lib \endverbatim
  </code>
  and run
  <code>
  \verbatim
   $ sudo ldconfig \endverbatim
  </code>
  (If this step isn't done, hauv will still compile but the machine won't be able to execute the hauv binaries)

- svn co hauv project and follow the readme. Simply you can do:
  <code>
  \verbatim
   $ mkdir hauv-project; 
   $ svn co https://svn.csail.mit.edu/marine/projects/hauv-project hauv-project
   $ cd hauv-project; make checkout; \endverbatim
  </code>

- You need to modify a line of CMakeLists.txt for custom opencv 2.2.0
  Open hauv-project/hauv/src/navigator/CMakeLists.txt
  Add the following line at the beginning of the file
  <code>
  \verbatim
   include (/usr/local/opencv-2.2.0/share/opencv/OpenCVConfig.cmake) \endverbatim
  </code>

- Change the two lines which link sesonarclient with opencv 2.2 as follows:
  <code>
  \verbatim
   pods_use_pkg_config_packages(sesonarclient opencv) -> pods_use_pkg_config_packages(sesonarclient opencv2.2)
   pods_use_pkg_config_packages(sesonarclient opencv) -> pods_use_pkg_config_packages(sesonarclient opencv2.2) \endverbatim
  </code>

- Now compile and install hauv-project
  <code>
  \verbatim
   $ cd hauv-project
   $ make;  \endverbatim
  </code>

1) Prepare config (master.cfg)
- If you are running seserver and rtvan in the same machine:
  <code>
  \verbatim
   $ cd ${PERLS}/config
   $ ln -s huls3.cfg master.cfg
   $ cd ${HAUV-PROJECT}/build/config/
   $ ln -s ${PERLS}/config/huls3.cfg master.cfg  \endverbatim
  </code>

  so that both seserver and rtvan uses the same configuration.

- If you are running seserver and rtvan in different machine
  <code>
  \verbatim
   $ cd ${PERLS}/config
   $ ln -s huls3.cfg master.cfg
   $ cd ${HAUV-PROJECT}/build/config/
   $ ln -s huls3.cfg master.cfg \endverbatim
  </code>

  We did make double check before running the processes.

2) Run
You can run them separately, but the simplest way to run rtvan+seserver is to use procman.
In demo, we did:

- On toughbook
  (Drivers & logging to avoid time sync issue)
  <code>
  \verbatim
   $ cd perls/build/bin
   $ bot-procman-deputy . & \endverbatim
  </code>
  and did rest of the control by ssh from bigbook

- On hauv-laptop (MIT)
  <code>
  \verbatim
   $ cd {HAUV-PROJECT}/build/bin
   $ bot-procman-deputy . & \endverbatim
  </code>

- On bigbook (UMich)
  <code>
  \verbatim
   $ cd perls/build/bin
   $ bot-procman-deputy . &
   $ bot-procman-sheriff 
    (load perls/config/procman/procman-huls3.cfg \endverbatim
  </code>

* 3) Notes
* check van-processed folder and clean it up before running a mission
* System > Administration > NVIDIA X Setting : choose maximun performace instead of adaptive for GPU

<HR>

Instructions for reverting to old revision of <perls, hauv-project> for February 2012 demo.

On *bigbook*:

0) Check out perls rev 1715 to ~/perls-june

1) Make sure up-to-date perls is named something other than ~/perls (like ~/perls-svn)

2) Create symbolic link ~/perls -> ~/perls-june

3) cd ~/perls/third-party/build && ccmake .. && make && cd ~/perls/build/ && make

4) rm ~/perls && ln -s ~/perls-svn ~/perls

5) cd ~/perls/third-party/build && ccmake .. && make && cd ~/perls/build/ && make

6) rm ~/perls && ln -s ~/perls-june ~/perls

7) on *both* bigbook and hauv-mit, use the PanamaCity hauv-project (available on hauv-mit laptop) to launch hauv-viewer

8) on hauv-mit laptop, launch deputy from ~/hauv-project/build/bin where ~/hauv-project is the one from Panama City

9) use procman-huls3-seserver.cfg from ~/perls-svn to start processes from procman

To revert back to up-to-date perls, one simply has to create symbolic link ~/perls that points to ~/perls-svn.  Make sure that hauv-mit is running deputy from ~/perls/build/bin in this configuration

* @todo Add an option in isamserver to append utime to the original graph.isam file.
* @todo Complete sonar cone display to all three (split/half-split/3DFLS) configurations
* @todo Enable hauv pose attach renderer to select between isam and DR.
* @todo in hauv-client, we should check validity of DVL lock and cnv (use kinematics) before adding odometry
* @todo complete isam state thread in branch and merge it back to trunk
* @todo complete E matrix RANSAC and add it to twoview thread
* @todo need better node adding strategy in hauv-client. Currently, camera and sonar can add a node right next to each other.
* @todo test other robust optimization in isam. (e.g. dog-leg)
* @todo resolve rtvan missing lcm, plinks
*/

