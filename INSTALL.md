# PeRL Software Library

This is an alpha release of the PeRL Software library (perls).



### DIRECTORY STRUCTURE

<!-- ------------------------------------------------ -->
The directory hierarchy is as follows:
```
build - cmake build tree takes place here, all local install output gets dumped here
cmake - custom cmake modules go here
config - bot-param and other configuration files go here
lcmdefs - lcm definition files go here, organized by packages
matlab - matlab scripts go here
src - all C/C++/Java/Python source files go here, organized by folder within group
third-party - all third-party source files go here
```

### ENVIRONMENT VARIABLES 

<!-- -------------------------------------------------- -->

perls uses the following environment variables, if specified:
- LCM_DEFAULT_URL
- BOT_PARAM_SERVER_NAME

### JAVA
<!-- -------------------------------------------------- -->
NOTE: on 10.04 and below, lcm works best with SUN's JRE.  If you run
into any compile problems, make sure that your system is configured to
use the SUN JRE by default and not any of the other java alternatives.
```sh
sudo apt-get install sun-java6-jdk
sudo update-alternatives --config java
sudo update-alternatives --config javac
sudo update-alternatives --config jar
```

### ESSENTIAL PACKAGES 

<!-- -------------------------------------------------- -->
For your convience, run the bash script `sudo perls-essential.sh`,
located in the 'third-party' directory, to download all required
debian packages.
(The cmake build process will prompt you for any others that
may be missing.)


## BUILDING
<!-- -------------------------------------------------- -->
perls uses cmake-2.8* as the build environment.  all configuration and 
compilation should be done from within the perls/build folder.

Step 1: Build third-party libs first, e.g., lcm and libbot2 at a minimum

`cd perls/third-party/build`
`ccmake ..`
`make`

to uninstall do `make uninstall` and to do a clean build do `make clean`

Step 2: Build the perls software suite

`cd perls/build`
`ccmake ..`
`make`, will dump exes in perls/build/bin, libs in perls/build/lib, etc

To do a clean build, just execute `make clean` from this same directory. Or
if you want to erase all of the cmake generated build files and start
from scratch just do `rm -rf *` in the build folder.

Optional targets include:
`make docs`, will generate help documentation in perls/build/doxygen.
e.g., cd into perls/build/doxygen/html and user your favorite browser
to view index.html


*Note: for Ubuntu 8.04 hardy systems, cmake-2.4 is the default install.
To install cmake-2.6 simply issue the following command: 
sudo apt-get install -t hardy-backports cmake

### Building in 2018

Here is a valid sequence of instructions as of April 2018 under Ubuntu 16.04 LTS

**Sources** : previous section, hints & more recent post copied below

A major change is that the third-party section is no longer needed.

#### Structure

The following directory structure is assumed : 
```
~/acfr
    |- acfr-lcm
    \- acfr-mission
```
Where : 
- `acfr-lcm`  :  current repo
- `acfr-mission` : dependency of current repo (ACFR gitlab)

#### Required tools

Configuration is done with `cmake` and its graphical helper `ccmake`

```sh
sudo apt-get install cmake cmake-curses-gui
```

Computer should also have a valid python install with pip. No installation instructions are provided as these steps are evolving fast.

#### Install pre-compiled dependencies

```sh
wget http://www-personal.acfr.usyd.edu.au/lachlan/mirror/acfrmarine-keyring_xenial_all.deb
wget http://www-personal.acfr.usyd.edu.au/lachlan/mirror/acfrmarine-repo_xenial_all.deb
sudo dpkg -i acfrmarine-repo_xenial_all.deb
sudo dpkg -i acfrmarine-keyring_xenial_all.deb
sudo apt-get update
sudo apt-get install seabed-localisation # building acfr-lcm or other code that depends on these you will need the development packages to get the headers.
sudo apt-get install seabed-localisation-dev bot2-all liblcm1-dev vimba small libuf-dev
# (or just libflounder-dev / libplankton-dev as appropriate)
```

#### Compiling acfr-lcm `core`

By design, all cmake-generated files should not mess with the source code. All configuration and building steps are done within a `build` folder.

```sh
cd ~/acfr/acfr-lcm

# install repo dependencies
sudo ./getdeps.sh

mkdir build
cd build
ccmake ..
# navigate using described commands to select only needed modules
# > configure
# > generate
# exit ccmake
make
```

#### Compiling acfr-mission

The process is similar:
```sh
cd ~/acfr/acfr-mission

# install repo dependencies
sudo ./getdeps.sh
# these are python depencies
sudo pip install -r requirements.txt

mkdir build
cd build

ccmake ..
# there are here much less options, cmake could also do the work
make

```

If all went well, you can then install the code to make the headers available
```
sudo make install
```


#### Compiling acfr-lcm `acfr` module

Additionnal dependency:

```sh
sudo apt-get install libuf-dev
```

then re-configure throught `ccmake` and run `make`

#### Copy of latest installation instructions

> The current way to install precompiled versions of (most) of our code (putting this here so people can find it in the future):
```sh
wget http://www-personal.acfr.usyd.edu.au/lachlan/mirror/acfrmarine-keyring_xenial_all.deb
wget http://www-personal.acfr.usyd.edu.au/lachlan/mirror/acfrmarine-repo_xenial_all.deb
sudo dpkg -i acfrmarine-repo_xenial_all.deb
sudo dpkg -i acfrmarine-keyring_xenial_all.deb
sudo apt-get update
sudo apt-get install seabed-localisation
#If building acfr-lcm or other code that depends on these you will need the development packages to get the headers.
sudo apt-get install seabed-localisation-dev
# (or just libflounder-dev / libplankton-dev as appropriate)
```
> Or change xenial for the codename of your Ubuntu LTS or Debian stable/oldstable release (current jessie, trusty and stretch should (mostly!) work)
