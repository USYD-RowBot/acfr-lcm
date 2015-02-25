#!/usr/bin/env bash
PACKAGES=""
function addpkg {
    PACKAGES="$PACKAGES $@"
}

# needed in order to check distribution using 'bc' provided by by pkg and 
# 'add-apt-repository' provided by python-software-properties pkg (available for Ubuntu 9.10 and above)
sudo apt-get install bc python-software-properties

DIST=$(lsb_release -rs)
NAME=$(lsb_release -cs)
echo $DIST $NAME
DIST=$(echo "scale=0; $DIST * 100" | bc)
DIST=$(printf "%.0f" $DIST)

#=========================================================================
# 1) list common packages here in alphabetical order
#=========================================================================
addpkg \
    autoconf \
    automake \
    build-essential \
    ccache \
    dbus \
    dbus-x11 \
    doxygen \
    freeglut3-dev \
    gettext \
    gfortran \
    gnuplot \
    graphviz \
    gtk-doc-tools \
    libaa1-dev \
    libavcodec-dev \
    libavformat-dev \
    libblas-dev \
    libboost-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libdbus-1-dev \
    libdc1394-22-dev \
    libdevil-dev \
    libf2c2-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libglib2.0-dev \
    libglu1-mesa-dev \
    libgps-dev \
    libgsl0-dev \
    libgtk2.0-dev \
    liblapack-dev \
    libncurses-dev \
    libproj-dev \
    libsdl1.2-dev \
    libsuitesparse-dev \
    libraw1394-dev \
    libtiff4-dev \
    libwxgtk2.8-dev \
    libxi-dev \
    libxml2-dev \
    libxmu-dev \
    mesa-common-dev \
    python-dev \
    libsigc++-2.0-dev \
    libglibmm-2.4-dev \
    libxml++2.6-dev \
    sip-dev

#=========================================================================
# 2) list distribution-specific packages here in alphabetical order
#=========================================================================
# cmake
if [ $DIST -le 804 ]; then
    addpkg cmake-gui
else
    addpkg cmake-curses-gui
fi

# opencv
if [ $DIST -ge 1004 -a $DIST -lt 1204 ]; then
    addpkg libcv-dev libcvaux-dev libhighgui-dev
elif [ $DIST -ge 1204 ]; then
    addpkg libcv-dev libhighgui-dev libopencv-contrib-dev libopencv-gpu-dev
fi

# changed package name in 12.04
if [ $DIST -ge 1204 ]; then
    addpkg libcurl4-gnutls-dev
else
    addpkg libcurl4-dev
fi

# sun
if [ $DIST -ge 910 -a $DIST -lt 1204 ]; then
    sudo add-apt-repository "deb http://archive.canonical.com/ $NAME partner"
    addpkg sun-java6-jdk
elif [ $DIST -ge 1204 ]; then
    addpkg openjdk-6-jdk
fi

# go forth!
echo "apt-get install $PACKAGES"
sudo apt-get update
sudo apt-get install $PACKAGES

# configure java
sudo update-alternatives --config java
sudo update-alternatives --config jar
sudo update-alternatives --config javac
