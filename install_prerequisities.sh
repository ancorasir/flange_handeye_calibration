#!/usr/bin/env bash

sudo apt-get -y update \
    && sudo apt-get -y upgrade \
    && sudo apt-get install -y vim screen tree ssh

sudo apt -y install software-properties-common \
	&& sudo add-apt-repository -y ppa:freecad-maintainers/freecad-stable \
	&& sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test \
	&& sudo add-apt-repository -y ppa:git-core/ppa \
	&& sudo apt update \
	&& sudo apt -y upgrade \
	&& sudo apt -y install g++ g++-4.9 gcc-4.9 \
	&& sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9 \
	&& sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9 \
	&& sudo apt -y remove git \
	&& sudo apt -y install git cmake cmake-curses-gui cmake-qt-gui libgtk2.0-dev pkg-config doxygen graphviz chrpath \
	libavcodec-dev libavformat-dev libswscale-dev libxt-dev python-numpy \
	libeigen3-dev libflann-dev libusb-1.0-0-dev libqhull-dev libtiff5-dev libavahi-client-dev \
	libbz2-dev makeself mesa-common-dev python2.7-dev gawk dh-autoreconf \
	liboce-foundation-dev liboce-modeling-dev liboce-ocaf-dev libxerces-c-dev

sudo add-apt-repository -y ppa:webupd8team/sublime-text-3 \
	&& sudo apt update \
	&& sudo apt -y upgrade \
	&& sudo apt -y install meshlab sublime-text-installer freecad searchmonkey \
	&& sudo apt -y install libxmlrpc-c++8 libxmlrpc-c++8-dev

wget https://github.com/NixOS/patchelf/archive/0.9.tar.gz
tar -xzvf 0.9.tar.gz
cd patchelf-0.9
./bootstrap.sh
./configure
sudo make install