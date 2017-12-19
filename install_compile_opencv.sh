# install and compile openCV
# see here: https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html

# ubuntu packages
sudo apt-get install build-essential  # compiler
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev  # required
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev  # optional

# get openCV and openCV contrib (for ArUco)
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# build
cd ~/opencv
mkdir release
cd release

# exclude jasper (not available on ubuntu 17.04+)
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_JASPER=OFF -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
make -j 4
sudo make install

