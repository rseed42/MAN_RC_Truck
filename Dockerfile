FROM ros:lunar
LABEL maintainer "Sebastian Schwarz <sebastian.schwarz.b@man.eu>"
WORKDIR /home/ros
 
# Install necessary packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        python \
        python-dev \
        python-numpy \
        python-pip \
        python3 \
        python3-dev \
        python3-numpy \
        python3-pip \
        build-essential \
        cmake \
        libgtk2.0-dev \
        pkg-config \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libtbb2 \
	libtbb-dev \
	libjpeg-dev \
	libpng-dev \
	libtiff-dev \
	libjasper-dev \
	libdc1394-22-dev \
	ros-lunar-cv-bridge \
        && rm -rf /var/lib/apt/lists/*
 
# build
RUN git clone https://github.com/opencv/opencv.git \
    && git clone https://github.com/opencv/opencv_contrib.git \
    && mkdir opencv/release \
    && cd opencv/release \
    && cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D WITH_JASPER=OFF \
        -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        .. \
    && make -j 4 \
    && make install \
    && ldconfig \
    && cd ../.. \
    && rm -rf opencv \
    && rm -rf opencv_contrib
  
# Install necessary python packages
RUN pip install \
        pygame \
        attrdict \
    && git clone https://github.com/rseed42/MAN_RC_Truck.git