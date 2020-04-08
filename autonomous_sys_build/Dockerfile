# noVNC + TurboVNC + VirtualGL
# Useful links for the software we are using:
# http://novnc.com
# https://turbovnc.org
# https://virtualgl.org

# For container without CUDA support, use this image
#FROM nvidia/opengl:1.0-glvnd-runtime

# NVIDIA with CUDA support 
FROM nvidia/cudagl:10.0-base-ubuntu16.04

ENV NCCL_VERSION 2.4.2
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-libraries-$CUDA_PKG_VERSION \
        cuda-libraries-dev-$CUDA_PKG_VERSION \
        cuda-nvml-dev-$CUDA_PKG_VERSION \
        cuda-minimal-build-$CUDA_PKG_VERSION \
        cuda-command-line-tools-$CUDA_PKG_VERSION \
        libnccl-dev=$NCCL_VERSION-1+cuda10.0 \
        cuda-nvtx-$CUDA_PKG_VERSION \
        libnccl2=$NCCL_VERSION-1+cuda10.0

ARG SOURCEFORGE=https://sourceforge.net/projects
ARG TURBOVNC_VERSION=2.1.2
ARG VIRTUALGL_VERSION=2.5.2
ARG LIBJPEG_VERSION=1.5.2
ARG WEBSOCKIFY_VERSION=0.8.0
ARG NOVNC_VERSION=1.0.0

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ca-certificates \
        curl wget emacs vim less sudo lsof net-tools git htop gedit gedit-plugins \
	unzip zip psmisc xz-utils \
	libglib2.0-0 libxext6 libsm6 libxrender1 \
	libpython-dev libsuitesparse-dev libeigen3-dev libsdl1.2-dev doxygen graphviz libignition-math2-dev \
        gcc \
        libc6-dev \
        libglu1 \
        libglu1:i386 \
        libxv1 \
        libxv1:i386 \
	lubuntu-desktop xvfb xterm terminator zenity mesa-utils \
        make cmake \
        python \
        python-numpy \
        x11-xkb-utils \
        xauth \
        xfonts-base \
        xkb-data && \
    rm -rf /var/lib/apt/lists/*

RUN cd /tmp && \
    curl -fsSL -O ${SOURCEFORGE}/turbovnc/files/${TURBOVNC_VERSION}/turbovnc_${TURBOVNC_VERSION}_amd64.deb \
        -O ${SOURCEFORGE}/libjpeg-turbo/files/${LIBJPEG_VERSION}/libjpeg-turbo-official_${LIBJPEG_VERSION}_amd64.deb \
        -O ${SOURCEFORGE}/virtualgl/files/${VIRTUALGL_VERSION}/virtualgl_${VIRTUALGL_VERSION}_amd64.deb \
        -O ${SOURCEFORGE}/virtualgl/files/${VIRTUALGL_VERSION}/virtualgl32_${VIRTUALGL_VERSION}_amd64.deb && \
    dpkg -i *.deb && \
    rm -f /tmp/*.deb && \
    sed -i 's/$host:/unix:/g' /opt/TurboVNC/bin/vncserver

# Install miniconda
RUN cd /tmp && \
    curl -fsSL -O https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    chmod u+x Miniconda3-latest-Linux-x86_64.sh && \
    ./Miniconda3-latest-Linux-x86_64.sh -b

ENV PATH ${PATH}:/opt/VirtualGL/bin:/opt/TurboVNC/bin

RUN curl -fsSL https://github.com/novnc/noVNC/archive/v${NOVNC_VERSION}.tar.gz | tar -xzf - -C /opt && \
    curl -fsSL https://github.com/novnc/websockify/archive/v${WEBSOCKIFY_VERSION}.tar.gz | tar -xzf - -C /opt && \
    mv /opt/noVNC-${NOVNC_VERSION} /opt/noVNC && \
    chmod -R a+w /opt/noVNC && \
    mv /opt/websockify-${WEBSOCKIFY_VERSION} /opt/websockify && \
    cd /opt/websockify && make && \
    cd /opt/noVNC/utils && \
    ln -s /opt/websockify

COPY xorg.conf /etc/X11/xorg.conf
COPY index.html /opt/noVNC/index.html

# Defeat screen locking and power management
#RUN gsettings set org.gnome.desktop.lockdown disable-lock-screen true
RUN mv /etc/xdg/autostart/light-locker.desktop /etc/xdg/autostart/light-locker.desktop_bak
RUN mv /etc/xdg/autostart/xfce4-power-manager.desktop /etc/xdg/autostart/xfce4-power-manager.desktop_bak

# Expose whatever port NoVNC will serve from. In our case it will be 40001, see ./start_desktop.sh
EXPOSE 40001
ENV DISPLAY :1

# Install desktop file for this user
RUN mkdir -p /root/Desktop
COPY ./terminator.desktop /root/Desktop
RUN mkdir -p /root/.config/terminator
COPY ./terminator_config /root/.config/terminator/config
COPY ./chrome.desktop /root/Desktop
COPY ./galculator.desktop /root/Desktop
COPY ./htop.desktop /root/Desktop
COPY ./self.pem /root/self.pem
# Precede bash on all new terminator shells with vglrun so that 3d graphics apps will use the GPU
RUN perl -pi -e 's/^Exec=terminator$/Exec=terminator -e "vglrun bash"/g' /usr/share/applications/terminator.desktop

COPY ./background.png /usr/share/lubuntu/wallpapers/1604-lubuntu-default-wallpaper.png

RUN mkdir -p /root/.vnc
COPY ./xstartup.turbovnc /root/.vnc/xstartup.turbovnc
RUN chmod a+x /root/.vnc/xstartup.turbovnc

# Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
    /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update -y && \
    apt-get install -y ros-kinetic-desktop-full && \
    apt-get install -y ros-kinetic-navigation && \
    apt-get install -y ros-kinetic-roscpp && \
    apt-get install -y ros-kinetic-joy && \
    apt-get install -y ros-kinetic-kobuki-safety-controller && \
    apt-get install -y ros-kinetic-yocs-velocity-smoother && \
    apt-get install -y ros-kinetic-geometry-msgs && \
    apt-get install -y ros-kinetic-yocs-cmd-vel-mux && \
    apt-get install -y ros-kinetic-diagnostic-aggregator && \
    apt-get install -y ros-kinetic-depthimage-to-laserscan && \
    apt-get install -y ros-kinetic-gazebo-ros && \
    apt-get install -y ros-kinetic-kobuki-gazebo-plugins && \
    apt-get install -y ros-kinetic-robot-pose-ekf && \
    apt-get install -y ros-kinetic-robot-state-publisher && \
    apt-get install -y ros-kinetic-geodesy ros-kinetic-hector-gazebo && \
    apt-get install -y ros-kinetic-rosbridge* && \
    apt-get install -y ros-kinetic-opencv-apps && \
    apt-get install -y ros-kinetic-tf2-geometry-msgs && \
    apt-get install -y ros-kinetic-web-video-server && \
    apt-get install -y ros-kinetic-xacro && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

# Upgrade gazebo to gazebo v7
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update -y && apt-get install -y gazebo7

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils locales

# Fix LANG issues
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen
RUN locale-gen
RUN echo "export LC_ALL=en_US.UTF-8" >> /root/.bashrc
RUN echo "export LANG=en_US.UTF-8" >> /root/.bashrc
RUN echo "export LANGUAGE=en_US.UTF-8" >> /root/.bashrc


# Add miniconda to root's PATH
RUN echo "export PATH=/root/miniconda3:$PATH" >> /root/.bashrc

# Create some useful default aliases
RUN printf "%s\n" \
           "alias cp=\"cp -i\"" \
           "alias mv=\"mv -i\"" \
           "alias rm=\"rm -i\"" >> /root/.bash_aliases

# OpenUAV 
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends \
        apt-utils \
        git \
        build-essential \
        curl \
        geographiclib-tools \
        libcurl3-dev \
        libfreetype6-dev \
        libpng12-dev \
        libzmq3-dev \
        pkg-config \
        python-dev \
        python3-pip \
        rsync \
        software-properties-common \
        unzip \
        zip \
        zlib1g-dev \
        openjdk-8-jdk \
        openjdk-8-jre-headless \
        uwsgi \
        python-setuptools \
        python3-setuptools \
        python-future \
        python-lxml \
        python-rosdep \
        python-rosinstall-generator \
        python-wstool \
        libopencv-dev \
        gstreamer1.0-* \
        libgstreamer1.0-* \ 
        libgstreamer-plugins-base1.0-dev \
        libignition-math2-dev \
        libprotobuf-dev \
        libprotoc-dev \
        libgeographic-dev \
        protobuf-compiler \
        uwsgi-src \
        ros-kinetic-control-toolbox \
        uwsgi-plugin-python3 \
        python-pip \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip
RUN rm -rf /etc/ros/rosdep/sources.list.d/*
RUN rosdep init && rosdep update
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN sudo pip install -U rosinstall catkin_tools
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws

## Initialise wstool
RUN wstool init ~/catkin_ws/src

## Build MAVROS
### Get source (upstream - released)
RUN rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
### Get latest released mavlink package
RUN rosinstall_generator --rosdistro kinetic --upstream mavros | tee -a /tmp/mavros.rosinstall
### Setup workspace & install deps
RUN wstool merge -t src /tmp/mavros.rosinstall
RUN wstool update -t src -j$(nproc)
RUN rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial
RUN ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Install everything again for Python 3 because Firmware compiles using it.
RUN pip3 install --upgrade pip
RUN pip3 install --user wheel setuptools pyserial empy toml numpy jinja2 pyyaml pyros-genmsg packaging

# Firmware April 6 2020 commit is used
RUN mkdir -p /root/src
WORKDIR /root/src
RUN git clone https://github.com/PX4/Firmware.git --recursive
WORKDIR /root/src/Firmware
RUN git reset --hard 0860a61fa666b18b64ac453e744dee81af5834f0
RUN git submodule sync --recursive && git submodule update --init --recursive


RUN /bin/bash -c 'export LC_ALL=en_US.UTF-8; export LC_ALL=en_US.UTF-8; export LC_ALL=en_US.UTF-8; \
    make -j$(nproc) px4_sitl_default'

RUN ln -s /root/src/Firmware /root/catkin_ws/src/
RUN ln -s /root/src/Firmware/Tools/sitl_gazebo /root/catkin_ws/src/

RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=:/root/src/Firmware/Tools/sitl_gazebo/build" >> ~/.profile && \
    echo "export GAZEBO_MODEL_PATH=:/root/src/Firmware/Tools/sitl_gazebo/models" >> ~/.profile && \
    echo "export GAZEBO_RESOURCE_PATH=:/root/src/Firmware/Tools/sitl_gazebo/media" >> ~/.profile && \
    echo "export PYTHONPATH=/root/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/jade/lib/python2.7/dist-packages" >> ~/.profile


RUN echo "installing ROS image packages"
WORKDIR /root

# PX4 catkin build uses python2 and we need to do the same setup in python 2
RUN pip2.7 install --upgrade pip
RUN pip2.7 install --user wheel setuptools pyserial empy toml numpy jinja2 pyyaml pyros-genmsg packaging

ENV PYTHONPATH /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages:/usr/local/lib/python2.7/dist-packages

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash && catkin build'
ADD openuav_sample/install_geographiclib_datasets.sh /home/
RUN cd /home && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh
RUN sed -i '/force_color_prompt/s/^#//g' ~/.bashrc


ADD openuav_sample/setup.sh /home/
RUN chmod +x /home/setup.sh
RUN cp ~/.profile /home/
COPY samples/formation /simulation/formation
COPY samples/leader-follower /simulation/leader-follower
COPY samples/testSimulation /simulation/testSimulation
RUN groupadd -r user && useradd --create-home --gid user user && echo 'user ALL=NOPASSWD: ALL' > /etc/sudoers.d/user

# Sublime Text Editor
RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add - && \
	echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list && \
        apt-get update && \
	apt-get install sublime-text -y

# QGroundControl dependencies
RUN apt-get update -y && apt-get install -y bison build-essential flex gperf ibgstreamer-plugins-base0.10-dev libasound2-dev libatkmm-1.6-dev libbz2-dev libcap-dev libcups2-dev libdrm-dev libegl1-mesa-dev libfontconfig1-dev libfreetype6-dev libgcrypt11-dev libglu1-mesa-dev libgstreamer0.10-dev libicu-dev libnss3-dev libpci-dev libpulse-dev libssl-dev libudev-dev libx11-dev libx11-xcb-dev libxcb-composite0 libxcb-composite0-dev libxcb-cursor-dev libxcb-cursor0 libxcb-damage0 libxcb-damage0-dev libxcb-dpms0 libxcb-dpms0-dev libxcb-dri2-0 libxcb-dri2-0-dev libxcb-dri3-0 libxcb-dri3-dev libxcb-ewmh-dev libxcb-ewmh2 libxcb-glx0 libxcb-glx0-dev libxcb-icccm4 libxcb-icccm4-dev libxcb-image0 libxcb-image0-dev libxcb-keysyms1 libxcb-keysyms1-dev libxcb-present-dev libxcb-present0 libxcb-randr0 libxcb-randr0-dev libxcb-record0 libxcb-record0-dev libxcb-render-util0 libxcb-render-util0-dev libxcb-render0 libxcb-render0-dev libxcb-res0 libxcb-res0-dev libxcb-screensaver0 libxcb-screensaver0-dev libxcb-shape0 libxcb-shape0-dev libxcb-shm0 libxcb-shm0-dev libxcb-sync-dev libxcb-sync0-dev libxcb-sync1 libxcb-util-dev libxcb-util0-dev libxcb-util1 libxcb-xevie0 libxcb-xevie0-dev libxcb-xf86dri0 libxcb-xf86dri0-dev libxcb-xfixes0 libxcb-xfixes0-dev libxcb-xinerama0 libxcb-xinerama0-dev libxcb-xkb-dev libxcb-xkb1 libxcb-xprint0 libxcb-xprint0-dev libxcb-xtest0 libxcb-xtest0-dev libxcb-xv0 libxcb-xv0-dev libxcb-xvmc0 libxcb-xvmc0-dev libxcb1 libxcb1-dev libxcomposite-dev libxcursor-dev libxdamage-dev libxext-dev libxfixes-dev libxi-dev libxrandr-dev libxrender-dev libxslt-dev libxss-dev libxtst-dev perl python ruby
RUN wget https://download.openuas.us/Qt-5.11.3.tar -O /usr/local/Qt-5.11.3.tar
WORKDIR /usr/local
RUN tar xvf Qt-5.11.3.tar

# Removed source build of Qt-5.11.3 after build errors in CUDA image.

#WORKDIR /home/user/qt-everywhere-src-5.11.3
#RUN ./configure -opensource -confirm-license -release -static -nomake tests -nomake examples -no-compile-examples
#RUN make -j $(($(grep -c ^processor /proc/cpuinfo 2>/dev/null || 1)+2))
#RUN sudo make install

#Install chrome
RUN wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb && \
    apt install -y ./google-chrome-stable_current_amd64.deb --no-install-recommends && \
    rm -f ./google-chrome-stable_current_amd64.deb

RUN rm -rf /opt/qt-every*
WORKDIR /home/user
COPY scripts/qgroundcontrol /home/user/qgroundcontrol

USER root
WORKDIR /root
RUN echo  'xhost +SI:localuser:user > /dev/null' >> ~/.bashrc 
RUN usermod -a -G dialout user
COPY start_desktop.sh /usr/local/bin/start_desktop.sh
RUN echo "LD_LIBRARY_PATH=/home/user/qgroundcontrol/usr/lib/x86_64-linux-gnu:/home/user/qgroundcontrol/Qt/libs:/root/catkin_ws/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/lib/x86_64-linux-gnu/gazebo-7/plugins" >> /etc/profile
RUN echo "QML2_IMPORT_PATH=/home/user/qgroundcontrol/Qt/qml" >> /etc/profile
RUN echo "QT_PLUGIN_PATH=/home/user/qgroundcontrol/Qt/plugins" >> /etc/profile
RUN mkdir -p /home/user/.icons 
RUN cp /home/user/qgroundcontrol/qgroundcontrol.png /home/user/.icons
RUN cp /home/user/qgroundcontrol/qgroundcontrol.desktop /root/Desktop

RUN wget -O /home/user/.icons/sublime.png https://upload.wikimedia.org/wikipedia/en/d/d2/Sublime_Text_3_logo.png
RUN wget -O /root/Desktop/sublime.desktop https://download.openuas.us/sublime.desktop
CMD /usr/local/bin/start_desktop.sh

