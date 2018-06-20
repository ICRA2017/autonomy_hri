FROM ros:indigo-perception

RUN apt-get update && apt-get install -y \
	python-catkin-pkg python-rosdep python-wstool \
	python-catkin-tools ros-indigo-catkin \
	build-essential wget \
	&& rm -rf /var/lib/apt/lists

ENV CATKIN_WS=/root/catkin_ws

RUN rm /bin/sh \
	&& ln -s /bin/bash /bin/sh	

RUN source /ros_entrypoint.sh \
	&& mkdir -p $CATKIN_WS/src \
	&& cd $CATKIN_WS && catkin_init_workspace \
	&& cd src && git clone https://github.com/ICRA2017/autonomy_hri.git

RUN source /ros_entrypoint.sh \
	&& cd $CATKIN_WS/src \
	&& git clone https://github.com/AutonomyLab/miarn_ros.git

RUN echo -e "deb http://archive.hark.jp/harkrepos $(lsb_release -cs) non-free\ndeb-src http://archive.hark.jp/harkrepos $(lsb_release -cs) non-free" > /etc/apt/sources.list.d/hark.list \
	&& wget -q -O - http://archive.hark.jp/harkrepos/public.gpg | apt-key add -

RUN apt-get update && apt-get install -y \
	hark-ros-indigo hark-ros-stacks-indigo \
	&& rm -rf /var/lib/apt/lists
	
RUN source /ros_entrypoint.sh \
	&& source /opt/ros/indigo/stacks/setup.bash \
	&& cd $CATKIN_WS \
	&& rosdep install --from-paths src --ignore-src -r -y

#RUN source /ros_entrypoint.sh \
#	&& cd $CATKIN_WS \
#	&& unlink CMakeLists.txt \
#	&& catkin_make

