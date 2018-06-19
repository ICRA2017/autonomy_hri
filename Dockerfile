FROM ros:indigo-perception

RUN apt-get update && apt-get install -y \
	python-catkin-pkg python-rosdep python-wstool \
	python-catkin-tools ros-indigo-catkin \
	build-essential \
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
	
RUN source /ros_entrypoint.sh \
	&& cd $CATKIN_WS \
	&& rosdep install --from-paths src --ignore-src -r -y

RUN source /ros_entrypoint.sh \
	&& cd $CATKIN_WS \
	&& unlink CMakeLists.txt \
	&& catkin_make

