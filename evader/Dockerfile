FROM ros:humble-ros-core

COPY ./ /home/evader/evader
WORKDIR /home/evader/evader

RUN apt-get update
RUN apt-get install -y python3-rosdep ros-dev-tools

RUN rosdep init
RUN rosdep update 
RUN rosdep install --from-paths . --ignore-src -r -y 

RUN useradd -ms /bin/bash evader

RUN chown -R evader:evader /home/evader/
RUN chmod +x /home/evader/evader/entrypoint.sh

USER evader
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && colcon build --cmake-args -DDOCKER_COMPILE=ON"

RUN rm -rf /home/evader/evader/src
RUN rm -rf /home/evader/evader/include
RUN rm CMakeLists.txt
RUN rm package.xml
RUN rm Dockerfile

ENTRYPOINT [ "/home/evader/evader/entrypoint.sh" ]
