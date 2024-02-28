ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Run apt-get (dkpg) without interactive dialogue
ARG DEBIAN_FRONTEND=noninteractive
########################################
# SECTION 1: Essentials                #
########################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl software-properties-common \
    # Setup locale language config \
    locales \
  && apt-get upgrade -y \
  && rm -rf /var/lib/apt/lists/* \
  # Setup locale language config \
  && locale-gen "en_US.UTF-8" \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  # Set timezone \
  && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8


########################################
# SECTION 2: Install ROS humble        #
########################################
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

RUN add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] "\
    "http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-desktop ros-dev-tools \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init && rosdep update \
  && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc


########################################
# SECTION 3: Install Python/pip        #
########################################
# Default python with up-to-date pip
RUN apt-get update && apt-get install -y \
    python3 python3-pip \
  && rm -rf /var/lib/apt/lists/* \
  && python3 -m pip install --upgrade pip


################################################
# SECTION 4: Install mycobot_ros2 dependencies #
################################################
# Install python dependencies
ARG PYMYCOBOT_VERSION
RUN pip3 install "pymycobot $PYMYCOBOT_VERSION" --user

ARG WORKDIR="/workspace"
# Copy robotiq ROS package
WORKDIR ${WORKDIR}
COPY . src/mycobot_ros2

RUN apt-get update \
  && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y \
  && rm -rf /var/lib/apt/lists/*


########################################
# SECTION N: Additional config & MISC  #
########################################
# Non-interactive shell
ENV BASH_ENV="/root/.noninteractive_bashrc"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASH_ENV

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["/bin/bash"]
