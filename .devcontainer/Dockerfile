FROM amd64/ros:humble-ros-base

# ** [Optional] Uncomment this section to install additional packages. **
#
ARG WORKSPACE
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common libusb-1.0-0-dev wget zsh
   #
RUN add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt install -y gcc-10 gcc-10-base gcc-10-doc g++-10 && \
    apt install -y libstdc++-10-dev libstdc++-10-doc && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
ENV DEBIAN_FRONTEND=dialog
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true
# Set up auto-source of workspace for ros user
ENV HOME /home/ros
RUN git clone https://github.com/ohmyzsh/ohmyzsh.git $HOME/.oh-my-zsh
ADD .zshrc $HOME
RUN echo "if [ -f ${WORKSPACE}/install/setup.zsh ]; then source ${WORKSPACE}/install/setup.zsh; fi" >> $HOME/.zshrc
