FROM ubuntu:resolute
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

# Update apt-get
RUN apt-get update -q \
    && apt-get upgrade -y

RUN apt-get install -y dpkg-dev libeigen3-dev cmake

COPY . /root/dqrobotics/cpp
RUN cd /root/dqrobotics/cpp && dpkg-buildpackage --sanitize-env -us -uc -B -rfakeroot