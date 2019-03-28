FROM debian:buster-slim

RUN apt-get update
RUN apt-get install -y tcsh curl git-core ninja-build cmake
RUN apt-get install -y clang-7
RUN apt-get install -y clang-format-7
RUN apt-get install -y libboost-test-dev
RUN echo "export PATH=/usr/lib/llvm-7.0/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" >> /root/.bashrc


