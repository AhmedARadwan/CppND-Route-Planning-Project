FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
RUN apt install -y build-essential \
                   cmake \
                   libcairo2-dev \
                   libgraphicsmagick1-dev \
                   libpng-dev

RUN apt install -y git \
    && cd /root/ \
    && git clone https://github.com/svgpp/svgpp.git \
    && cd svgpp/ && git checkout v1.3.0

RUN cd /root && git clone --recurse-submodules https://github.com/cpp-io2d/P0267_RefImpl \
    && cd P0267_RefImpl \
    && sed -n '3i include_directories(/root/svgpp/include/)' CMakeLists.txt \
    && mkdir Debug && cd Debug \
    && cmake --config Debug "-DCMAKE_BUILD_TYPE=Debug" .. \
    && cmake --build . && make install

WORKDIR /app
