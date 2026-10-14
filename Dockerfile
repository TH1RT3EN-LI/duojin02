ARG ROS_BASE_IMAGE=ros:humble-ros-base-jammy
ARG DEPENDENCIES_IMAGE=dependencies
FROM ${ROS_BASE_IMAGE} AS package_manifests
COPY src /source
RUN python3 -c 'from pathlib import Path; import shutil; source = Path("/source"); output = Path("/package-manifests"); \
    [(destination.parent.mkdir(parents=True, exist_ok=True), shutil.copyfile(path, destination)) \
     for path in source.rglob("package.xml") for destination in [output / path.relative_to(source)]]'

FROM ${ROS_BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN printf 'APT::Install-Recommends "false";\nAPT::Install-Suggests "false";\n' \
      > /etc/apt/apt.conf.d/99-hardware-dependencies && \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs pkg-config curl ca-certificates \
    python3-pip python3-colcon-common-extensions python3-rosdep python3-pytest \
    python3-yaml \
    libboost-thread-dev \
    ros-humble-asio-cmake-module \
    && rm -rf /var/lib/apt/lists/*

COPY --from=package_manifests /package-manifests /tmp/hardware-dependencies/src
RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi && \
    rosdep update --rosdistro humble && \
    apt-get update && \
    rosdep install --from-paths /tmp/hardware-dependencies/src --ignore-src \
      --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/* /tmp/hardware-dependencies

FROM ${DEPENDENCIES_IMAGE} AS hardware_dependencies
SHELL ["/bin/bash", "-c"]
COPY scripts/fix_humble_cmake.py /opt/duojin02/tools/fix_humble_cmake.py
RUN python3 /opt/duojin02/tools/fix_humble_cmake.py
COPY requirements-build.txt /tmp/requirements-build.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements-build.txt

FROM hardware_dependencies AS slam_source
COPY scripts/prepare_slam_backend.py /source/scripts/prepare_slam_backend.py
COPY patches/slam_toolbox /source/patches/slam_toolbox
RUN python3 /source/scripts/prepare_slam_backend.py --root /source --destination /source/backend/slam_toolbox

FROM hardware_dependencies AS cuda_toolkit
ARG DUOJIN_ENABLE_CUDA=ON
COPY docker/cuda-toolkit.json /source/docker/cuda-toolkit.json
COPY scripts/prepare_cuda_toolkit.py /source/scripts/prepare_cuda_toolkit.py
RUN if [[ "${DUOJIN_ENABLE_CUDA}" != "OFF" ]]; then \
      python3 /source/scripts/prepare_cuda_toolkit.py --root /source \
        --destination /opt/cuda --cache-directory /tmp/cuda-downloads && \
      rm -rf /tmp/cuda-downloads; \
    else mkdir -p /opt/cuda; \
    fi

FROM hardware_dependencies AS slam_builder
ARG BUILD_JOBS=2
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}
WORKDIR /opt/duojin02
COPY --from=slam_source /source/backend/slam_toolbox /source/slam_toolbox
RUN source /opt/ros/humble/setup.bash && \
    MAKEFLAGS="-j${BUILD_JOBS}" colcon build --base-paths /source/slam_toolbox \
      --build-base build/slam_backend --install-base slam_backend --merge-install \
      --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    cp /source/slam_toolbox/hardware_build.json slam_backend/share/slam_toolbox/

FROM hardware_dependencies AS builder
ARG BUILD_WORKERS=1
ARG BUILD_JOBS=2
ARG DUOJIN_ENABLE_CUDA=ON
ARG CUDA_ARCHITECTURES=87;120
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}
ENV PATH=/opt/cuda/bin:${PATH}
ENV CUDAToolkit_ROOT=/opt/cuda
WORKDIR /opt/duojin02
COPY --from=slam_builder /opt/duojin02/slam_backend /opt/duojin02/slam_backend
COPY --from=cuda_toolkit /opt/cuda /opt/cuda
COPY src ./src
RUN source /opt/ros/humble/setup.bash && \
    source /opt/duojin02/slam_backend/setup.bash && \
    MAKEFLAGS="-j${BUILD_JOBS}" colcon build --merge-install --parallel-workers ${BUILD_WORKERS} \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DDUOJIN_ENABLE_CUDA=${DUOJIN_ENABLE_CUDA} \
        "-DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}"

FROM hardware_dependencies AS hardware_runtime
COPY --from=slam_builder /opt/duojin02/slam_backend /opt/duojin02/slam_backend
COPY --from=builder /opt/duojin02/install /opt/duojin02/install
COPY docker/entrypoint.sh /usr/local/bin/duojin02-entrypoint
RUN chmod +x /usr/local/bin/duojin02-entrypoint && mkdir -p /ws/maps && \
    printf '%s\n' \
      'source /opt/ros/humble/setup.bash' \
      'source /opt/duojin02/slam_backend/setup.bash' \
      'source /opt/duojin02/install/setup.bash' \
      'for overlay in slam_backend hardware; do' \
      '  if [[ -r /ws/install/$overlay/setup.bash ]]; then' \
      '    if [[ "$(cat /ws/install/$overlay/.target_architecture 2>/dev/null)" == "$(uname -m)" ]]; then' \
      '      source /ws/install/$overlay/setup.bash' \
      '    else echo "Skipping incompatible $overlay overlay; rebuild with ./scripts/build_hardware.sh." >&2; fi' \
      '  fi' \
      'done' \
      >> /root/.bashrc
ENV DUOJIN02_WORKSPACE_ROOT=/ws
ENV ROS_DOMAIN_ID=20
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/duojin02-entrypoint"]
CMD ["bash"]

FROM hardware_runtime AS testing
COPY --from=slam_source /source/backend/slam_toolbox /opt/duojin02/test_sources/slam_toolbox
COPY --from=cuda_toolkit /opt/cuda /opt/cuda
ENV PATH=/opt/cuda/bin:${PATH}
ENV CUDAToolkit_ROOT=/opt/cuda

FROM hardware_runtime AS runtime
