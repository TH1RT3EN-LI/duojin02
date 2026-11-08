ARG ROS_BASE_IMAGE=ros:humble-ros-base-jammy
ARG DEPENDENCIES_IMAGE=dependencies
FROM ${ROS_BASE_IMAGE} AS package_manifests
COPY src /source
RUN python3 -c 'from pathlib import Path; import shutil; source = Path("/source"); output = Path("/package-manifests"); \
    [(destination.parent.mkdir(parents=True, exist_ok=True), shutil.copyfile(path, destination)) \
     for path in source.rglob("package.xml") for destination in [output / path.relative_to(source)]]'

FROM ${ROS_BASE_IMAGE} AS dependencies

FROM ${DEPENDENCIES_IMAGE} AS hardware_dependencies
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
COPY --from=package_manifests /package-manifests /tmp/duojin02-dependencies/src
COPY scripts/install_dependencies.sh scripts/fix_humble_cmake.py /tmp/duojin02-dependencies/scripts/
COPY requirements-build.txt /tmp/duojin02-dependencies/requirements-build.txt
RUN printf 'APT::Install-Recommends "false";\nAPT::Install-Suggests "false";\n' \
      > /etc/apt/apt.conf.d/99-hardware-dependencies && \
    /tmp/duojin02-dependencies/scripts/install_dependencies.sh && \
    rm -rf /var/lib/apt/lists/* /tmp/duojin02-dependencies

FROM hardware_dependencies AS slam_source
COPY scripts/prepare_slam_backend.py /source/scripts/prepare_slam_backend.py
COPY patches /source/patches
RUN /usr/bin/python3 /source/scripts/prepare_slam_backend.py --root /source --destination /source/backend/slam_toolbox && \
    /usr/bin/python3 /source/scripts/prepare_slam_backend.py --root /source --dependency nav2_map_server --destination /source/backend/nav2_map_server

FROM hardware_dependencies AS cuda_toolkit
ARG DUOJIN_ENABLE_CUDA=ON
COPY docker/cuda-toolkit.json /source/docker/cuda-toolkit.json
COPY scripts/prepare_cuda_toolkit.py /source/scripts/prepare_cuda_toolkit.py
RUN if [[ "${DUOJIN_ENABLE_CUDA}" != "OFF" ]]; then \
      /usr/bin/python3 /source/scripts/prepare_cuda_toolkit.py --root /source \
        --destination /opt/cuda --cache-directory /tmp/cuda-downloads && \
      rm -rf /tmp/cuda-downloads; \
    else mkdir -p /opt/cuda; \
    fi

FROM hardware_dependencies AS slam_builder
ARG BUILD_JOBS=2
ARG BUILD_WORKERS=1
WORKDIR /opt/duojin02
COPY scripts/environment.sh scripts/build_slam_backend.sh scripts/build_hardware.sh scripts/prepare_slam_backend.py scripts/cuda_install_provenance.py ./scripts/
COPY patches ./patches
COPY --from=slam_source /source/backend ./.cache/slam_backend/src
COPY src/duojin02_scan_geometry ./src/duojin02_scan_geometry
RUN ./scripts/build_slam_backend.sh

FROM slam_builder AS builder
ARG DUOJIN_ENABLE_CUDA=ON
ARG CUDA_ARCHITECTURES=120
ENV PATH=/opt/cuda/bin:${PATH}
ENV CUDAToolkit_ROOT=/opt/cuda
COPY --from=cuda_toolkit /opt/cuda /opt/cuda
COPY src ./src
RUN ./scripts/build_hardware.sh -DDUOJIN_ENABLE_CUDA=${DUOJIN_ENABLE_CUDA} \
      "-DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}"

FROM hardware_dependencies AS local_environment
ARG CUDA_ARCHITECTURES=120
COPY --from=builder /opt/duojin02/install /opt/duojin02/install
COPY --from=slam_source /source/backend/slam_toolbox /opt/duojin02/test_sources/slam_toolbox
COPY --from=cuda_toolkit /opt/cuda /opt/cuda
COPY source.sh /opt/duojin02/source.sh
COPY scripts/environment.sh /opt/duojin02/scripts/environment.sh
COPY docker/entrypoint.sh /usr/local/bin/duojin02-entrypoint
RUN chmod +x /usr/local/bin/duojin02-entrypoint && mkdir -p /ws/maps
ENV DUOJIN02_WORKSPACE_ROOT=/ws
ENV PATH=/opt/cuda/bin:${PATH}
ENV CUDAToolkit_ROOT=/opt/cuda
ENV CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/duojin02-entrypoint"]
CMD ["bash"]
