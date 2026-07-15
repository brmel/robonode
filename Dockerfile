# Reproducible build + runtime for the RoboNode live cell server.
# Multi-stage: a full toolchain builds the package (incl. MuJoCo from source);
# the runtime stage keeps only the C++ runtime and the built tree, so the
# binary's build-tree RPATH still resolves libmujoco (paths are preserved by
# copying /src to the same location).
#
#   docker build -t robonode .
#   docker run --rm -p 8080:8080 robonode   →  http://localhost:8080

# ---- build stage ---------------------------------------------------------
FROM ubuntu:24.04 AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
# MuJoCo on (the physics twin the web app renders); UR adapter off (not needed
# by cell_server and it pulls a heavier vendor tree).
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DROBONODE_BUILD_MUJOCO=ON -DROBONODE_BUILD_UR_ADAPTER=OFF \
 && cmake --build build -j --target cell_server

# ---- runtime stage -------------------------------------------------------
FROM ubuntu:24.04 AS runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
      libstdc++6 ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*
# Same path as the build stage so the binary's RPATH to libmujoco stays valid.
COPY --from=build /src /src
ENV ROBONODE_WORLDS_DIR=/src/sim-mujoco/worlds \
    ROBONODE_WEB_DIR=/src/apps/cell_server/web
EXPOSE 8080
HEALTHCHECK --interval=10s --timeout=3s --start-period=5s --retries=5 \
  CMD curl -fsS http://localhost:8080/ >/dev/null || exit 1
CMD ["/src/build/apps/cell_server/cell_server"]
