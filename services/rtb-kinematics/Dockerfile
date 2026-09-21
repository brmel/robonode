# Real-robot kinematics service (Robotics Toolbox) — a capability behind the
# Kinematics/Planner seam, shipped as its own container so it scales and
# deploys independently of the RT core.
#
#   docker build -t robonode/rtb-kinematics services/rtb-kinematics

# ---- build stage ---------------------------------------------------------
# Robotics Toolbox pulls swift-sim, which ships no wheel for arm64 and compiles
# a C++ extension. Give the compiler to a stage we throw away rather than to the
# image that runs in production.
FROM python:3.12-slim AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /svc
COPY requirements.txt .
RUN pip install --no-cache-dir --prefix=/install -r requirements.txt

# ---- runtime stage -------------------------------------------------------
FROM python:3.12-slim
COPY --from=build /install /usr/local
WORKDIR /svc
COPY service.py test_service.py ./

EXPOSE 8091
HEALTHCHECK --interval=10s --timeout=3s --start-period=20s --retries=6 \
  CMD python3 -c "import urllib.request,sys; sys.exit(0 if urllib.request.urlopen('http://localhost:8091/health').status==200 else 1)"

CMD ["python3", "service.py", "--port", "8091"]
