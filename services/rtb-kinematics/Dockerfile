# Real-robot kinematics service (Robotics Toolbox) — a capability behind the
# Kinematics/Planner seam, shipped as its own container so it scales and
# deploys independently of the RT core.
#
#   docker build -t robonode/rtb-kinematics services/rtb-kinematics
FROM python:3.12-slim

WORKDIR /svc
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY service.py test_service.py ./

EXPOSE 8091
HEALTHCHECK --interval=10s --timeout=3s --start-period=20s --retries=6 \
  CMD python3 -c "import urllib.request,sys; sys.exit(0 if urllib.request.urlopen('http://localhost:8091/health').status==200 else 1)"

CMD ["python3", "service.py", "--port", "8091"]
