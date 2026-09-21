# The Python side of the platform, self-contained: a contributor working here
# never builds C++ and never touches the physics.
VENV ?= .venv
PY   := $(VENV)/bin/python

$(VENV):
	python3 -m venv $(VENV)
	$(PY) -m pip install --quiet --upgrade pip
	$(PY) -m pip install --quiet -r dev-requirements.txt

.PHONY: setup run test lint clean
setup: $(VENV)                      ## create the venv and install pinned deps
run: $(VENV)                        ## serve on :8091
	$(PY) service.py --port 8091
test: $(VENV)                       ## FK/IK/moveL self-checks against real models
	$(PY) test_service.py
lint: $(VENV)
	$(VENV)/bin/ruff check .
clean:
	rm -rf $(VENV) __pycache__ .pytest_cache
