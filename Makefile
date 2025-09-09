# Default goal for Makefile Tools and CLI
.DEFAULT_GOAL := all

SUDO ?= sudo
DOCKER_BUILDKIT ?= 1
BUILDKIT_PROGRESS ?= plain
export DOCKER_BUILDKIT
export BUILDKIT_PROGRESS

# --- Cerebellum image build ---
.PHONY: cerebellum-img cerebellum-cache-clean

IMG_OUT ?= cerebellum/out/rpios-rpi4-cerebellum.img

cerebellum-img:
	@echo "Building Cerebellum (RPi OS Lite, arm64) image into $(IMG_OUT)"
	cd cerebellum/tools && ./build_rpi4_rpios_image.sh
	@echo "Image built: $(IMG_OUT)"

cerebellum-cache-clean:
	rm -f cerebellum/out/cache/rpios_latest.archive || true
	@echo "cerebellum image cache cleared (cerebellum/out/cache)"

## --- Compose helpers ---
COMPOSE_HELPER := ./scripts/compose_cmd.sh

# Build compose commands for monitor and cerebellum
ifeq (,$(wildcard $(COMPOSE_HELPER)))
  MONITOR_COMPOSE ?= $(shell \
    (command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1 && echo "docker compose -f monitor/docker/compose.yml") || \
    (command -v docker-compose >/dev/null 2>&1 && echo "docker-compose -f monitor/docker/compose.yml") || \
    (command -v podman >/dev/null 2>&1 && echo "podman compose -f monitor/docker/compose.yml") )
  CEREBELLUM_COMPOSE ?= $(shell \
    (command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1 && echo "docker compose -f cerebellum/docker/compose.yml") || \
    (command -v docker-compose >/dev/null 2>&1 && echo "docker-compose -f cerebellum/docker/compose.yml") || \
    (command -v podman >/dev/null 2>&1 && echo "podman compose -f cerebellum/docker/compose.yml") )
else
  MONITOR_COMPOSE ?= $(shell $(COMPOSE_HELPER) -f monitor/docker/compose.yml)
  CEREBELLUM_COMPOSE ?= $(shell $(COMPOSE_HELPER) -f cerebellum/docker/compose.yml)
endif

compose_guard_monitor = @if [ -z "$(MONITOR_COMPOSE)" ]; then \
  echo "ERROR: No compose tool found for monitor. Install Docker (compose) or Podman." >&2; exit 127; fi
compose_guard_cerebellum = @if [ -z "$(CEREBELLUM_COMPOSE)" ]; then \
  echo "ERROR: No compose tool found for cerebellum. Install Docker (compose) or Podman." >&2; exit 127; fi

# --- Monitor (laptop dev) helpers ---
.PHONY: monitor-build monitor-up monitor-down monitor-shell monitor-rviz monitor-rqt

monitor-build:
	$(compose_guard_monitor)
	$(MONITOR_COMPOSE) build

monitor-up:
	$(compose_guard_monitor)
	$(MONITOR_COMPOSE) up -d

monitor-down:
	@# Only attempt to stop if Docker engine socket is accessible
	@if [ -n "$(MONITOR_COMPOSE)" ] && [ -S /var/run/docker.sock ] && [ -r /var/run/docker.sock ]; then \
	  $(MONITOR_COMPOSE) down >/dev/null 2>&1 || true; \
	else \
	  echo "Skipping monitor-down (Docker engine not accessible)."; \
	fi

monitor-shell:
	$(compose_guard_monitor)
	$(MONITOR_COMPOSE) run --rm monitor bash

monitor-rviz:
	chmod +x monitor/run_rviz.sh
	./monitor/run_rviz.sh

monitor-rqt:
	chmod +x monitor/run_rqt.sh
	./monitor/run_rqt.sh

# --- Cerebellum build helpers ---
.PHONY: cerebellum-build

cerebellum-build:
	$(compose_guard_cerebellum)
	$(CEREBELLUM_COMPOSE) build cerebellum

# (QEMU emulation targets removed per request)

# Primary high-level targets
.PHONY: cerebellum cerebellum-burn monitor

# Build Raspberry Pi OS Lite image for cerebellum
cerebellum: cerebellum-img
	@echo "Cerebellum image ready: $(abspath $(IMG_OUT))"

# Launch Raspberry Pi Imager to burn the image
cerebellum-burn: cerebellum
	@IMG_ABS="$(abspath $(IMG_OUT))"; \
	if command -v rpi-imager >/dev/null 2>&1; then \
	  echo "Launching Raspberry Pi Imager..."; rpi-imager >/dev/null 2>&1 & \
	elif command -v flatpak >/dev/null 2>&1 && flatpak list | grep -q org.raspberrypi.rpi-imager; then \
	  echo "Launching Raspberry Pi Imager (Flatpak)..."; flatpak run org.raspberrypi.rpi-imager >/dev/null 2>&1 & \
	else \
	  echo "rpi-imager not found. Please install it (https://www.raspberrypi.com/software/)."; \
	fi; \
	if command -v xdg-open >/dev/null 2>&1; then xdg-open "$$(dirname "$$IMG_ABS")" >/dev/null 2>&1 || true; fi; \
	echo "Image: $$IMG_ABS"; echo "If not preselected, choose 'Use custom' and pick the image."

# Build and bring up the monitor stack locally
monitor: monitor-build monitor-up

# -----------------------------------------------------------------------------
# Conventional wrapper targets for VS Code Makefile Tools
# These provide standard UX: configure, build, clean, run/launch, test, etc.
# -----------------------------------------------------------------------------
.PHONY: all configure build rebuild run launch test clean distclean help

# all: default to building core images used in development
all: build

# configure: check and report selected compose tools and basic env
configure:
	@echo "Configuring project (no-op)"
	@echo "Using compose for monitor: $(if $(MONITOR_COMPOSE),$(MONITOR_COMPOSE),<none>)"
	@echo "Using compose for cerebellum: $(if $(CEREBELLUM_COMPOSE),$(CEREBELLUM_COMPOSE),<none>)"
	@echo "BuildKit: DOCKER_BUILDKIT=$(DOCKER_BUILDKIT), BUILDKIT_PROGRESS=$(BUILDKIT_PROGRESS)"
	@echo "IMG_OUT: $(IMG_OUT)"
	@echo "PREFIX: $(PREFIX)"
	@echo "SYSTEMD_DIR: $(SYSTEMD_DIR)"
	@echo "Done."

# build: build both dev images (monitor + cerebellum)
build: monitor-build
	@echo "Build complete."

# rebuild: force rebuild images without cache
rebuild:
	$(compose_guard_monitor)
	$(MONITOR_COMPOSE) build --no-cache || true
	$(compose_guard_cerebellum)
	$(CEREBELLUM_COMPOSE) build --no-cache cerebellum || true
	@echo "Rebuild complete."

# run/launch: bring up the monitor dev stack (GUI tools available separately)
run: monitor-up
launch: run

# test: placeholder for project tests
test:
	@echo "No tests defined yet. Add per-component test targets as needed."

# clean: shut down dev stack and clear lightweight caches
clean: monitor-down
	@$(MAKE) cerebellum-cache-clean
	@echo "Clean complete."

# distclean: stronger clean (also removes generated image artifact if present)
distclean: clean
	@rm -f -- "$(IMG_OUT)" 2>/dev/null || true
	@echo "Distclean complete."

# help: list common and component-specific targets
help:
	@echo "Common targets:"
	@echo "  make configure     # probe env and report settings"
	@echo "  make build         # build monitor image"
	@echo "  make monitor       # build and start monitor stack"
	@echo "  make cerebellum    # build Raspberry Pi OS Lite image"
	@echo "  make cerebellum-burn # open Raspberry Pi Imager and show image"
	@echo "  make run           # alias to start monitor stack"
	@echo "  make launch        # alias for run"
	@echo "  make test          # run tests (placeholder)"
	@echo "  make clean         # stop stack, clear caches"
	@echo "  make distclean     # clean + remove image artifact"
	@echo "\nComponent targets:"
	@echo "  make monitor-build | monitor-up | monitor-down | monitor-shell | monitor-rviz | monitor-rqt"
	@echo "  make cerebellum-build (container build, optional)"
	@echo "  make cerebellum-img | cerebellum-cache-clean"
