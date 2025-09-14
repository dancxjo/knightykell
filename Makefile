.PHONY: image ubuntu-images test provision deprovision reconcile ros2-dev-image ros2-dev-run compose-up compose-down compose-shell compose-up-audio compose-up-pulse compose-shell-pulse compose-provision

# Prefer Docker Compose v2 plugin; fall back to docker-compose v1 if available
DOCKER_COMPOSE := $(shell if docker compose version >/dev/null 2>&1; then echo "docker compose"; elif docker-compose version >/dev/null 2>&1; then echo "docker-compose"; else echo ""; fi)
define REQUIRE_COMPOSE
    @if [ -z "$(DOCKER_COMPOSE)" ]; then \
        echo "Error: Docker Compose not found. Install the Docker Compose plugin (docker compose) or docker-compose."; \
        exit 1; \
    fi
endef

# Build Raspberry Pi images for hosts defined in hosts.toml.
# Usage: make image HOSTS="brainstem forebrain" (default builds all)
HOSTS ?=

image:
	@python scripts/build_images.py $(HOSTS)

ubuntu-images:
	@python scripts/build_ubuntu_images.py $(HOSTS)

test:
	@pytest

# Provision the current host (requires sudo). Uses the current repo as source.
provision:
	@sudo -E env PSYCHE_SRC="$(shell pwd)" python3 scripts/setup_host.py

# Remove installed services, env, and runtime files from the host.
deprovision:
	@sudo -E python3 scripts/deprovision.py

# Disable any previously installed psyche-* services that are not
# configured for this host according to hosts.toml
reconcile:
	@sudo -E python3 scripts/reconcile_services.py

# Build a local ROS 2 dev container (Ubuntu 24.04 + ROS ${ROS_DISTRO:-jazzy})
ros2-dev-image:
	@docker build -f Dockerfile.ros2-dev -t psyche/ros2-dev:jazzy .

# Run the dev container with host networking so ROS 2 discovery works
# Mount the repo into /opt/psyche and expose it via PSYCHE_SRC
ros2-dev-run:
	@docker run --rm -it \
	  --network host \
	  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
	  -v $(shell pwd):/opt/psyche \
	  -e PSYCHE_SRC=/opt/psyche \
	  -w /opt/psyche \
	  psyche/ros2-dev:jazzy bash

# Docker Compose helpers
compose-up:
	$(REQUIRE_COMPOSE)
	@$(DOCKER_COMPOSE) up -d --build ros2

compose-down:
	$(REQUIRE_COMPOSE)
	@$(DOCKER_COMPOSE) down

compose-shell:
	$(REQUIRE_COMPOSE)
	@$(DOCKER_COMPOSE) exec ros2 bash

# Start audio-enabled service (ALSA passthrough)
compose-up-audio:
	$(REQUIRE_COMPOSE)
	@$(DOCKER_COMPOSE) --profile audio up -d --build ros2-audio

# One-shot provisioning inside the container (no systemd)
compose-provision:
	$(REQUIRE_COMPOSE)
	@$(DOCKER_COMPOSE) --profile provision up --build --abort-on-container-exit ros2-provision

# Start PulseAudio client container (needs host Pulse socket)
compose-up-pulse:
	$(REQUIRE_COMPOSE)
	@$(DOCKER_COMPOSE) --profile pulse up -d --build ros2-pulse

compose-shell-pulse:
	$(REQUIRE_COMPOSE)
	@$(DOCKER_COMPOSE) exec ros2-pulse bash
