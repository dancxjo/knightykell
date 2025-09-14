.PHONY: image ubuntu-images test provision deprovision reconcile ros2-dev-image ros2-dev-run

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
