.PHONY: image ubuntu-images test provision

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
