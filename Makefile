.PHONY: image test

# Build Raspberry Pi images for hosts defined in hosts.toml.
# Usage: make image HOSTS="brainstem forebrain" (default builds all)
HOSTS ?=

image:
	@python scripts/build_images.py $(HOSTS)

test:
	@pytest
