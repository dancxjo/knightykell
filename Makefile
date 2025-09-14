.PHONY: image ubuntu-images test provision deprovision reconcile

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
