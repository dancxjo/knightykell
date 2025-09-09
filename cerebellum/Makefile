SUDO ?= sudo
PREFIX ?= /opt/cerebellum
SYSTEMD_DIR ?= /etc/systemd/system

OLED_SRC := host/oled/oled_statusd.py
OLED_DST_DIR := $(PREFIX)/oled
OLED_DST := $(OLED_DST_DIR)/oled_statusd.py

SERVICE_SRC := firstboot/oled-statusd.service
SERVICE_DST := $(SYSTEMD_DIR)/oled-statusd.service

.PHONY: install-oled install-oled-service enable-oled restart-oled uninstall-oled

install-oled:
	@echo "Installing OLED daemon to $(OLED_DST)"
	$(SUDO) mkdir -p "$(OLED_DST_DIR)"
	$(SUDO) install -m 0755 "$(OLED_SRC)" "$(OLED_DST)"
	@echo "Done. Try: $(SUDO) python3 $(OLED_DST)"

install-oled-service: install-oled
	@echo "Installing systemd unit to $(SERVICE_DST)"
	$(SUDO) install -m 0644 "$(SERVICE_SRC)" "$(SERVICE_DST)"
	$(SUDO) systemctl daemon-reload
	@echo "Done. Enable with: make enable-oled"

enable-oled:
	$(SUDO) systemctl enable --now oled-statusd.service
	@echo "Service enabled and started. Check: systemctl status oled-statusd.service"

restart-oled:
	$(SUDO) systemctl restart oled-statusd.service
	@echo "Service restarted."

uninstall-oled:
	-$(SUDO) systemctl disable --now oled-statusd.service 2>/dev/null || true
	-$(SUDO) rm -f "$(SERVICE_DST)"
	-$(SUDO) systemctl daemon-reload
	-$(SUDO) rm -f "$(OLED_DST)"
	@echo "Uninstalled OLED daemon and unit (if present)."

