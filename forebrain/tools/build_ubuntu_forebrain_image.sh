#!/bin/bash
set -e

# Configuration
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORK_DIR="${SCRIPT_DIR}/work"
IMG_FILE="${WORK_DIR}/ubuntu_forebrain.img"
IMG_SIZE="16G"
MNT_ROOT="${WORK_DIR}/mnt_root"
MNT_EFI="${WORK_DIR}/mnt_efi"
DISTRO="noble"
UBUNTU_BASE_URL="http://cdimage.ubuntu.com/ubuntu-base/releases/${DISTRO}/release/ubuntu-base-24.04-base-amd64.tar.gz"
UBUNTU_BASE_FILE="${WORK_DIR}/ubuntu-base.tar.gz"

# Ensure work directory exists
mkdir -p "${WORK_DIR}"
mkdir -p "${MNT_ROOT}"
mkdir -p "${MNT_EFI}"

# Create image file
if [ ! -f "$IMG_FILE" ]; then
    echo "Creating image file..."
    dd if=/dev/zero of="$IMG_FILE" bs=1M count=0 seek=16384
fi

# Partition the image
if ! losetup -j "$IMG_FILE" | grep -q "$IMG_FILE"; then
    echo "Partitioning image..."
    parted -s "$IMG_FILE" \
        mklabel gpt \
        mkpart ESP fat32 1MiB 513MiB \
        set 1 esp on \
        mkpart primary ext4 513MiB 100%
fi

# Setup loop device
LOOP_DEV=$(sudo losetup -fP --show "$IMG_FILE")
echo "Image mounted on ${LOOP_DEV}"

# Format partitions
echo "Formatting partitions..."
sudo mkfs.vfat -F32 "${LOOP_DEV}p1"
sudo mkfs.ext4 -F "${LOOP_DEV}p2"

# Mount partitions
echo "Mounting partitions..."
sudo mount "${LOOP_DEV}p2" "${MNT_ROOT}"
sudo mkdir -p "${MNT_ROOT}/boot/efi"
sudo mount "${LOOP_DEV}p1" "${MNT_ROOT}/boot/efi"

# Download Ubuntu base
if [ ! -f "$UBUNTU_BASE_FILE" ]; then
    echo "Downloading Ubuntu base..."
    curl -L "$UBUNTU_BASE_URL" -o "$UBUNTU_BASE_FILE"
fi

# Extract Ubuntu base
if [ ! -f "${MNT_ROOT}/etc/os-release" ]; then
    echo "Extracting Ubuntu base..."
    sudo tar -xpf "$UBUNTU_BASE_FILE" -C "${MNT_ROOT}"
fi

# Copy docker project to a temporary location accessible from chroot
echo "Copying docker project..."
sudo cp -r "${SCRIPT_DIR}/../docker" "${MNT_ROOT}/tmp/"

# Prepare chroot environment
echo "Preparing chroot environment..."
sudo cp /etc/resolv.conf "${MNT_ROOT}/etc/"
sudo mount --bind /dev "${MNT_ROOT}/dev"
sudo mount --bind /dev/pts "${MNT_ROOT}/dev/pts"
sudo mount --bind /proc "${MNT_ROOT}/proc"
sudo mount --bind /sys "${MNT_ROOT}/sys"

# Chroot and configure
echo "Configuring system inside chroot..."
sudo chroot "${MNT_ROOT}" /bin/bash << "EOF"
set -e

export DEBIAN_FRONTEND=noninteractive

# Set up basic environment
echo "forebrain" > /etc/hostname
echo "127.0.0.1 localhost" > /etc/hosts
echo "127.0.1.1 forebrain" >> /etc/hosts

# Configure apt
cat << 'EOT' > /etc/apt/sources.list
deb http://archive.ubuntu.com/ubuntu/ noble main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu/ noble-updates main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu/ noble-security main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu/ noble-backports main restricted universe multiverse
EOT

apt-get update

# Install essential packages
apt-get install -y --no-install-recommends \
    linux-image-generic \
    grub-efi-amd64-signed \
    shim-signed \
    systemd-sysv \
    openssh-server \
    avahi-daemon \
    network-manager \
    sudo \
    curl \
    htop \
    vim \
    git \
    python3 \
    python3-pip \
    docker.io

# Set root password (replace with a secure method)
echo "root:root" | chpasswd

# Create a user
useradd -m -s /bin/bash -G sudo,docker user
echo "user:user" | chpasswd

# Install GRUB
grub-install --target=x86_64-efi --efi-directory=/boot/efi --bootloader-id=ubuntu --recheck --no-floppy
update-grub

# Enable services
systemctl enable ssh
systemctl enable avahi-daemon
systemctl enable NetworkManager
systemctl enable docker

# Copy docker-compose project
mkdir -p /opt/knightykell/forebrain
cp -r /tmp/docker /opt/knightykell/forebrain/

# Create systemd service for docker-compose
cat << 'EOT' > /etc/systemd/system/knightykell-forebrain.service
[Unit]
Description=Knightykell Forebrain Docker Compose
Requires=docker.service
After=docker.service

[Service]
Type=oneshot
RemainAfterExit=true
WorkingDirectory=/opt/knightykell/forebrain/docker
ExecStart=/usr/bin/docker compose up -d
ExecStop=/usr/bin/docker compose down

[Install]
WantedBy=multi-user.target
EOT

systemctl enable knightykell-forebrain.service

# Cleanup
apt-get clean
rm -rf /var/lib/apt/lists/*

EOF

# Unmount everything
echo "Unmounting partitions..."
sudo umount -l "${MNT_ROOT}/dev/pts"
sudo umount -l "${MNT_ROOT}/dev"
sudo umount -l "${MNT_ROOT}/proc"
sudo umount -l "${MNT_ROOT}/sys"
sudo umount "${MNT_ROOT}/boot/efi"
sudo umount "${MNT_ROOT}"
sudo losetup -d "${LOOP_DEV}"

echo "Build complete: ${IMG_FILE}"
