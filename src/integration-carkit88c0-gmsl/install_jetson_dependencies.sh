#!/bin/bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Use script to install necessary runtime dependencies and tools on Jetson devices. 
# This script needs to be executed once on the Jetson after flashing.
# ------------------------------------------------------------------------------

# Update package repository
sudo apt update

# ------------------------------------------------------------------------------
# Install build tools
# ------------------------------------------------------------------------------

sudo apt install -y git build-essential autoconf automake autoconf-archive libtool pkg-config

# ------------------------------------------------------------------------------
# Install libgpiod 1.6 from source (specific version for required)
# ------------------------------------------------------------------------------

echo "Installing libgpiod v1.6..."
TEMP_DIR=$(mktemp -d)
pushd . > /dev/null
git clone --branch v1.6.x https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git $TEMP_DIR
cd $TEMP_DIR
./autogen.sh --enable-tools=yes
make -j$(nproc) && sudo make install
popd > /dev/null
rm -rf "$TEMP_DIR"

# ------------------------------------------------------------------------------
# Install runtime tools (devmem2 utility, I2C development libraries, V4L utilities)
# ------------------------------------------------------------------------------

sudo apt install -y devmem2 libi2c-dev v4l-utils libgpiod2

# ------------------------------------------------------------------------------
# Exchange spi driver to fix performance issues during spi communication and reboot
# see also https://forums.developer.nvidia.com/t/orin-nx-spi-delay-between-transfers/265234/21
# ------------------------------------------------------------------------------

SPI_DRIVER="/usr/lib/modules/5.10.120-tegra/kernel/drivers/spi/spi-tegra114.ko"
if [[ -f $SPI_DRIVER ]] && ! cmp -s $SPI_DRIVER artifacts/spi-tegra114.ko 2>/dev/null; then
    SPI_BACKUP_FILE=$(sudo mktemp -q $SPI_DRIVER.backup.XXXXXX || exit 1)
    sudo mv $SPI_DRIVER $SPI_BACKUP_FILE
    sudo cp artifacts/spi-tegra114.ko /usr/lib/modules/5.10.120-tegra/kernel/drivers/spi/
    echo "Rebooting to apply new SPI driver..."
    sudo depmod -a
fi

# ------------------------------------------------------------------------------
# Install docker (only if not already available)
# ------------------------------------------------------------------------------

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker not found, installing Docker..."
    
    # Add Docker's official GPG key:
    sudo apt install -y ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

    # Update apt cache
    sudo apt update

    # Install docker packages
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

    # Ask user if they want to add current user to docker group
    while true; do
        read -p "Do you want to add the current user ($USER) to the docker group? This allows running Docker without sudo. (y/n): " yn
        case $yn in
            [Yy]* )
                sudo groupadd docker
                sudo usermod -aG docker $USER
                echo "User $USER added to docker group. You may need to log out and back in for changes to take effect."
                break;;
            [Nn]* ) 
                echo "Skipping docker group configuration."
                break;;
            * ) echo "Please answer yes or no.";;
        esac
    done
else
    echo "Docker is already installed, skipping Docker installation."
fi

# ------------------------------------------------------------------------------
# Final cleanup
# ------------------------------------------------------------------------------

sudo apt autoremove -y
echo "Installation of dependencies completed. Please reboot the device to apply all changes."
