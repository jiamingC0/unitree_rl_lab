#!/bin/bash

# Script to install dependencies for unitree_rl_lab deployment

echo "=========================================="
echo "Installing Unitree RL Lab Dependencies"
echo "=========================================="

# Update package list
echo "[1/3] Updating package list..."
sudo apt update

# Install Cyclone DDS
echo "[2/3] Installing Cyclone DDS..."
sudo apt install -y libcyclonedds-dev libcyclonedds-cxx-dev

# Verify installation
echo "[3/3] Verifying installation..."
if dpkg -l | grep -q cyclonedds; then
    echo "✓ Cyclone DDS installed successfully"
else
    echo "✗ Failed to install Cyclone DDS"
    echo "Please install manually:"
    echo "  sudo apt install libcyclonedds-dev libcyclonedds-cxx-dev"
fi

echo ""
echo "=========================================="
echo "Installation complete!"
echo "Now you can compile the controller:"
echo "  cd deploy/robots/g1_29dof/build"
echo "  make clean && make"
echo "=========================================="
