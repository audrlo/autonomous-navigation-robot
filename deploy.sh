#!/bin/bash

# Deployment script for autonomous robot to Raspberry Pi 5

set -e

# Configuration
PI_HOST="${1:-robot@raspberrypi.local}"
PI_ARCH="linux/arm64"
IMAGE_NAME="autonomous-robot"
TAG="latest"

echo "🤖 Autonomous Robot Deployment Script"
echo "Target: $PI_HOST"
echo "Architecture: $PI_ARCH"

# Build ARM64 image
echo "📦 Building ARM64 Docker image..."
docker buildx build \
    --platform $PI_ARCH \
    -f Dockerfile.arm64 \
    -t ${IMAGE_NAME}:${TAG}-arm64 \
    --load \
    .

# Save image to tar file
echo "💾 Exporting Docker image..."
docker save ${IMAGE_NAME}:${TAG}-arm64 | gzip > robot-image-arm64.tar.gz

# Copy files to Raspberry Pi
echo "📤 Copying files to Raspberry Pi..."
scp robot-image-arm64.tar.gz $PI_HOST:~/
scp docker-compose.yml $PI_HOST:~/
scp -r config $PI_HOST:~/ || true

# Deploy on Raspberry Pi
echo "🚀 Deploying on Raspberry Pi..."
ssh $PI_HOST << 'EOF'
    # Load Docker image
    echo "Loading Docker image..."
    docker load < robot-image-arm64.tar.gz
    
    # Tag the image
    docker tag autonomous-robot:latest-arm64 autonomous-robot:latest
    
    # Stop existing containers
    echo "Stopping existing containers..."
    docker-compose down || true
    
    # Start robot services
    echo "Starting robot services..."
    docker-compose up -d robot
    
    # Clean up
    rm robot-image-arm64.tar.gz
    
    echo "✅ Robot deployment complete!"
    echo "Check status with: docker-compose logs -f robot"
EOF

# Clean up local files
rm robot-image-arm64.tar.gz

echo "🎉 Deployment complete!"
echo "SSH into your Pi and run 'docker-compose logs -f robot' to see the logs"