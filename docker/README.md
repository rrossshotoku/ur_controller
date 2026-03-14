# Docker Development Environment

This directory contains the Docker setup for developing the UR Controller application.

## Prerequisites

- Docker Desktop (Windows/Mac) or Docker Engine (Linux)
- Docker Compose v2

## Quick Start

### 1. Start the Development Environment

```bash
cd docker

# Build and start both dev container and URSim
docker compose up -d

# This will:
# - Build the development container with all dependencies
# - Start URSim (UR5 simulator)
# - Create a shared network between containers
```

### 2. Access URSim Polyscope GUI

Open your browser and navigate to:

```
http://localhost:6080
```

This provides a web-based VNC connection to the Polyscope interface.

**Important:** Before running any programs, you need to:
1. In Polyscope, go to the hamburger menu (top right)
2. Set the robot to **Remote Control** mode
3. Power on the robot (if not already on)
4. Release the brakes

### 3. Enter the Development Container

```bash
# Open a shell in the dev container
docker compose exec dev bash

# You're now in /workspace which is your project root
```

### 4. Build and Test

```bash
# Inside the dev container:
cd /workspace
mkdir -p build && cd build

# Configure with CMake
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug

# Build
ninja

# Run tests
ctest --output-on-failure

# Test connection to URSim
./apps/connection_test --ip ursim
```

### 5. Stop the Environment

```bash
docker compose down
```

## Container Details

### `dev` - Development Container

- Ubuntu 22.04 base
- GCC, Clang, CMake, Ninja
- All project dependencies pre-installed:
  - Eigen3
  - ur_rtde
  - FCL
  - Ruckig
  - yaml-cpp
  - spdlog
  - Catch2
  - CLI11

### `ursim` - Universal Robots Simulator

- Official UR e-Series simulator
- Configured as UR5 by default
- Exposes all standard UR ports:
  - 30004: RTDE
  - 30003: Real-time interface
  - 30002: Secondary interface
  - 30001: Primary interface
  - 29999: Dashboard server
  - 6080: Web VNC

## Networking

Both containers are on the `ur_network` bridge network. From the dev container, you can reach URSim using the hostname `ursim`.

```cpp
// In your code:
ur_rtde::RTDEReceiveInterface rtde("ursim");
```

## Volumes

- `build-cache`: Persists build artifacts between container restarts
- `ursim-programs`: Persists URSim programs

## Troubleshooting

### URSim won't start
- Ensure Docker has enough memory allocated (4GB+ recommended)
- Check logs: `docker compose logs ursim`

### Can't connect to URSim from dev container
- Ensure URSim is fully started (Polyscope visible in browser)
- Robot must be in Remote Control mode
- Robot must be powered on with brakes released

### Build errors
- Ensure you're running commands inside the dev container
- Try removing build cache: `docker volume rm ur_controller_build_cache`

### Permission issues with mounted files
On Linux, you may need to ensure your user ID matches inside the container. Add to docker-compose.yml under `dev`:
```yaml
user: "${UID}:${GID}"
```
