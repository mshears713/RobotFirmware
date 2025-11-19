# ESP32 Robotics Firmware - Docker Setup Guide

**Complete containerization for Windows, WSL, and Linux**

---

## Overview

The entire ESP32 Robotics project is now containerized using Docker, providing:

✅ **Consistent Build Environment** - Same tools everywhere
✅ **No Local Dependencies** - Everything runs in containers
✅ **Windows & WSL Support** - Works on both platforms
✅ **Easy Deployment** - Single command to start UI
✅ **Isolated Development** - No conflicts with system packages

---

## Prerequisites

### Windows

1. **Docker Desktop for Windows**
   - Download: https://www.docker.com/products/docker-desktop
   - Minimum: Windows 10 64-bit Pro/Enterprise/Education
   - Enable WSL 2 backend (recommended)

2. **Git for Windows** (optional, for cloning)
   - Download: https://git-scm.com/download/win

### WSL (Windows Subsystem for Linux)

1. **Docker Desktop with WSL 2 Integration**
   - Install Docker Desktop on Windows
   - Enable WSL 2 integration in Docker Desktop settings
   - Or install Docker directly in WSL

2. **USB Access Tools** (for firmware upload)
   ```bash
   # In Windows PowerShell (Admin)
   winget install usbipd
   ```

### Linux

1. **Docker Engine**
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   sudo usermod -aG docker $USER
   ```

2. **Docker Compose**
   ```bash
   sudo apt-get install docker-compose
   ```

---

## Quick Start

### Windows (Command Prompt or PowerShell)

```cmd
# Clone repository
git clone https://github.com/yourusername/RobotFirmware.git
cd RobotFirmware

# Build firmware (in Docker)
docker\docker-build.bat

# Launch UI
docker\docker-run-ui.bat
```

Access UI at: http://localhost:8501

### WSL/Linux (Bash)

```bash
# Clone repository
git clone https://github.com/yourusername/RobotFirmware.git
cd RobotFirmware

# Make scripts executable
chmod +x docker/*.sh

# Build firmware (in Docker)
./docker/docker-build.sh

# Launch UI
./docker/docker-run-ui.sh
```

Access UI at: http://localhost:8501

---

## Container Architecture

### 1. Firmware Builder Container

**Image:** `esp32-robot-firmware:latest`

**Purpose:** Build ESP32 firmware without installing PlatformIO locally

**Includes:**
- Python 3.11
- PlatformIO Core
- ESP32 platform pre-installed
- Build tools (gcc, make, git)

**Usage:**
```bash
# Build firmware
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio run

# Run tests
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio test

# Clean build
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio run --target clean

# Interactive shell
docker-compose -f docker/docker-compose.yml run --rm firmware-builder bash
```

### 2. Streamlit UI Container

**Image:** `esp32-robot-ui:latest`

**Purpose:** Run Robot Console web interface

**Includes:**
- Python 3.11
- Streamlit
- All UI dependencies
- Health checks

**Exposed Ports:**
- 8501 - Streamlit web interface

**Usage:**
```bash
# Start UI
docker-compose -f docker/docker-compose.yml up robot-ui

# Start in background
docker-compose -f docker/docker-compose.yml up -d robot-ui

# Stop UI
docker-compose -f docker/docker-compose.yml down

# View logs
docker-compose -f docker/docker-compose.yml logs -f robot-ui
```

---

## Detailed Commands

### Building Firmware

**Windows:**
```cmd
cd RobotFirmware
docker\docker-build.bat
```

**WSL/Linux:**
```bash
cd RobotFirmware
./docker/docker-build.sh
```

**Output:** `firmware/.pio/build/esp32dev/firmware.bin`

### Uploading Firmware

⚠️ **USB Device Access Required**

#### Windows with Docker Desktop

1. **Enable USB Support:**
   - Open Docker Desktop
   - Go to Settings → Resources → USB devices
   - Enable and share your ESP32 device

2. **Upload:**
   ```cmd
   docker\docker-build-upload.bat
   ```

#### WSL 2

1. **Install usbipd-win** (Windows side):
   ```powershell
   # In PowerShell (Admin)
   winget install usbipd
   ```

2. **Attach USB Device:**
   ```powershell
   # List devices
   usbipd wsl list

   # Attach ESP32 (replace X-Y with your device bus ID)
   usbipd wsl attach --busid X-Y
   ```

3. **Verify in WSL:**
   ```bash
   lsusb
   ls -l /dev/ttyUSB*
   ```

4. **Upload:**
   ```bash
   ./docker/docker-build-upload.sh
   ```

#### Alternative: Upload Without Docker

If USB passthrough is problematic:

```bash
# Exit container, use native PlatformIO
cd firmware
pip install platformio
pio run --target upload
```

### Running the UI

**Windows:**
```cmd
docker\docker-run-ui.bat
```

**WSL/Linux:**
```bash
./docker/docker-run-ui.sh
```

**Access:** http://localhost:8501

**Configure ESP32 IP:**
- Open UI in browser
- Enter ESP32 IP address in sidebar
- Click "Connect"

### Running Tests

**Windows:**
```cmd
docker-compose -f docker\docker-compose.yml run --rm firmware-builder pio test
```

**WSL/Linux:**
```bash
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio test
```

---

## Docker Compose Services

### Available Services

```yaml
services:
  firmware-builder  # Build environment
  robot-ui          # Streamlit interface
  telemetry-logger  # Optional: automated testing
```

### Start All Services

```bash
docker-compose -f docker/docker-compose.yml up
```

### Start Specific Service

```bash
docker-compose -f docker/docker-compose.yml up robot-ui
```

### Stop All Services

```bash
docker-compose -f docker/docker-compose.yml down
```

### View Logs

```bash
docker-compose -f docker/docker-compose.yml logs -f
```

---

## Volume Mounts

### Firmware Builder

```
Host Path              Container Path
---------              --------------
./firmware        →    /workspace/firmware
./docs            →    /workspace/docs
./examples        →    /workspace/examples
/dev              →    /dev (USB devices)
```

### UI Container

```
Host Path              Container Path
---------              --------------
./ui              →    /app
robot-data (volume) →  /app/data (SQLite persistence)
```

---

## Environment Variables

### UI Container

Set in `docker-compose.yml` or via command line:

```bash
# Set ESP32 IP
docker-compose -f docker/docker-compose.yml run -e ESP32_IP=192.168.1.150 robot-ui

# Disable telemetry
docker-compose -f docker/docker-compose.yml run -e STREAMLIT_BROWSER_GATHER_USAGE_STATS=false robot-ui
```

---

## Development Workflow

### 1. Edit Code Locally

Files are mounted into containers, so changes are live:

```bash
# Edit firmware code
vim firmware/src/main.cpp

# Rebuild in container
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio run
```

### 2. Iterative Development

```bash
# Keep UI running with live reload
docker-compose -f docker/docker-compose.yml up robot-ui

# Edit UI code - changes appear immediately
vim ui/robot_console.py
```

### 3. Test Changes

```bash
# Run unit tests
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio test

# Run integration tests
docker-compose -f docker/docker-compose.yml run --rm telemetry-logger
```

---

## Troubleshooting

### Docker Not Running

**Windows:**
```cmd
# Check Docker Desktop is running
# System tray should show Docker icon
```

**WSL/Linux:**
```bash
# Check Docker daemon
sudo systemctl status docker

# Start Docker
sudo systemctl start docker
```

### Cannot Access UI

**Check container status:**
```bash
docker ps
docker-compose -f docker/docker-compose.yml ps
```

**Check logs:**
```bash
docker-compose -f docker/docker-compose.yml logs robot-ui
```

**Verify port:**
```bash
# Should show 0.0.0.0:8501
docker port esp32-robot-ui
```

### USB Device Not Found

**Windows:**
- Ensure Docker Desktop has USB device sharing enabled
- Try alternative: Use PlatformIO outside Docker for upload

**WSL:**
```bash
# Verify USB device attached
lsusb

# Re-attach if needed
# In PowerShell: usbipd wsl attach --busid X-Y

# Check permissions
ls -l /dev/ttyUSB0
```

### Build Failures

**Clean and rebuild:**
```bash
# Remove old build
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio run --target clean

# Rebuild image
docker-compose -f docker/docker-compose.yml build --no-cache firmware-builder

# Build firmware
docker-compose -f docker/docker-compose.yml run --rm firmware-builder pio run
```

### Image Size Too Large

**Prune unused images:**
```bash
docker image prune
docker system prune
```

---

## Performance Optimization

### Build Cache

Docker caches layers for faster rebuilds:

```bash
# First build: ~5 minutes (downloads everything)
# Subsequent builds: ~30 seconds (uses cache)
```

### Persistent Volumes

Data persists across container restarts:

```bash
# View volumes
docker volume ls

# Backup SQLite database
docker cp esp32-robot-ui:/app/data/robot_telemetry.db ./backup.db
```

---

## Production Deployment

### Build Optimized Images

```bash
# Production build with optimization
docker build -f docker/Dockerfile.ui -t esp32-robot-ui:prod --build-arg BUILD_ENV=production .
```

### Use Docker Compose Override

Create `docker-compose.override.yml`:

```yaml
version: '3.8'
services:
  robot-ui:
    environment:
      - ESP32_IP=192.168.1.100  # Your production IP
    restart: always
```

### Deploy with Docker Swarm or Kubernetes

```bash
# Docker Swarm
docker stack deploy -c docker/docker-compose.yml robot

# Kubernetes
kubectl apply -f k8s/
```

---

## Comparison: Docker vs Native

| Aspect | Docker | Native |
|--------|--------|--------|
| Setup Time | 5 min | 15-30 min |
| Disk Space | ~2 GB | ~1 GB |
| Build Speed | Fast | Faster |
| Portability | Excellent | OS-dependent |
| USB Upload | Complex | Simple |
| Isolation | Complete | None |
| Updates | Easy | Manual |

**Recommendation:**
- **Development:** Docker for consistency
- **Firmware Upload:** Native PlatformIO (easier USB access)
- **UI Deployment:** Docker (easier distribution)

---

## Advanced Usage

### Multi-Stage Builds

```dockerfile
# Optimize image size with multi-stage builds
FROM python:3.11 as builder
WORKDIR /app
COPY requirements.txt .
RUN pip install --user -r requirements.txt

FROM python:3.11-slim
COPY --from=builder /root/.local /root/.local
COPY ui/ /app/
ENV PATH=/root/.local/bin:$PATH
CMD ["streamlit", "run", "robot_console.py"]
```

### Health Monitoring

```bash
# Check health status
docker inspect --format='{{.State.Health.Status}}' esp32-robot-ui

# Auto-restart on failure (in docker-compose.yml)
restart: unless-stopped
```

### Resource Limits

```yaml
services:
  robot-ui:
    deploy:
      resources:
        limits:
          cpus: '0.5'
          memory: 512M
```

---

## Scripts Reference

### Windows Scripts

- `docker\docker-build.bat` - Build firmware
- `docker\docker-build-upload.bat` - Upload firmware
- `docker\docker-run-ui.bat` - Start UI

### WSL/Linux Scripts

- `docker/docker-build.sh` - Build firmware
- `docker/docker-build-upload.sh` - Upload firmware
- `docker/docker-run-ui.sh` - Start UI

---

## Support

For Docker-related issues:

1. **Check Docker installation:** `docker --version`
2. **Check Docker Compose:** `docker-compose --version`
3. **View container logs:** `docker logs <container-name>`
4. **Debug interactively:** `docker-compose run --rm <service> bash`

---

## Next Steps

1. **Quick Test:**
   ```bash
   ./docker/docker-run-ui.sh
   # Open http://localhost:8501
   ```

2. **Build Firmware:**
   ```bash
   ./docker/docker-build.sh
   ```

3. **Customize:**
   - Edit `docker-compose.yml` for your needs
   - Add environment variables
   - Configure networking

4. **Deploy:**
   - Follow production deployment guide
   - Set up monitoring
   - Configure backups

---

**Docker containerization complete!** 🐳

The project now runs consistently on Windows, WSL, and Linux with minimal setup.
