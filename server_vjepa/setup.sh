#!/usr/bin/env bash
# =============================================================================
# setup.sh — Phase 1 environment bootstrap for V-JEPA 2-AC robotic arm project
# Target hardware: 2x NVIDIA A100 40GB (CUDA 12.x)
# =============================================================================

set -Eeuo pipefail
IFS=$'\n\t'

# --------------------------------------------------------------------------
# Configuration (override via environment variables before running)
# --------------------------------------------------------------------------
VENV_DIR="${VENV_DIR:-./vjepa2_env}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
CUDA_TAG="${CUDA_TAG:-cu121}"                 # PyTorch wheel index tag for CUDA 12.1
TORCH_VERSION="${TORCH_VERSION:-2.4.1}"
TORCHVISION_VERSION="${TORCHVISION_VERSION:-0.19.1}"
TORCHAUDIO_VERSION="${TORCHAUDIO_VERSION:-2.4.1}"
INSTALL_ROS2="${INSTALL_ROS2:-false}"         # set true to attempt ROS2 apt install
ROS_DISTRO_OVERRIDE="${ROS_DISTRO_OVERRIDE:-}"
LOG_FILE="${LOG_FILE:-./setup.log}"
REQUIRED_GPU_COUNT="${REQUIRED_GPU_COUNT:-2}"

# --------------------------------------------------------------------------
# Logging helpers
# --------------------------------------------------------------------------
log()  { printf '\033[1;32m[INFO]\033[0m  %s\n' "$*"; }
warn() { printf '\033[1;33m[WARN]\033[0m  %s\n' "$*"; }
err()  { printf '\033[1;31m[ERROR]\033[0m %s\n' "$*" >&2; }

trap 'err "Setup failed at line $LINENO (exit code $?)."' ERR

log "Starting V-JEPA 2-AC Phase 1 environment setup"

# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
require_cmd() {
    local cmd="$1"
    if ! command -v "$cmd" >/dev/null 2>&1; then
        err "Required command '$cmd' not found on PATH."
        return 1
    fi
}

apt_install() {
    if command -v apt-get >/dev/null 2>&1; then
        log "Installing system packages: $*"
        sudo apt-get update -y
        sudo apt-get install -y "$@"
    else
        warn "apt-get not available; skipping system package install for: $*"
    fi
}

# --------------------------------------------------------------------------
# 1. Pre-flight checks
# --------------------------------------------------------------------------
log "Running pre-flight checks"

require_cmd "$PYTHON_BIN"

PY_VER="$("$PYTHON_BIN" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
log "Detected Python version: $PY_VER"
PY_MAJOR="${PY_VER%%.*}"
PY_MINOR="${PY_VER##*.}"
if [ "$PY_MAJOR" -lt 3 ] || { [ "$PY_MAJOR" -eq 3 ] && [ "$PY_MINOR" -lt 9 ]; }; then
    err "Python 3.9+ is required (found $PY_VER)."
    exit 1
fi

if ! "$PYTHON_BIN" -c 'import venv' >/dev/null 2>&1; then
    warn "python3-venv module not found; attempting to install it."
    apt_install "python3-venv" "python3-pip" "python3-dev" || true
fi

if command -v nvidia-smi >/dev/null 2>&1; then
    GPU_COUNT="$(nvidia-smi --query-gpu=name --format=csv,noheader | wc -l | tr -d ' ')"
    log "Detected $GPU_COUNT GPU(s):"
    nvidia-smi --query-gpu=index,name,memory.total,driver_version --format=csv,noheader

    DRIVER_CUDA_VERSION="$(nvidia-smi | grep -oP 'CUDA Version:\s*\K[0-9]+\.[0-9]+' || true)"
    if [ -n "$DRIVER_CUDA_VERSION" ]; then
        log "Driver reports max supported CUDA version: $DRIVER_CUDA_VERSION"
        DRIVER_MAJOR="${DRIVER_CUDA_VERSION%%.*}"
        if [ "$DRIVER_MAJOR" -lt 12 ]; then
            warn "Driver supports CUDA < 12. PyTorch cu121 wheels may not work; consider updating the NVIDIA driver."
        fi
    fi

    if [ "$GPU_COUNT" -lt "$REQUIRED_GPU_COUNT" ]; then
        warn "Expected $REQUIRED_GPU_COUNT GPUs for this project but found $GPU_COUNT. Continuing anyway."
    fi
else
    warn "nvidia-smi not found. GPU drivers may not be installed. Continuing with CPU-only fallback wheels for torch is NOT recommended for this project."
fi

# System libraries commonly needed for video decode (V-JEPA2 datasets), OpenCV, and building extensions
log "Ensuring core system libraries are present (ffmpeg, build tools, video/image libs)"
apt_install \
    build-essential \
    git \
    curl \
    wget \
    ffmpeg \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libjpeg-dev \
    libpng-dev \
    unzip || true

# --------------------------------------------------------------------------
# 2. Create Python virtual environment
# --------------------------------------------------------------------------
if [ -d "$VENV_DIR" ]; then
    log "Virtual environment already exists at $VENV_DIR — reusing it."
else
    log "Creating virtual environment at $VENV_DIR"
    "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
log "Activated virtual environment: $(python -c 'import sys; print(sys.prefix)')"

log "Upgrading pip, setuptools, wheel"
pip install --upgrade pip setuptools wheel

# --------------------------------------------------------------------------
# 3. Install PyTorch stack with CUDA 12 support
# --------------------------------------------------------------------------
log "Installing PyTorch $TORCH_VERSION / torchvision $TORCHVISION_VERSION / torchaudio $TORCHAUDIO_VERSION (CUDA tag: $CUDA_TAG)"
pip install \
    "torch==${TORCH_VERSION}" \
    "torchvision==${TORCHVISION_VERSION}" \
    "torchaudio==${TORCHAUDIO_VERSION}" \
    --index-url "https://download.pytorch.org/whl/${CUDA_TAG}"

log "Verifying PyTorch CUDA availability"
python - <<'PYEOF'
import torch
print(f"torch version:        {torch.__version__}")
print(f"cuda available:       {torch.cuda.is_available()}")
print(f"cuda version (build): {torch.version.cuda}")
print(f"gpu count visible:    {torch.cuda.device_count()}")
for i in range(torch.cuda.device_count()):
    print(f"  [{i}] {torch.cuda.get_device_name(i)}")
PYEOF

# --------------------------------------------------------------------------
# 4. Install V-JEPA 2 / ML dependencies
# --------------------------------------------------------------------------
log "Installing V-JEPA 2 and general ML/vision dependencies"
pip install \
    "transformers>=4.44" \
    "accelerate>=0.33" \
    "huggingface_hub>=0.24" \
    "timm>=1.0" \
    "einops" \
    "numpy<2" \
    "scipy" \
    "opencv-python-headless" \
    "Pillow" \
    "av" \
    "decord" \
    "pandas" \
    "pyyaml" \
    "omegaconf" \
    "hydra-core" \
    "iopath" \
    "submitit" \
    "tqdm" \
    "tensorboard" \
    "wandb" \
    "matplotlib" \
    "scikit-learn" \
    "jupyterlab"

# --------------------------------------------------------------------------
# 5. Robot control / API dependencies (standard, non-ROS)
# --------------------------------------------------------------------------
log "Installing standard robot control and simulation APIs"
pip install \
    "pyserial" \
    "dynamixel-sdk" \
    "pymodbus" \
    "gymnasium" \
    "mujoco" \
    "pybullet" \
    "modern-robotics" \
    "opencv-contrib-python-headless" \
    "pyrealsense2" || warn "One or more optional robot-API packages failed to install. Continuing."

# --------------------------------------------------------------------------
# 6. Optional: ROS2 installation (system-level, via apt)
# --------------------------------------------------------------------------
if [ "$INSTALL_ROS2" = "true" ]; then
    log "INSTALL_ROS2=true — attempting ROS2 installation via apt"

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        UBUNTU_CODENAME="${VERSION_CODENAME:-unknown}"
    else
        UBUNTU_CODENAME="unknown"
    fi

    if [ -n "$ROS_DISTRO_OVERRIDE" ]; then
        ROS_DISTRO="$ROS_DISTRO_OVERRIDE"
    else
        case "$UBUNTU_CODENAME" in
            jammy)  ROS_DISTRO="humble" ;;
            noble)  ROS_DISTRO="jazzy" ;;
            *)      ROS_DISTRO="" ;;
        esac
    fi

    if [ -z "$ROS_DISTRO" ]; then
        warn "Could not determine a supported ROS2 distro for Ubuntu codename '$UBUNTU_CODENAME'. Skipping ROS2 install. Set ROS_DISTRO_OVERRIDE to force a distro."
    elif [ -d "/opt/ros/${ROS_DISTRO}" ]; then
        log "ROS2 '$ROS_DISTRO' already installed at /opt/ros/${ROS_DISTRO} — skipping."
    else
        log "Installing ROS2 '$ROS_DISTRO' for Ubuntu '$UBUNTU_CODENAME'"
        apt_install locales software-properties-common curl gnupg lsb-release
        sudo locale-gen en_US en_US.UTF-8 || true
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true

        sudo mkdir -p /usr/share/keyrings
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /tmp/ros-archive-keyring-source.key
        sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
            < /tmp/ros-archive-keyring-source.key

        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

        apt_install "ros-${ROS_DISTRO}-ros-base" "python3-colcon-common-extensions" "python3-rosdep"

        if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
            sudo rosdep init || true
        fi
        rosdep update || true

        log "ROS2 '$ROS_DISTRO' installed. Source it with: source /opt/ros/${ROS_DISTRO}/setup.bash"
    fi

    log "Installing rclpy-compatible Python bindings into the virtual environment"
    pip install "empy<4" "lark" "catkin_pkg" "netifaces" || true
else
    log "INSTALL_ROS2=false (default) — skipping ROS2 install. Using standard robot APIs (pyserial, gymnasium, pybullet, mujoco, modern-robotics) instead."
    log "Re-run with INSTALL_ROS2=true ./setup.sh to additionally install ROS2 alongside these APIs."
fi

# --------------------------------------------------------------------------
# 7. Freeze environment for reproducibility
# --------------------------------------------------------------------------
REQUIREMENTS_LOCK="requirements.lock.txt"
log "Writing resolved dependency versions to $REQUIREMENTS_LOCK"
pip freeze > "$REQUIREMENTS_LOCK"

# --------------------------------------------------------------------------
# 8. Final summary
# --------------------------------------------------------------------------
log "Phase 1 setup complete."
log "Virtual environment: $VENV_DIR"
log "Activate it with:    source $VENV_DIR/bin/activate"
log "Dependency lockfile: $REQUIREMENTS_LOCK"

python - <<'PYEOF'
import torch
print("\n=== Environment Summary ===")
print(f"Torch:   {torch.__version__} (CUDA build {torch.version.cuda})")
print(f"CUDA OK: {torch.cuda.is_available()}")
print(f"GPUs:    {torch.cuda.device_count()}")
PYEOF

log "Next: Phase 2 — download V-JEPA 2-AC checkpoints and configure the robotic arm interface."
