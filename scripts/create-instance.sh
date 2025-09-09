#!/bin/bash
set -e

# Create a mechanical press instance with specific configuration
# Usage: ./scripts/create-instance.sh --name INSTANCE_NAME --namespace NAMESPACE --config CONFIG_FILE

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Default values
INSTANCE_NAME=""
NAMESPACE=""
CONFIG_FILE=""
SERVICE_NAME=""
ROS_DISTRO="${ROS_DISTRO:-jazzy}"  # Use environment variable or default to current LTS
WORKSPACE_PATH=""  # Will be detected or prompted for

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --name)
      INSTANCE_NAME="$2"
      shift 2
      ;;
    --namespace)
      NAMESPACE="$2" 
      shift 2
      ;;
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --service-name)
      SERVICE_NAME="$2"
      shift 2
      ;;
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --workspace)
      WORKSPACE_PATH="$2"
      shift 2
      ;;
    --help)
      echo "Usage: $0 --name INSTANCE_NAME --namespace NAMESPACE --config CONFIG_FILE [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --service-name SERVICE_NAME    Custom service name (default: mechanical-press-INSTANCE_NAME)"
      echo "  --ros-distro ROS_DISTRO        ROS distribution (default: jazzy, or \$ROS_DISTRO env var)"
      echo "  --workspace WORKSPACE_PATH     ROS workspace path (auto-detected or prompted if not provided)"
      echo ""
      echo "Examples:"
      echo "  # Development instance with default ROS distro"
      echo "  $0 --name dev-press --namespace /dev --config config/examples/development.yaml"
      echo ""
      echo "  # Using environment variable for ROS distro"
      echo "  export ROS_DISTRO=humble"
      echo "  $0 --name dev-press --namespace /dev --config config/examples/development.yaml"
      echo ""
      echo "  # Production instance with specific ROS distro"
      echo "  $0 --name factory-line1-press1 --namespace /factory/line1/press1 --config config/examples/factory_line1_press1.yaml --ros-distro humble"
      echo ""
      echo "  # Specify custom workspace location"
      echo "  $0 --name dev-press --namespace /dev --config config/examples/development.yaml --workspace /path/to/workspace"
      echo ""
      echo "Available example configs:"
      find "$PROJECT_ROOT/config/examples" -name "*.yaml" 2>/dev/null | sed 's|.*/|  - config/examples/|' || true
      echo ""
      echo "Note: This script will build mechanical_press from source and install a clean copy"
      echo "to the service location. Your workspace doesn't need to be pre-built."
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

# Validate required arguments
if [ -z "$INSTANCE_NAME" ] || [ -z "$NAMESPACE" ] || [ -z "$CONFIG_FILE" ]; then
  echo "Error: Missing required arguments"
  echo "Usage: $0 --name INSTANCE_NAME --namespace NAMESPACE --config CONFIG_FILE"
  echo "Use --help for more information"
  exit 1
fi

# Set service name if not provided
if [ -z "$SERVICE_NAME" ]; then
  SERVICE_NAME="mechanical-press-$INSTANCE_NAME"
fi

# Check if emoco user exists (required for service execution)
if ! id emoco &>/dev/null; then
  echo "Error: User 'emoco' does not exist"
  echo "Create the service user first:"
  echo "  sudo useradd -r -s /bin/false emoco"
  echo ""
  exit 1
fi

# Always use workspace builds (package installation covered in PACKAGING_FOR_DISTRIBUTION.md)
echo "Building from workspace source..."

# Auto-detect or prompt for workspace
  if [ -z "$WORKSPACE_PATH" ]; then
    # First, check if we're running from within a workspace structure
    if [ -f "../../package.xml" ] && [ -d "../../src" ]; then
      # We're in a package directory, workspace is two levels up
      WORKSPACE_PATH="$(cd ../.. && pwd)"
      echo "Auto-detected workspace from script location: $WORKSPACE_PATH"
    # Then try common workspace locations
    elif [ -d "$HOME/ros2_ws/src/mechanical_press" ]; then
      WORKSPACE_PATH="$HOME/ros2_ws"
      echo "Auto-detected workspace: $WORKSPACE_PATH"
    else
      echo "Could not auto-detect workspace containing mechanical_press package."
      echo "Common locations checked:"
      echo "  - Relative to script location (../../)"
      echo "  - ~/ros2_ws/src/mechanical_press"
      echo ""
      read -p "Please enter your ROS workspace path: " WORKSPACE_PATH
    fi
  fi

  # Expand tilde in workspace path
  WORKSPACE_PATH="${WORKSPACE_PATH/#\~/$HOME}"

# Resolve config file path
if [ ! -f "$CONFIG_FILE" ]; then
  # Try relative to project root
  CONFIG_FILE="$PROJECT_ROOT/$CONFIG_FILE"
  if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found: $CONFIG_FILE"
    exit 1
  fi
fi

echo "Creating mechanical press instance:"
echo "  Instance Name: $INSTANCE_NAME"
echo "  Namespace: $NAMESPACE" 
echo "  Config File: $CONFIG_FILE"
echo "  Service Name: $SERVICE_NAME"
echo "  Install Method: Building from workspace source"
echo "  Workspace: $WORKSPACE_PATH"
echo ""

# Create instance-specific directories
INSTANCE_DIR="/opt/rosapps/mechanical-press-instances/$INSTANCE_NAME"
CONFIG_DIR="/etc/rosapps/mechanical-press-instances/$INSTANCE_NAME"
LOG_DIR="/var/log/rosapps-mechanical-press-$INSTANCE_NAME"
STATE_DIR="/var/lib/rosapps-mechanical-press-$INSTANCE_NAME"

echo "Creating directories..."
sudo mkdir -p "$INSTANCE_DIR"
sudo mkdir -p "$CONFIG_DIR"
sudo mkdir -p "$LOG_DIR"
sudo mkdir -p "$STATE_DIR"

# Copy configuration
echo "Installing configuration..."
sudo cp "$CONFIG_FILE" "$CONFIG_DIR/params.yaml"

# Build and install mechanical_press from workspace source
echo "Building mechanical_press from workspace source..."
echo "Workspace: $WORKSPACE_PATH"

# Check if source exists
if [ ! -d "$WORKSPACE_PATH/src/mechanical_press" ]; then
    echo "Error: mechanical_press source not found at $WORKSPACE_PATH/src/mechanical_press"
    echo ""
    echo "Make sure:"
    echo "  1. You're in the correct workspace directory"
    echo "  2. The mechanical_press package is in the src/ directory"
    echo "  3. The workspace path is correct: $WORKSPACE_PATH"
    exit 1
fi

if [ -d "$WORKSPACE_PATH" ]; then
    # Create temporary clean build for just mechanical_press
    TEMP_INSTALL="/tmp/mechanical_press_install_$(date +%s)"
    
    echo "Building mechanical_press in clean environment..."
    cd "$WORKSPACE_PATH"
    
    # Clear Python cache to ensure fresh build
    find "$WORKSPACE_PATH/src/mechanical_press" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
    find "$WORKSPACE_PATH/src/mechanical_press" -name "*.pyc" -delete 2>/dev/null || true
    
    # Source ROS environment
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    
    colcon build --packages-select mechanical_press \
        --install-base "$TEMP_INSTALL" \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    
    # Copy only the clean build to service location
    sudo mkdir -p "$INSTANCE_DIR/install"
    sudo cp -r "$TEMP_INSTALL"/* "$INSTANCE_DIR/install/"
    sudo chown -R emoco:emoco "$INSTANCE_DIR/install"
    
    # Cleanup
    rm -rf "$TEMP_INSTALL"
    
    echo "✓ Built and installed from workspace source to service location"
else
    echo "Error: Workspace directory not found at $WORKSPACE_PATH"
    exit 1
fi

# Create instance environment file
sudo tee "$CONFIG_DIR/instance.env" > /dev/null << EOF
INSTANCE_NAME=$INSTANCE_NAME
NAMESPACE=$NAMESPACE
CONFIG_FILE=$CONFIG_DIR/params.yaml
ROS_DISTRO=$ROS_DISTRO
APP_PACKAGE=mechanical_press
APP_LAUNCH_FILE=mechanical_press.launch.py
ROS_LOCALHOST_ONLY=1
EOF

# Create systemd service file
echo "Creating systemd service..."
sudo tee "/etc/systemd/system/$SERVICE_NAME.service" > /dev/null << EOF
[Unit]
Description=Mechanical Press Instance: $INSTANCE_NAME
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=emoco
Group=emoco
Restart=always
RestartSec=2

# Instance-specific environment
EnvironmentFile=-$CONFIG_DIR/instance.env
Environment=HOME=$STATE_DIR
Environment=ROS_LOG_DIR=$LOG_DIR

# Working directory
WorkingDirectory=$INSTANCE_DIR

# Launch command - source ROS and workspace environment (ignore missing workspace files)
ExecStart=/bin/bash -c 'source /opt/ros/\${ROS_DISTRO}/setup.bash && source $INSTANCE_DIR/install/setup.bash 2>/dev/null || true && ros2 launch \${APP_PACKAGE} \${APP_LAUNCH_FILE} namespace:=\${NAMESPACE} param_file:=\${CONFIG_FILE} instance_name:=\${INSTANCE_NAME}'

# Security hardening
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=full
ProtectHome=true

# Logging and state
StateDirectory=rosapps-mechanical-press-$INSTANCE_NAME
LogsDirectory=rosapps-mechanical-press-$INSTANCE_NAME

[Install]
WantedBy=multi-user.target
EOF

# Set ownership
sudo chown -R emoco:emoco "$INSTANCE_DIR" "$LOG_DIR" "$STATE_DIR"

# Reload systemd
sudo systemctl daemon-reload

echo ""
echo "✓ Instance '$INSTANCE_NAME' created successfully!"
echo ""
echo "Service: $SERVICE_NAME.service"
echo "Config: $CONFIG_DIR/params.yaml"
echo "Logs: $LOG_DIR"
echo "State: $STATE_DIR"
echo ""
echo "To start the instance:"
echo "  sudo systemctl enable --now $SERVICE_NAME.service"
echo ""
echo "To check status:"
echo "  sudo systemctl status $SERVICE_NAME.service"
echo ""  
echo "To view logs:"
echo "  journalctl -u $SERVICE_NAME -f"
echo ""
echo "To test the instance:"
echo "  ros2 topic list | grep '$NAMESPACE'"
echo "  ros2 service list | grep '$NAMESPACE'"