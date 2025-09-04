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
ROS_DISTRO="jazzy"  # Default to current LTS

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
    --help)
      echo "Usage: $0 --name INSTANCE_NAME --namespace NAMESPACE --config CONFIG_FILE [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --service-name SERVICE_NAME    Custom service name (default: mechanical-press-INSTANCE_NAME)"
      echo "  --ros-distro ROS_DISTRO        ROS distribution (default: humble)"
      echo ""
      echo "Examples:"
      echo "  # Development instance"
      echo "  $0 --name dev-press --namespace /dev --config config/examples/development.yaml"
      echo ""
      echo "  # Production instance with custom ROS distro"
      echo "  $0 --name factory-line1-press1 --namespace /factory/line1/press1 --config config/examples/factory_line1_press1.yaml --ros-distro jazzy"
      echo ""
      echo "Available example configs:"
      find "$PROJECT_ROOT/config/examples" -name "*.yaml" 2>/dev/null | sed 's|.*/|  - config/examples/|' || true
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
echo ""

# Check if mechanical press package is installed
if ! dpkg -l mechanical-press &>/dev/null; then
  echo "Warning: mechanical-press package not installed"
  echo "You may need to run: sudo dpkg -i mechanical-press_*.deb"
  echo "Continuing anyway for development setup..."
fi

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

# Create emoco user if it doesn't exist
sudo useradd -r -s /bin/false emoco 2>/dev/null || true

# Copy configuration
echo "Installing configuration..."
sudo cp "$CONFIG_FILE" "$CONFIG_DIR/params.yaml"

# Create instance environment file
sudo tee "$CONFIG_DIR/instance.env" > /dev/null << EOF
INSTANCE_NAME=$INSTANCE_NAME
NAMESPACE=$NAMESPACE
CONFIG_FILE=$CONFIG_DIR/params.yaml
ROS_DISTRO=jazzy
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

# Launch command - use bash to source ROS environment
ExecStart=/bin/bash -c 'source /opt/ros/\${ROS_DISTRO}/setup.bash && ros2 launch \${APP_PACKAGE} \${APP_LAUNCH_FILE} namespace:=\${NAMESPACE} param_file:=\${CONFIG_FILE} instance_name:=\${INSTANCE_NAME}'

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