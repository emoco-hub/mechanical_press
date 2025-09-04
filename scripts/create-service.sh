#!/bin/bash
set -e

# Create systemd service for mechanical press
# Usage: ./scripts/create-service.sh [service-name]

SERVICE_NAME=${1:-"mechanical-press"}
APP_NAME="mechanical-press"

echo "Creating systemd service: ros2_app@${SERVICE_NAME}.service"

# Create systemd service file
sudo tee "/etc/systemd/system/ros2_app@${SERVICE_NAME}.service" > /dev/null << EOF
[Unit]
Description=ROS 2 app: ${SERVICE_NAME}
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=emoco
Group=emoco
Restart=always
RestartSec=2
EnvironmentFile=-/etc/rosapps/${APP_NAME}.env
WorkingDirectory=/opt/rosapps/${APP_NAME}/current
ExecStart=/opt/rosapps/${APP_NAME}/current/bin/run.sh

# Security hardening
NoNewPrivileges=yes
PrivateTmp=yes
ProtectSystem=full
ProtectHome=true

# State and logging
StateDirectory=rosapps-${SERVICE_NAME}
LogsDirectory=rosapps-${SERVICE_NAME}
Environment=HOME=/var/lib/rosapps-${SERVICE_NAME}
Environment=ROS_LOG_DIR=/var/log/rosapps-${SERVICE_NAME}

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd
sudo systemctl daemon-reload

echo "Service created: ros2_app@${SERVICE_NAME}.service"
echo ""
echo "To start the service:"
echo "  sudo systemctl enable --now ros2_app@${SERVICE_NAME}.service"
echo ""
echo "To check status:"
echo "  sudo systemctl status ros2_app@${SERVICE_NAME}.service"
echo ""
echo "To view logs:"
echo "  journalctl -u ros2_app@${SERVICE_NAME} -f"