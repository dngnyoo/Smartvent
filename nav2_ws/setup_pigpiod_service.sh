#!/bin/bash
# Setup pigpiod as a systemd service for GoPiGo3

echo "Creating pigpiod systemd service..."

# Create systemd service file
sudo tee /etc/systemd/system/pigpiod.service > /dev/null <<EOF
[Unit]
Description=Pigpio daemon
After=network.target

[Service]
Type=forking
ExecStart=/usr/bin/pigpiod -l
ExecStop=/bin/systemctl kill pigpiod
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd daemon
sudo systemctl daemon-reload

# Enable pigpiod to start on boot
sudo systemctl enable pigpiod

# Start pigpiod now
sudo systemctl start pigpiod

# Check status
echo ""
echo "pigpiod service setup complete!"
echo ""
echo "Status:"
sudo systemctl status pigpiod --no-pager

echo ""
echo "To check status later, run: sudo systemctl status pigpiod"
echo "To stop: sudo systemctl stop pigpiod"
echo "To disable auto-start: sudo systemctl disable pigpiod"
