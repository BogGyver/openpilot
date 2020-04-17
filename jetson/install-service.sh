sudo cp /data/openpilot/jetson/openpilot.service /etc/systemd/system/openpilot.service
sudo cp /data/openpilot/jetson/openpilot.logrotate /etc/logrotate.d/openpilot
sudo systemctl daemon-reload
sudo systemctl enable openpilot
sudo systemctl start openpilot
