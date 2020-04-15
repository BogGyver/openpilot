sudo cp /data/openpilot/jetson/openpilot.service /etc/systemd/system/openpilot.service
sudo systemctl daemon-reload
sudo systemctl enable openpilot
sudo systemctl start openpilot
