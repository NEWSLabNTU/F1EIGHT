#!/usr/bin/env bash
set -e
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir"

sudo apt install -y gpsd gpsd-clients libgps-dev
sudo cp -v gpsd /etc/default/gpsd

sudo systemctl enable gpsd
sudo systemctl restart gpsd.service
