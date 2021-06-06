# pi_hw_interface

# Setup
This package comes with a systemd service which allows for the automatic configuration of the canbus after the network initalization step is complete. This file will bring the can bus into a ready state.

In order to use this service, run the following commands
1. Copy the `can_init.service` file from this directory into `/etc/systemd/system`
2. run `sudo systemctl daemon-reload` to reload the systemd service list
3. run `sudo systemctl enable can_init` to enable the service to run at boot

