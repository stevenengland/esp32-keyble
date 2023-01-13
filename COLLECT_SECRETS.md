# Linux

Using a live linux or WSL are pretty fine.

## Install the base system

```bash
#!/bin/bash
# (Optional, but recommended) Fully update/upgrade system
sudo apt-get -y update
# Install Node.js LTS
curl -sL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install --upgrade -y build-essential nodejs
# Make sure required libraries for Bluetooth are installed
sudo apt-get -y install bluetooth bluez libbluetooth-dev libudev-dev
# Install keyble
sudo npm install --update --global --unsafe-perm keyble
# (Optional, but recommended) Allow keyble to be run without sudo
sudo setcap cap_net_raw+eip $(eval readlink -f `which node`)
# (Optional, but recommended) Install tools for controlling via MQTT
sudo apt-get -y install mosquitto-clients
# From <https://github.com/oyooyo/keyble>
```

## Get and set important information from the smart lock

```bash
# Register a user with the content of the qr code (M0123...)
keyble-registeruser -n John -q M0123456789ABK0123456789ABCDEF0123456789ABCDEFNEQ1234567

# Have a look at the output and watch for user id and key
Press and hold "Unlock" button until the yellow light flashes in order to enter pairing mode
Registering user on Smart Lock with address "01:23:56:67:89:ab", card key "0123456789abcdef0123456789abcdef" and serial "NEQ1234567"...
User registered. Use arguments: --address 01:23:56:67:89:ab --user_id 1 --user_key ca78ad9b96131414359e5e7cecfd7f9e
Setting user name to "John"...
User name changed, finished registering user.
```
