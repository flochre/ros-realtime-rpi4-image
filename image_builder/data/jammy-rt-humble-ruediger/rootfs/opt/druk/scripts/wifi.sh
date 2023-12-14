#!/usr/bin/env bash

mac_addr=$(echo $(cat /sys/class/net/wlan0/address) | sed 's/\://g');
ssid="Druk-"${mac_addr:6};
password="12345678";

# Create netplan .yaml file
echo -e "network:\n\
    version: 2\n\
    renderer: NetworkManager\n\
    wifis:\n\
        wlan0:\n\
            dhcp4: true\n\
            addresses: [10.9.75.37/24]\n\
            optional: true\n\
            access-points:\n\
                "$ssid":\n\
                    password: "$password"\n\
                    # band: 5GHz\n\
                    mode: ap\n\
            routes:\n\
                - to: default\n\
                  via: 10.9.75.37\n\
" | sudo tee /etc/netplan/10-druk-wifi.yaml

sudo netplan generate
sudo netplan apply