#!/bin/bash

#added by ZWL

echo "enable wlan2"
sudo ifconfig wlan2 up
echo "route add 192.168.31.1 metric 0"
sudo route add default gw 192.168.31.1 dev wlan2 metric 0
echo 'nameserver 8.8.8.8' | sudo tee -a /etc/resolv.conf

sudo iptables -t nat -A POSTROUTING -o wlan2 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan2 -j ACCEPT
sudo iptables -A FORWARD -m state --state ESTABLISHED,RELATED -j ACCEPT