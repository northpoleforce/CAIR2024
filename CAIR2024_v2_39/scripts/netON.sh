sudo route add default gw 192.168.123.123 dev eth0 metric 0
sudo route del default gw 192.168.123.161 dev eth0 metric 0
sudo route add default gw 192.168.123.161 dev eth0 metric 100
echo "nameserver 114.114.114.114" | sudo tee -a /etc/resolv.conf > /dev/null
ping www.baidu.com