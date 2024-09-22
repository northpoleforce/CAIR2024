clear

scp -r northpoleforce@192.168.123.123:~/CAIR2024/CAIR2024_v2_39/include ~/CAIR2024/CAIR2024_v2_39
scp -r northpoleforce@192.168.123.123:~/CAIR2024/CAIR2024_v2_39/src ~/CAIR2024/CAIR2024_v2_39
scp -r northpoleforce@192.168.123.123:~/CAIR2024/CAIR2024_v2_39/CMakeLists.txt ~/CAIR2024/CAIR2024_v2_39

cd ~/CAIR2024/CAIR2024_v2_39/build
make -j