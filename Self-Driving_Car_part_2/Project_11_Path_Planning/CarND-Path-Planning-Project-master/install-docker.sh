#! /bin/bash
apt-get install libuv1-dev libssl-dev libz-dev git -y
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make -j4
make install
cd ..
cd ..
ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
rm -r uWebSockets
