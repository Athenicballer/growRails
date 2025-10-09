cd grower
rm -rf build
mkdir build
cd build

# Use the toolchain file as before
# cmake -DCMAKE_TOOLCHAIN_FILE=/home/user/rpi-toolchain.cmake .. 
cmake -DCMAKE_TOOLCHAIN_FILE=/home/user/source/rpi-toolchain.cmake .. 

make