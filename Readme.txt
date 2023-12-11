voir ce site pour ajouter les droits sur les ports https://arduino.stackexchange.com/questions/21215/first-time-set-up-permission-denied-to-usb-port-ubuntu-14-04
pour activer le port ttySO (uart du rpi), ajouter dans le fichier /boot/firmware/config.txt à la fin enable_uart=1
activer hotspot
lancer les bonnes commande dans /etc/rc.local
Pour voir les erreurs dans rc.local -> systemctl status rc-local.service
installer miniconda :https://github.com/conda-forge/miniforge/#download

Pour installer open3d sur ubuntu 20.10 arm: http://www.open3d.org/docs/latest/arm.html
sudo apt-get update -y
sudo apt-get install -y apt-utils build-essential git cmake
sudo apt-get install -y python3 python3-dev python3-pip
sudo apt-get install -y xorg-dev libglu1-mesa-dev
sudo apt-get install -y libblas-dev liblapack-dev liblapacke-dev
sudo apt-get install -y libsdl2-dev libc++-7-dev libc++abi-7-dev libxi-dev
sudo apt-get install -y clang-7
pip install cmake
# Clone
git clone --recursive https://github.com/isl-org/Open3D
cd Open3D
git submodule update --init --recursive
mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_CUDA_MODULE=OFF -DBUILD_GUI=OFF -DBUILD_TENSORFLOW_OPS=OFF -DBUILD_PYTORCH_OPS=OFF -DBUILD_UNIT_TESTS=ON -DCMAKE_INSTALL_PREFIX=~/open3d_install -DPYTHON_EXECUTABLE=$(which python3) -DUSE_BLAS=ON -DBUILD_FILAMENT_FROM_SOURCE=ON -DCMAKE_CXX_FLAGS_RELEASE=ON -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG -faligned-new"  \    ..
make -j 1 (essayer sans ça)
make install-pip-package -j 1
