#!/bin/bash

GPU=$(lspci | grep ' NVIDIA ' | grep -Po '\[\K[^]]*')
if [[ $GPU == "Tesla K80" ]]; then
  echo $GPU
  wget https://us.download.nvidia.com/tesla/440.118.02/NVIDIA-Linux-x86_64-440.118.02.run
  chmod a+x NVIDIA-Linux-x86_64-440.118.02.run
  ./NVIDIA-Linux-x86_64-440.118.02.run -s --install-libglvnd
elif [[ $GPU == "Tesla P100" ]]; then
  echo $GPU
  wget https://us.download.nvidia.com/tesla/440.118.02/NVIDIA-Linux-x86_64-440.118.02.run
  chmod a+x NVIDIA-Linux-x86_64-440.118.02.run
  ./NVIDIA-Linux-x86_64-440.118.02.run -s --install-libglvnd
elif [[ $GPU == "Tesla P4" ]]; then
  echo $GPU
  wget https://us.download.nvidia.com/tesla/460.32.03/NVIDIA-Linux-x86_64-460.32.03.run
  chmod a+x NVIDIA-Linux-x86_64-460.32.03.run
  ./NVIDIA-Linux-x86_64-460.32.03.run -s --install-libglvnd
else
  echo $GPU
  echo -n "Do you want to install NVIDIA drivers (Don't do it, if you already have drivers) (y/n)? "
  read answer
  if [[ "$answer" != "${answer#[Yy]}" ]] ;then
    wget http://us.download.nvidia.com/XFree86/Linux-x86_64/384.111/NVIDIA-Linux-x86_64-384.111.run
    chmod a+x NVIDIA-Linux-x86_64-384.111.run
    ./NVIDIA-Linux-x86_64-384.111.run -s --install-libglvnd
  else
    echo "Not installing"
  fi
fi
