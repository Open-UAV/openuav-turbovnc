#!/bin/bash

apt-get update -y
apt-get install -y emacs lsof wget curl git htop less build-essential terminator make cmake net-tools lubuntu-desktop xvfb
dpkg --add-architecture i386
apt update -y
apt install libc6:i386 -y
