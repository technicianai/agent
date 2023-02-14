#/bin/bash

sudo apt-get -y install \
    neovim \
    tmux \
    git

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo groupadd docker
sudo usermod -aG docker ${USER}
