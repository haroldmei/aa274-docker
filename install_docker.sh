# # Update the apt package index
# sudo apt-get update

# # Install packages to allow apt to use a repository over HTTPS
# sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common

# # Add Docker’s official GPG key
# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

# # Set up the stable repository
# sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

# # Update the apt package index
# sudo apt-get update

# # Install the latest version of Docker CE and containerd
# sudo apt-get install docker-ce docker-ce-cli containerd.io

# Install CUDA
#!/bin/bash
echo "Checking for CUDA and installing."
# Check for CUDA and try to install.
if ! dpkg-query -W cuda-10-0; then
	curl -O http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
	sudo dpkg -i ./cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
	sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
	sudo apt-get update
	sudo apt-get install cuda-10-0 -y
fi
# Enable persistence mode
sudo nvidia-smi -pm 1

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
rm get-docker.sh

# Install Nvidia support
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
	sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu16.04/amd64/nvidia-docker.list | \
	sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo pkill -SIGHUP dockerd
docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
docker network create --driver bridge aa274_net

# Create Docker group to avoid need for sudo
sudo groupadd docker
sudo usermod -aG docker $USER

echo "\nDocker successfully installed. Please restart the computer for changes to take effect.\n"
