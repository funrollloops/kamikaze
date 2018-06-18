#!/bin/bash

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root"
  exit 1
fi

if grep -q 92xx reverse-ssh-proxy.service; then
  echo 'Replace 92xx with a port in reverse-ssh-proxy.service.'
  exit 1
fi

apt install autossh

NEW_USER=autossh
adduser $NEW_USER \
  --disabled-password --disabled-login
usermod -G "" $NEW_USER

KEY_FILE=/home/$NEW_USER/.ssh/id_ed25519
mkdir -p /home/$NEW_USER/.ssh
if [ ! -e $KEY_FILE ]; then
  ssh-keygen -o -a 100 -t ed25519 -N "" -f $KEY_FILE
fi
chown autossh:autossh -R /home/autossh
chmod -R 0500 /home/autossh

systemctl enable $(readlink -f reverse-ssh-proxy.service)

echo "public key: ${KEY_FILE}.pub"
cat ${KEY_FILE}.pub
echo "add above public key to https://console.cloud.google.com/compute/instancesDetail/zones/us-west1-a/instances/main?project=nebree8"
echo "you might have to log into the instance and add the authorized key to /home/autossh/.ssh/known_hosts manually"
