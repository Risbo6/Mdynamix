#!/bin/bash
echo -n "Entrer l'IP du véhicule: "
read IPvehicule

echo -n "Entrer l'IP de la workstation: "
read IPworkstation
sudo sed -i '$ d' /etc/hosts
echo "$IPvehicule    jetson4g" >> /etc/hosts 

export ROS_IP=$IPworkstation
export ROS_MASTER_URI=http://jetson4g:11311
