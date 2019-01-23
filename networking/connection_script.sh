sudo ufw disable

echo 
echo -------------------------------------------
echo

sudo arp-scan --localnet

echo 
echo -------------------------------------------
echo

export ROS_IP=$(hostname -I)

echo Everything ready. $ROS_IP as ROS_IP
echo Remember to enable ufw!