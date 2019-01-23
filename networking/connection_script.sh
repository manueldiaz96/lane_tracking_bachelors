green='\033[1;32m'
reset='\033[0m'
RED='\033[1;31m'

echo 

printf "${green}CONNECTION SCRIPT ${reset} \n"

echo 
echo -------------------------------------------
echo

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
printf "${RED}Remember to enable ufw!${reset}\n"

roscore && sudo ufw enable