mkdir -p ~/build
 ros2 launch mybot_cartographer cartographer.launch.py use_sim_time:=True
 
 ----------------------------------------
 
 lsusb


 <img width="739" height="287" alt="image" src="https://github.com/user-attachments/assets/50063618-dc8b-44e6-b64e-1ab1655b53b0" />

 
 git clone https://github.com/brektrou/rtl8821CU.git
 
 cd rtl8821CU/
 
 -------------------

 make
 
 sudo apt update

 ---------------------------
  
 sudo apt install build-essential

 sudo apt install dkms
 
 sudo ./dkms-install.sh
 
 sudo modprobe 8821cu


