ip a
sudo apt update && sudo apt upgrade -y
ps aux | grep apt
sudo tail -n 20 /var/log/dpkg.log
ps aux | grep apt
sudo apt install ubuntu-desktop-minimal -y
ps aux | grep apt
exit
ps aux | grep apt
exit
ps aux | grep apt
exit
ps aux | grep apt
sudo iwconfig wlan0 power off
echo "wireless-power off" | sudo tee -a /etc/network/interfaces
exexit
ip a
sudo reboot
sudo apt update
sudo apt install tigervnc-standalone-server tigervnc-common -y
nano ~/.vnc/xstartup
clear
ls ~/.vnc
nano ~/.vnc/xstartup
mkdir -p ~/.vnc
nano ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup
vncserver :1
vncserver -list
ps aux | grep vnc
vncserver -list
vncserver :1
which gnome-session
vncserver -kill :1 2>/dev/null || true
vncserver -list
nano ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup
vncserver :1
vncserver -list
vncserver -kill :1 2>/dev/null || true
tigervncserver -xstartup /usr/bin/xterm :1
vncserver -list 
vncserver :1
vncserver -list
ss -tlnp | grep 5901
vncserver -kill :1
vncserver -list
vncserver :1 -localhost no
ss -tlnp | grep 5901
sudo nano /etc/systemd/system/vncserver@.service
sudo systemctl daemon-reload
sudo systemctl enable vncserver@1.service
sudo systemctl start vncserver@1.service
sudo systemctl status vncserver@1.service
ss -tlnp | grep 5901
vncserver -kill :1
vncpasswd
vncserver :1 -localhost no
sudo reboot
ubuntu --version
sudo apt install terminator
sudo apt install htop
sudo apt update
sudo apt install zsh
chsh -s $(which zsh)
echo $SHELL
sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"
echo $SHELL
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k"
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
source ~/.zshrc
sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"
