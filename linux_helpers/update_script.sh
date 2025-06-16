echo "Das hier ist das Update-Skript. Hier werden alle Temporären Daten gelöscht und das Hintergrundbild gesetzt. Version: 06.05.2024"

rm -rf ~/.mozilla
cd ~/ros_ws/src/MesstechnikPraktikumV6/linux_helpers/
./install_icons.sh
cd ~/Downloads/; rm -rf * 
cd ~/Schreibtisch/; rm -rf * 

#gsettings set org.gnome.desktop.background picture-uri file:////home/pi/ros_ws/src/MesstechnikPraktikumV4/hardware/helene_background_eval2024.png
gsettings set org.gnome.desktop.background picture-uri file:///usr/share/backgrounds/warty-final-ubuntu.png

echo "Fertig :), Viel Spaß im Praktikum!"
