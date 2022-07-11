echo "Das hier ist das Update-Skript. Hier werden alle Temporären Daten gelöscht und das Hintergrundbild gesetzt. Version: 06.07.2022"

rm -rf ~/.mozilla
cd ~/ros_ws/src/MesstechnikPraktikumV6/linux_helpers/
./install_icons.sh
cd ~/Downloads/; rm -rf * 
cd ~/Schreibtisch/; rm -rf * 

gsettings set org.gnome.desktop.background picture-uri file:////home/pi/ros_ws/src/MesstechnikPraktikumV6/hardware/helene_background.png

echo "Fertig :), Viel Spaß im Praktikum!"
