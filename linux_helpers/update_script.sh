echo "Das hier ist das UpdateSkript. Hier werden alle Temporären Daten gelöscht und das Hintergrundbild gesetzt. Version: 06.07.2022"

rm -rf ~/.mozilla
cd ~/ros_ws/src/MesstechnikPraktikumV6/linux_helpers/
./install_icons.sh
cd ~/Downloads/; rm -rf * 

cp ~/ros_ws/src/MesstechnikPraktikumV6/hardware/helene_background.png /usr/share/backgrounds/helene_background.png
gsettings set org.gnome.desktop.background picture-uri
usr/share/backgrounds/helene_background.png


echo "Fertig :)"
