echo "Hiermit werden die Verküpfungen zum Starten von Helene hinzugefügt. Das hier ist allerdings alles für das Messtechnik Praktikum gedacht und ist mega spezifisch"
mkdir -p ~/.local/share/icons
mkdir -p ~/.local/share/applications

cp messtechnik_praktikum_sim.png ~/.local/share/icons/messtechnik_praktikum_sim.png
cp messtechnik_praktikum_real_robot.png ~/.local/share/icons/messtechnik_praktikum_real_robot.png

cp messtechnik_praktikum_sim.desktop ~/.local/share/applications/messtechnik_praktikum_sim.desktop
cp messtechnik_praktikum_real_robot.desktop ~/.local/share/applications/messtechnik_praktikum_real_robot.desktop

echo "Fertig :)"
