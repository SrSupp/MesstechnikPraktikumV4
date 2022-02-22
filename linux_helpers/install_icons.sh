echo "Hiermit werden die Verküpfungen zum Starten von Helene hinzugefügt"
mkdir -p ~/.local/share/icons
mkdir -p ~/.local/share/applications

cp messtechnik_praktikum_sim.png ~/.local/share/icons/messtechnik_praktikum_sim.png
cp messtechnik_praktikum_real_robot.png ~/.local/share/icons/messtechnik_praktikum_real_robot.png
cp Student.png ~/.local/share/icons/Student.png

cp messtechnik_praktikum_sim.desktop ~/.local/share/applications/messtechnik_praktikum_sim.desktop
cp messtechnik_praktikum_real_robot.desktop ~/.local/share/applications/messtechnik_praktikum_real_robot.desktop
cp student.desktop ~/.local/share/applications/student.desktop

echo "Fertig :)"