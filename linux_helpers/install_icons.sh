echo "Hiermit werden die Verküpfungen zum Starten von Helene hinzugefügt. Das hier ist allerdings alles für das Messtechnik Praktikum gedacht und ist mega spezifisch"
mkdir -p ~/.local/share/icons
mkdir -p ~/.local/share/applications

cp messtechnik_praktikum_sim.png ~/.local/share/icons/messtechnik_praktikum_sim.png
cp messtechnik_praktikum_real_robot.png ~/.local/share/icons/messtechnik_praktikum_real_robot.png
cp SimOhneNadel.png ~/.local/share/icons/SimOhneNadel.png
cp Icon_Log_Norm_MT.png ~/.local/share/icons/Icon_Log_Norm_MT.png
cp USB_Icon.png ~/.local/share/icons/USB_Icon.png
cp reset_icon.png ~/.local/share/icons/reset_icon.png

cp messtechnik_praktikum_sim.desktop ~/.local/share/applications/messtechnik_praktikum_sim.desktop
cp messtechnik_praktikum_real_robot.desktop ~/.local/share/applications/messtechnik_praktikum_real_robot.desktop
cp MesstechnikPrakt_helene_sim_wo_nadel.desktop ~/.local/share/applications/MesstechnikPrakt_helene_sim_wo_nadel.desktop
cp Start_Small_DataLoggerMT.desktop ~/.local/share/applications/Start_Small_DataLoggerMT.desktop
cp Connect_USB_realHelene.desktop ~/.local/share/applications/Connect_USB_realHelene.desktop
cp MTP_ResetEverything.desktop ~/.local/share/applications/MTP_ResetEverything.desktop


echo "Fertig :)"
