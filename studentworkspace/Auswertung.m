clc;clear;close all;
pkg load signal;
#Finde den neusten Datensatz einer Helene und lade ihn ein:
d = dir('*csv');
[~, index] = max([d.datenum]);
youngestFile = fullfile(d(index).folder, d(index).name)
Input = csvread(youngestFile);
#in der Csv-Datei steht erst die Zeit in s, dann x y z Koordinate in m, gefolgt von der Kraft in mv/v und dann dem reserved-Topic

#Lösche alles in Input, das nichts mit dem Einstich zu tun hat
Input_Cut = Input(Input(:,6)>0.5,1:5);

#Rechne die Kräfte von mV/V in N um
Input_Cut(:,5) =  Input_Cut(:,5) - mean(Input(10:50,5));
if abs(max(Input_Cut(:,5))) > abs(min(Input_Cut(:,5))) #Autodetect, wie rum die Messbruecke angeschlossen ist
    Input_Cut(:,5) = Input_Cut(:,5)*50;
else
    Input_Cut(:,5) = Input_Cut(:,5)*-50;
end

#Rechne den Z-Offset um
Input_Cut(:,4) = (Input_Cut(:,4) - Input_Cut(2,4))*-1000;
Input_Filter = Input_Cut;

#Wirf einen Tiefpassfilter über x,y und z, da die Signale des Roboterarms leicht rauschen. 
#Input_Filter(:,2:4) = lowpass(Input_Cut(:,2:4),4,50);
fc = 10; #Cutoff Frequenz von 5 Hz
fs = 1/(Input_Cut(2,1) - Input_Cut(1,1)); 
w = fc/(fs/2);
[b,a]=butter (2, w); #Butterworth-Filter 2. Ordnung zusammenbasteln
Input_Filter(:,2:4) = filter(b,a,Input_Cut(:,2:4));

#Plot
figure
plot(Input_Filter(:,4),Input_Filter(:,5))
xlabel("Einstichtiefe (mm)")
ylabel("Kraft in Z-Richtung (N)")