%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Pt = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
Ps = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%TODO: Calculate the wavelength

lambda = c / (fc);
wave_len = ['Wavelength is ', num2str(lambda)];
disp(wave_len);


%TODO : Measure the Maximum Range a Radar can see.

max_len_numerator = Pt * G^2 * lambda^2 * RCS;
max_len_denominator = Ps * (4*pi)^3;

max_len = (max_len_numerator/max_len_denominator)^0.25;
max_len_str = ['Maximum range RADAR can see is ', num2str(max_len)];
disp(max_len_str);