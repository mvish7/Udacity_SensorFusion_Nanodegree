%%%%%%%% TODOs to calculate the range in meters of four targets with respective 
%measured beat frequencies [0 MHz, 1.1 MHz, 13 MHz, 24 MHz]

% You can use the following parameter values:
% 
% The radar maximum range = 300m
% The range resolution = 1m
% The speed of light c = 3*10^8

max_range = 300;
d_res = 1;
c = 3 * 10^8;

% TODO : Find the Bsweep of chirp for 1 m resolution
%%%%%%% d_res = c/(2*b_sweep)

b_sweep = c / (2 * d_res);

% TODO : Calculate the chirp time based on the Radar's Max Range
%%%%%%% The sweep time can be computed based on the time needed for the signal
% to travel the maximum range. 
% In general, for an FMCW radar system, the sweep time should be at least 5 to 6 
% times the round trip time. This example uses a factor of 5.5:

t_chirp = 5.5 * 2 * max_range / c; 


% TODO : define the frequency shifts

beat_freq = [0 , 1.1 * 10^6 , 13 * 10^6 , 24 * 10^6];

for b_f = beat_freq

    range = (c * t_chirp * b_f) / (2 * b_sweep);
    calculated_range = ['Calculated range is ', num2str(range)];
    % Display the calculated range
    disp(calculated_range);
end