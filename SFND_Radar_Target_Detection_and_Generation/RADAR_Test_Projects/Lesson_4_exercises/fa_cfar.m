% T : Number of Training Cells
% 
% G : Number of Guard Cells
% 
% N : Total number of Cells
% 
% Define the number of training cells and guard cells
% Start sliding the window one cell at a time across the complete FFT 1D array. Total window size should be: 2(T+G)+CUT
% For each step, sum the signal (noise) within all the leading or lagging training cells
% Average the sum to determine the noise threshold
% Using an appropriate offset value scale the threshold
% Now, measure the signal in the CUT, which is T+G+1 from the window starting point
% Compare the signal measured in 5 against the threshold measured in 4
% If the level of signal measured in CUT is smaller than the threshold measured, then assign 0 value to the signal within CUT.

% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
Ns = 1000;

% Generate random noise
s=randn(Ns,1);

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

%plot the output
%plot(s);

% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
% 1b. Guard Cells 
T = 3;
G = 1;

% Offset : Adding room above noise threshold for desired SNR 
offset=3;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1:(Ns-(G+T))     
 
    if (i>T+1)
        lagging_t_cells_noise = s(i-(G/2+T-1): i-G/2)/T;
    else
        lagging_t_cells_noise = zeros(1, T);
    end
    % 2. - 5. Determine the noise threshold by measuring it within the training cells

    if (i<Ns-T-1)
        leading_t_cells_noise = s(i+G/2: i+(G/2+T-1))/T;
    else
        leading_t_cells_noise = zeros(1, T);
    end

    %selecting higher noise level as threshold
    if (leading_t_cells_noise > lagging_t_cells_noise)
        selected_noise_threshold = leading_t_cells_noise;
    else
        selected_noise_threshold = lagging_t_cells_noise;
    end
    % noise over T cells
    threshold = sum(selected_noise_threshold, 'all')*offset;
    
    threshold_cfar = [threshold_cfar, {threshold}];
    
    % 6. Measuring the signal within the CUT
    signal = s(i);
    
    % 8. Filter the signal above the threshold
    if signal < threshold
        signal = 0;
    end
    
    signal_cfar = [signal_cfar, {signal}];
end




% plot the filtered signal
% figure, plot (cell2mat(signal_cfar),'g--');
% ylim([0, 15]);
% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
%ylim([0, 15]);
hold on,plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(T+G))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection');