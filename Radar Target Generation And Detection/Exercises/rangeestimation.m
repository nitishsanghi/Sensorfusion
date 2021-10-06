% TODO : Find the Bsweep of chirp for 1 m resolution
Bsweep = 3*10^8/(1*2);


% TODO : Calculate the chirp time based on the Radar's Max Range
Tchirp = 5.5*2*300/(3*10^8);

% TODO : define the frequency shifts 
beats = [0 1.1e6 13e6 24e6];
calculated_range = 3*10^8*Tchirp*beats/(2*Bsweep);
% Display the calculated range
disp(calculated_range);