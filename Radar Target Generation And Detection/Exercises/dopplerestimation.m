% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz

% TODO: Calculate the wavelength
lambda = c/frequency;

% TODO: Define the doppler shifts in Hz using the information from above 
doppler_shifts = [3e3 -4.5e3 11e3 -3e3];

% TODO: Calculate the velocity of the targets  fd = 2*vr/lambda
calculated_velocity = doppler_shifts*lambda/2;

dist_delta = calculated_velocity*5;

final_distance = 200 - dist_delta;

% TODO: Display results
disp(final_distance);