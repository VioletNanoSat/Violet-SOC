% * State-of-Charge Algorithm for Batteries
% Uses a mixture of colomb counting and direct voltage mapping to estimate
% the state of charge of the battery.

clc
clear

%% Battery Parameters

% MaxCharge is the coulomb count of the battery. We estimate it in terms
% of current * time (seconds). Based upon Violet Satellite Testing
% Charge-Up of 2.2A * Seconds (estimated) - 

% MaxCharge = 2.2;
MaxCharge = 17275 * 2.2; % 2.2A * 17275 seconds
%MaxCharge = 100;





%% Simulation

% simulation variables

desired_iterations = 1800;
current_iteration = 1;
% current_scale_factor = 0.01;
current_scale_factor = 1;
sample_time_interval = 0.2;

% current input characteristics
current_sample = zeros(20,1);

% SOCv is the state of charge derived from voltage measurements. Since that
% is outside the scope of the simulation to obtain voltages for that, we
% will preset it.
SOCv = 75;

accumulated_current = SOCv / 100 * MaxCharge;

while current_iteration < desired_iterations
    
    current_sample_total = 0;
    for j = 1:20
        current_sample(j) = (rand(1)) * current_scale_factor;
        current_sample_total = current_sample_total + current_sample(j);
    end
    
    current_sample_total = current_sample_total * sample_time_interval
    accumulated_current = accumulated_current + current_sample_total;
    
    if accumulated_current < 0
        accumulated_current = 0;
    elseif accumulated_current > MaxCharge
        accumulated_current = MaxCharge;
    end
    
    SOCc = accumulated_current / MaxCharge * 100;
    weighted_soc = combine_soc(SOCv, SOCc);
    
    current_iteration = current_iteration + 1;

end





