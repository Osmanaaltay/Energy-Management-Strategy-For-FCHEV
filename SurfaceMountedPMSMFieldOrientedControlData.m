%% Parameters file for SPS model: SurfaceMountedPMSMFieldOrientedControl.slx 

%% SPS sample time (s)
Ts=20e-6;    

%% Motor parameters
%
Rs = 0.2;          % Stator resistance per phase         (Ohm)
La = 2.057e-3;     % Armature inductance                 (H)
lambda = 0.175;    % Flux linkage established by magnets (V.s)
p = 3;             % Number of pole pairs
J = 0.01;          % Rotor inertia                       (Kg.m^2)
F = 0.005;         % Friction coefficient                (N.m.s)
Vdc_nom = 400;     % Nominal DC voltage (V)
%
%% Control parameters
%
% Speed regulator
Kp_wreg=0.25;        % Proportional gain
Ki_wreg=25;          % Integral gain
Limit_wreg=20;       % Regulator output limit
%
% Current regulator
Kp_Ireg=0.2;           % Proportional gain
Ki_Ireg=10;            % Integral gain
Limit_Ireg=2/sqrt(3);  % Regulator output limit
%
Fsw=8000;              % SVPWM switching frequency (Hz)
%
%%

