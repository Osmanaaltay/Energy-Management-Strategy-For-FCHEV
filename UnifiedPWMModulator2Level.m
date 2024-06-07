function [P, D] = UnifiedPWMModulator2Level(Ts, Fsw, Vref, PWMmethod, Pavg, Clk) %#codegen
coder.allowpcode('plain');

%% Unified PWM modulator for a 3-phase 2-Level converter with pulse averaging capability 
%
% The pulses generation can be perform using one of the following Pulse-Width Modulation (PWM) methods:
% 1. Sinusoidal PWM (SPWM)
% 2. Space Vector PWM (SVPWM)
% 3. 30 degrees discontinuous PWM

% Reference: 
% 1) J. S. Kim and S. K. Sul, ÃÂÃÂA novel voltage modulation technique
% of the space vector PWM,ÃÂÃÂ in Conf. Rec. IPECÃÂÃÂ95, Yokohama, Japan, 1995
% 2) D. Chung, J. S. Kim and S. K. Sul, ÃÂÃÂUnified Voltage Modulation Technique
% for Real-Time Three-Phase Power Conversion"
% in IEEE Transaction on Industry ApplicATIONS, October 29, 1997
%
% For pulse averaging, linear interpolation is used:
% If the two known points are given by the coordinates (x0,y0) and (x1,y1),
% the linear interpolant is the straight line between these points.
% For a value x in the interval (x0, x1), the value y along the straight line
% is given from the equation of slopes: (y?y0)/(x?x0) = (y1-y0)/(x1?x0)
% Solving this equation for y, which is the unknown value at x, gives:
% y = ( y0*(x1-x) + y1*(x-x0) ) / (x1-x0)
%
% Pierre Giroux, IREQ, January 2020
% Copyright 2020 Hydro-Quebec
%%
dataType = 'double';
persistent StateOld  CrOld Slope VrefOld TS
if isempty(StateOld)
    StateOld=zeros(3,1,dataType);
    VrefOld=zeros(3,1,dataType);
    TS=Ts*Fsw*4; %  % Carrier value increment in pu of PWM period (1/Fsw)
    %
    Phase=mod(110,360);
    if Phase<180
        Slope=1;
        CrOld=-1+2*(Phase/180) -TS;
    else
        Slope=-1;
        CrOld= (1-2*(Phase-180)/180) + TS;
    end
end
%
D=zeros(3,1,dataType);
Cr=0;        % PWM carrier
State=zeros(3,1,dataType);
P=zeros(6,1,dataType);
DelayOn=zeros(6,1,dataType);
DelayOnP=zeros(3,1,dataType);
DelayOff=zeros(6,1,dataType);
DelayOffP=zeros(3,1,dataType);
TransPM=zeros(1,1,dataType);
TransMP=zeros(1,1,dataType);
Ton=zeros(3,1,dataType);
Toff=zeros(3,1,dataType);
%
Tg=zeros(3,1,dataType);
T_img=zeros(3,1,dataType);

%
%%
Tsamp=1/Fsw/2;      % Half PWM period (s)
%
% Compute imaginary switching time
for n=1:3
    T_img(n)=Tsamp*Vref(n)/2;
end
%
% Determine Tmin & Tmax
Tmax=max(T_img);
Tmin=min(T_img);
%
% Effective time calculation
Teff=Tmax-Tmin;
Tzero=Tsamp-Teff;
%% Compute Toffset
switch PWMmethod
    case 1     % SPWM
        Toffset=Tsamp/2;
    case 2     % SPWM with pulse averaging
        Toffset=Tsamp/2;
    case 3     % SVPWM
        Toffset=Tzero/2 - Tmin;
    case 4     % SVPWM with pulse averaging
        Toffset=Tzero/2 - Tmin;
    case 5     % 30%  Discontinous modulation
        if (Tmin+Tmax) >= 0
            Toffset = -Tmin;
        else
            Toffset = Tsamp-Tmax;
        end
end
%
% Time shift
for n=1:3
    Tg(n)=(T_img(n) + Toffset);
    D(n)=Tg(n)*Fsw*2;
    Vref(n)=2*D(n)-1;
end
if Pavg==0
 % Gate timing calculation
for n=1:3
    Ton(n)=Tsamp-Tg(n);
    if Ton(n)<0     % Ton not negative
        Ton(n)=0;
    end
    if Ton(n)> Tsamp    % Ton not greater than Tsamp
        Ton(n)=Tsamp;
    end
    Toff(n)=Tg(n)+Tsamp;
    if Toff(n)>(2*Tsamp)    % Toff not to exceed PWM period
        Toff(n)=(2*Tsamp);
    end
    if Toff(n)<Tsamp     % Ton not smaller than Tsamp
        Toff(n)=Tsamp;
    end
end

%
%% Timing generation
time=rem(Clk,Tsamp*2);
%
%% Pulses generation
for n=1:3
    if time>=Ton(n) && Toff(n)>time
        P(n)=1;
    else
        P(n)=0;
    end
end
%
%% Generate complementary pulses
P(5)=P(3);
P(3)=P(2);
P(2)=1-P(1);
P(4)=1-P(3);
P(6)=1-P(5);   
%%
else
    %% Generate the PWM carrier & Determine Slope Transition:
    % Slope from Plus-to-Minus (TransPM=1) or Slope from Minus-to-Plus (TransMP=1)
    if Slope==1
        Cr=CrOld + TS;   % Plus carrier
        if Cr>1
            Cr=2-Cr;
            Slope=-1;
            TransPM=1;
        end
    elseif Slope==-1
        Cr=CrOld - TS;
        if Cr<-1
            Cr=-2-Cr;
            Slope=1;
            TransMP=1;
        end
    end
    % 
    %% Determine States
    %
    for n=1:3
        if Vref(n)>=Cr
            State(n)=1;         % P(1)=1; P(2)=0;
        else
            State(n)=0;         % P(1)=0; P(2)=1;
        end
    end
    %
    %% Compute delays ON and delay OFF and average pulse values
    for n=1:3
        if (StateOld(n)==1 && State(n)==0)
            if (TransPM==0 && TransMP==0)   % No transition
                DelayOnP(n)=0;
                DelayOffP(n)=Ts-((VrefOld(n)-CrOld)*Ts /(Cr-CrOld+VrefOld(n)-Vref(n)));
            elseif (TransPM==1)
                DelayOnP(n)=0;
                DelayOffP(n)=Ts-((VrefOld(n)-CrOld)*Ts /( (CrOld+TS)-CrOld+VrefOld(n)-Vref(n)) );
            else % TransMP=1
                DelayOnP(n)=0;
                DelayOffP(n)=Ts-((VrefOld(n)-(Cr-TS))*Ts /(Cr-(Cr-TS)+VrefOld(n)-Vref(n)));
            end
            P(1 + 2*(n-1))=(Ts-DelayOffP(n))/Ts;
            P(2 + 2*(n-1))=1-P(1 + 2*(n-1));
        elseif (StateOld(n)==0 && State(n)==1)
            if (TransPM==0 && TransMP==0)
                DelayOnP(n)=Ts-((VrefOld(n)-CrOld)*Ts /(Cr-CrOld+VrefOld(n)-Vref(n)));
                DelayOffP(n)=0;
            elseif (TransPM==1)
                DelayOnP(n)=Ts-((VrefOld(n)-(Cr+TS))*Ts /(Cr-(Cr+TS)+VrefOld(n)-Vref(n)));
                DelayOffP(n)=0;
            else % TransMP=1
                DelayOnP(n)=Ts-((VrefOld(n)-CrOld)*Ts /((CrOld-TS)-CrOld+VrefOld(n)-Vref(n)));
                DelayOffP(n)=0;
            end
            P(1 + 2*(n-1))=DelayOnP(n)/Ts;
            P(2 + 2*(n-1))=1-P(1 + 2*(n-1));
            %% Pulse value variation from ON/OFF/ON  or OFF/ON/OFF with the same sample time
        elseif ((StateOld(n)==1 && State(n)==1) && (TransPM==1) && (VrefOld(n)~=1.0))
            DelayOffP(n)=Ts-((VrefOld(n)-CrOld)*Ts /( (CrOld+TS)-CrOld+VrefOld(n)-Vref(n)) );
            DelayOnP(n)=Ts-((VrefOld(n)-(Cr+TS))*Ts /(Cr-(Cr+TS)+VrefOld(n)-Vref(n)));
            P(1 + 2*(n-1))=1-(DelayOffP(n)-DelayOnP(n))/Ts;
            P(2 + 2*(n-1))=1-P(1 + 2*(n-1));
        elseif ((StateOld(n)==0 && State(n)==0) && (TransMP==1) && (VrefOld(n)~=-1))
            DelayOnP(n)=Ts-((VrefOld(n)-CrOld)*Ts /((CrOld-TS)-CrOld+VrefOld(n)-Vref(n)));
            DelayOffP(n)=Ts-((VrefOld(n)-(Cr-TS))*Ts /(Cr-(Cr-TS)+VrefOld(n)-Vref(n)));
            P(1 + 2*(n-1))=(DelayOnP(n)-DelayOffP(n))/Ts;
            P(2 + 2*(n-1))=1-P(1 + 2*(n-1));
        else
            DelayOnP(n)=0;
            DelayOffP(n)=0;
            if (State(n)==1)
                P(1 + 2*(n-1))=1;
                P(2 + 2*(n-1))=0;
            else
                P(1 + 2*(n-1))=0;
                P(2 + 2*(n-1))=1;
            end
        end
    end
    %
    %% Determine value for all delays On and delays Off
    for n=1:3
        DelayOn(1+2*(n-1))=DelayOnP(n);
        DelayOn(2+2*(n-1))=DelayOffP(n);
        DelayOff(1+2*(n-1))=DelayOffP(n);
        DelayOff(2+2*(n-1))=DelayOnP(n);
    end
    
    %
    %% Refresh old values
    %
    for n=1:3
        StateOld(n)=State(n);
        VrefOld(n)=Vref(n);
    end
    CrOld=Cr;
end