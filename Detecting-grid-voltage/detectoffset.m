% This sample to calculate RMS was finded in this book bellow
% Signal Processing for Intelligent Sensor Systems with MATLAB®, Second Edition
% Page 374
% And in this article http://www.eetindia.co.in/ARTICLES/2006FEB/PDF/EEIOL_2006FEB04_EMS_TA.pdf?SOURCES=DOWNLOAD

clear all;
close all;
clc;

%PLL Modelling starts from here
Fs=1000; %Sampling frequency = 50Khz
GridFreq=60; %Nominal Grid Frequency in Hz
Tfinal=3.5; %Time the simulation is run for = 0.5 seconds
Ts=1/Fs; %Sampling Time = 1/Fs
t=0:Ts:Tfinal; %Simulation Time vector
wn=2*pi*GridFreq; %Nominal Grid Frequency in radians

%RMS constants
N = 30;
alpha = (N-1)/N;
beta = 1/N;

%Mean constants
ta = 0.1; %Tempo de acomodacao
a = exp(-1/(Fs*ta));

Vrms = 220;
Vp = Vrms*sqrt(2);

L=length(t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     SIN WAVE                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for n=1:floor(L/4)
    u(n) = Vp*sin(2*pi*GridFreq*Ts*n);
end
for n=floor(L/4):floor(2*L/4)
    u(n) = 2*Vp*sin(2*pi*GridFreq*Ts*n);
end
for n=floor(2*L/4):floor(3*L/4)
    u(n) = 0.5*Vp*sin(2*pi*GridFreq*Ts*n);
end
for n=floor(3*L/4):floor(L)
    u(n) = 3*Vp*sin(2*pi*GridFreq*Ts*n);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     TRINAG. WAVE                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% x = sawtooth (2 * pi * GridFreq * t , 0.5);
% for n=1:floor(L)    
%     u(n) = V*x(n);
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     STEP    WAVE                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for n=1:floor(L/3)    
%     u(n) = 0;
% end
% 
% for n=floor(L/3):floor(2*L/3)
%     u(n) = V;
% end
% 
% for n=floor(2*L/3):floor(L)
%     u(n) = 0;
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     STEP    WAVE + NOISE                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for n=1:floor(L/3)    
%     u(n) = 0;
% end
% 
% for n=floor(L/3):floor(2*L/3)
%     u(n) = V;
% end
% 
% for n=floor(2*L/3):floor(L)
%     u(n) = 0;
% end
% 
% u = awgn(u,40,'measured');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  CONVERTO TO ADC COUNTS                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R1 = 2200000;
R2 = 15e3;
R3 = 15e3; %Resistor to generate offset
GAIN = R2/(R1+R2);

ADC_BITS = 10;
ADC_COUNTS = 2^10;
SupplyVoltage = 5.0;
OffSet = (R3/(R2+R3))*SupplyVoltage;

Vout = GAIN.*u; % After resistive network R1 & R2
VoutOffset = (Vout+OffSet); % Plus offset generate by R3 & R4
adc = floor((((VoutOffset/SupplyVoltage))*ADC_COUNTS)); %adc Counts

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     RMS START HERE                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

VCAL_M = 147.7440649211;
VCAL_B = -0.0106438008;
V_RATIO = ((VCAL_M * ((SupplyVoltage) / (ADC_COUNTS))));

Vrms = 0;

% Enable Noise
%u = awgn(u,50,'measured');
offsetV = floor((((OffSet/SupplyVoltage))*ADC_COUNTS));
filteredV = 0;

rmsOut =[0,0];
rmsIn = 0;

meanOut =[0,0];
meanIn = 0;

Plot_Var1=[0,0];
Plot_Var2=[0,0];

for n=2:Tfinal/Ts % Calculate RMS
    
    offsetV = offsetV + ((adc(n) - offsetV)/1024);
    filteredV = adc(n) - offsetV;
    rmsIn = filteredV;

    rmsOut(1) = sqrt( alpha*rmsOut(2)^2 + beta*rmsIn^2 );    
    rmsOut(2) = rmsOut(1);
    
    meanIn = rmsOut(1);
    
    % recursive in function of number of samples
    % http://www.iitk.ac.in/esc101/08Jan/lecnotes/lecture22.pdf
    %meanOut(1) = meanOut(2)*(N-1)/N + (1/N)*meanIn;
    meanOut(1) = alpha*meanOut(2) + beta*meanIn;
    meanOut(2) = meanOut(1);    
    
    Vrms = ((V_RATIO * meanOut(1)) + VCAL_B);
    
    %Plot_Var1(n+1)  = meanOut(2);
    Plot_Var1(n+1)  = u(n);
    Plot_Var2(n+1)  = Vrms;
    
end

% Truncate vector to same length
% t = t(1:length(Plot_Var1));
% u = u(1:length(Plot_Var1));

figure,subplot(1,1,1),plot(t,Plot_Var1,t,Plot_Var2,'r'),title('AC and RMS Voltages');
legend('AC','RMS');


