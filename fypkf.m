%% SETUP 
if loc == 'h'
    cd 'C:\Users\ttvr3\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter'
    winopen('C:\Users\Vince\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter')
elseif loc == 'u'
    cd 'C:\Users\ttvr3\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter'
    winopen('C:\Users\ttvr3\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter')
elseif loc == 'm'
    cd 'C:\Users\ttvr3\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter'
    winopen('C:\Users\vjren\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter')
end

g = 9.81 ;
dt = 1 ;
V = 5.56 ;

%% DATA
close all

timestamp = [0:0.01:30]' ;

% sensor & noise properties

% % uniformly distributed
% a1 = -15 ;
% b1 = 15 ;
% r1 = a1 + (b1-a1).*rand(3001,1) ;
% 
% a2 = -15 ;
% b2 = 15 ;
% r2 = a2 + (b2-a2).*rand(3001,1) ;

% normally distributed
ra1 = normrnd(0,2.5,length(timestamp),1) ;
ra2 = normrnd(0,2.5,length(timestamp),1) ;

rr1 = normrnd(0,0.05,length(timestamp)-1,1) ;
rr2 = normrnd(0,0.05,length(timestamp)-1,1) ;

% ground truth data
points = ones(length(timestamp),1) ;
points(1:2001,1) = sin(0.1*[0:360/2000:360]') ; % straight & steady
points(2002:2362,1) = 30*sin([0:1:360].*pi/180) ;
points(2363:2500,1) = 0.5*sin(0.02*[1:(360/length(points(2363:2500,1))):360]') ; % straight & steady
points(2501:2550,1) = exp([0:3.3142/length(points(2501:2549,1)):3.3142]')-0.6 ; % turn initiation
points(2551:2850,1) = points(2551:2850,1)*25 ; % steady turn 
points(2850:2900,1) = -exp([0:3.2189/length(points(2851:2900,1)):3.2189]')+26 ; % turn leveling
points(2901:3001,1) = -0.5*sin(0.02*[1:360/length(points(2901:3001,1)):360]') ;% straight & steady

%noisy data
data1 = points + ra1 ; %"sensor 1" measurements
data2 = points + ra2 ; %"sensor 2" measurements

for i = 1:length(data1a)
    rrpts(i,1) = points(i+1,1) - points(i,1,1) ;
end

data1a = rrpts + rr1 ;
data2a = rrpts + rr2 ;

%plot roll angle
figure
hold on
grid on
grid minor
plot(timestamp, data1, '-','color',[1,0,0])
plot(timestamp, data2, '-','color',[0.8,0.6,0])
plot(timestamp, points, '-','color',[0,1,0])
legend('{\it"Sensor 1"} data','{\it"Sensor 2"} data','Ground Truth','location','southwest')
title({'{\itArtificial} Roll Angle Measurement Data';'{\itFeatures straight and level, obstacle avoidance, steady turn}'})
ylabel('Roll Angle (deg)')
ylim([-42,42])
xlabel('Time Stamp (seconds)')
xlim([0,30])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot roll rate
figure
hold on
grid on
grid minor
plot(timestamp(2:length(timestamp),1), data1a, '-','color',[1,0,0])
plot(timestamp(2:length(timestamp),1), data2a, '-','color',[0.8,0.6,0])
plot(timestamp(2:length(timestamp),1), rrpts, '-','color',[0,1,0])
legend('{\it"Sensor 1"} data','{\it"Sensor 2"} data','Ground Truth','location','southwest')
title({'{\itArtificial} Roll Rate Measurement Data';'{\itFeatures straight and level, obstacle avoidance, steady turn}'})
ylabel('Roll Rate (deg/s)')
ylim([-2,2])
xlabel('Time Stamp (seconds)')
xlim([0,30])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% STATE OBSERVER

est = [ 0 ;     % k estimate of x roll angle
        0 ] ;   % k estimate of x angular rate sensor bias

phi_est = 0 ; 
bx_est = 0 ;
    
A = [1, -dt;    
     0, 1] ;    
 
X = [phi_est;   % k - 1 estimate of x roll angle
     bx_est] ;  % k - 1 bias estimate of the x angular rate sensor

B = [dt;        
     0] ;

for ii = 1:length(rr1)
    est(1:2,ii) = A*X + B*rr1(ii,1) ; % U is x angular rate/roll rate measurement input
    phi_est = est(1,ii) ;
end


%% SENSOR FUSION

% basic recursive estimator

%% SMALL ROLL ANGLE APPROXIMATION (FROM ANGULAR RATE)

phi_d = atan((psi_dot*U)/g) ; % derived from roll rate
% phi_d = phi_m ; % sensor measurements

%% LARGE ROLL ANGLE APPROXIMATION

phi_omega = sgn(omega_z)*asin(omega_y/sqrt((omega_y^2)+(omega_z^2))) ;

%% BLENDING FUNCTION

phi_bar = 0.05 ;
phi_hat = phi_est*(pi/180) ; % state observer estimate

W = exp(-(phi_hat^2)/(phi_bar^2)) ;

phim = W*phi_d + (1 - W)*phi_omega ;

%% KALMAN FILTER

% % State variables
% Roll rate is the only input variable/measurement

% SETUP
GPS_Lat_vk = err4a ; % Latitude reciever measurement noise
% GPS_Lat_wk =  ;    % state transition noise

GPS_Lon_vk = err4b ; % Longitude reciever measurement noise
% GPS_Lon_wk =  ;    % state transition noise

% % Define System Model Variables
% A_a = 1 ;
% H_a = 1 ;
% Q_a = 05e-12 ;
% R_a = cov(GPS_Lat_vk) ;
% 
% A_b = 1 ;
% H_b = 1 ;
% Q_b = 25e-12 ;
% R_b = cov(GPS_Lon_vk) ;


% PREALLOCATION
xkp_a = zeros(length(rrpts),1) ;
xk_a = zeros(length(rrpts),1) ;
Pkp_a = zeros(length(rrpts),1) ;
Pk_a = zeros(length(rrpts),1) ;
Kk_a = zeros(length(rrpts),1) ;

xkp_b = zeros(length(rrpts),1) ;
xk_b = zeros(length(rrpts),1) ;
Pkp_b = zeros(length(rrpts),1) ;
Pk_b = zeros(length(rrpts),1) ;
Kk_b = zeros(length(rrpts),1) ;

% 0) Set initial values xk (estimate) & Pk (prediction)
xkp_a(1,1) = rr1(1,1) ;
xk_a(1,1) = rr1(1,1) ;
Pkp_a(1,1) = cov(err4a) ;
Pk_a(1,1) = cov(err4a) ;
zk_a = rr1 ;

xkp_b(1,1) = rr2(1,1) ;
xk_b(1,1) = rr2(1,1) ;
Pkp_b(1,1) = cov(err4b) ;
Pk_b(1,1) = cov(err4b) ;
zk_b = rr2 ;

for f1 = 2:mean(length(GPS_Lat),length(GPS_Lon))
    % 1) Predict state (xk) and error co-variance (Pk)
    xkp_a(f1,1) = A_a*xk_a(f1-1,1) ;
    Pkp_a(f1,1) = A_a*Pk_a(f1-1,1)*A_a' + Q_a ;
    
    xkp_b(f1,1) = A_b*xk_b(f1-1,1) ;
    Pkp_b(f1,1) = A_b*Pk_b(f1-1,1)*A_b' + Q_b ;
    
    % 2) Compute Kalman gain (Kk)
    Kk_a(f1,1) = Pkp_a(f1,1)*H_a'/(H_a*Pkp_a(f1,1)*H_a' + R_a) ;
    Kk_b(f1,1) = Pkp_b(f1,1)*H_b'/(H_b*Pkp_b(f1,1)*H_b' + R_b) ;
    
    % 3) Compute estimate (xhatk) <------------- Feed in measurement
    xk_a(f1,1) = xkp_a(f1,1) + Kk_a(f1,1)*(zk_a(f1,1) - H_a*xkp_a(f1,1)) ;
    xk_b(f1,1) = xkp_b(f1,1) + Kk_b(f1,1)*(zk_b(f1,1) - H_b*xkp_b(f1,1)) ;
    
    % 4) Compute error covariance(Pk) ---------> Feed Pk back to step 1 as Pk-1
    Pk_a(f1,1) = Pkp_a(f1,1) - Kk_a(f1,1)*H_a*Pkp_a(f1,1) ;
    Pk_b(f1,1) = Pkp_b(f1,1) - Kk_b(f1,1)*H_b*Pkp_b(f1,1) ;
end
