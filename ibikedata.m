%% INSTRUMENTED BICYCLE DATA ANALYSIS PROGRAM
%% SETUP 
tic
clc
clear
close all

disp('INSTRUMENTED BICYCLE DATA ANALYSIS PROGRAM')

cd 'C:\Users\Vince\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter'
% winopen('C:\Users\Vince\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter')
% winopen('C:\Users\Vince\OneDrive - Loughborough University\1. University\4. Part C\Modules\FYP\Proj\Data Logger\Data')

% EULER ANGLES
Motion = ["Roll";"Yaw";"Pitch"] ;
Euler  = ["X";"Z";"Y"] ;
Sensor = ["Z + Y Components";"Y + Z Components";"X Only"] ;
disp('Orientation:')
ornt_tab = table(Motion,Euler,Sensor) ;
disp(ornt_tab)
disp(' ')

% Environment Constants
g = 9.81 ;

% Test Bike Geometry Constants
seat_tube_angle_deg = 16.5 ;
seat_tube_angle_rad = seat_tube_angle_deg*(pi/180) ;

%% ------- INVARIANT
%% ORIENTATION ANALYSIS

cd 'C:\Users\Vince\OneDrive - Loughborough University\1. University\4. Part C\Modules\FYP\Proj\Data Logger\Data\20200407 TEST 4'
orntdata = csvread('DATALOG.CSV') ;
file_ornt = 'orientation.csv' ;

time_ornt = orntdata(:,1) ;

% Unmoderated Data
AcX1_ornt = orntdata(:,7) ;
AcY1_ornt = orntdata(:,8) ;
AcZ1_ornt = orntdata(:,9) ;

AcX2_ornt = orntdata(:,14) ;
AcY2_ornt = orntdata(:,15) ;
AcZ2_ornt = orntdata(:,16) ;

GyX1_ornt = orntdata(:,10) ;
GyY1_ornt = orntdata(:,11) ;
GyZ1_ornt = orntdata(:,12) ;

GyX2_ornt = orntdata(:,17) ;
GyY2_ornt = orntdata(:,18) ;
GyZ2_ornt = orntdata(:,19) ;

AcX_ornt = zeros(length(orntdata),1) ;
AcY_ornt = zeros(length(orntdata),1) ;
AcZ_ornt = zeros(length(orntdata),1) ;
GyX_ornt = zeros(length(orntdata),1) ;
GyY_ornt = zeros(length(orntdata),1) ;
GyZ_ornt = zeros(length(orntdata),1) ;

i = 1 ;
for i = 1:length(orntdata)
    AcX_ornt(i,1) = (AcX1_ornt(i,1)+AcX2_ornt(i,1))/2 ;
    AcY_ornt(i,1) = (AcY1_ornt(i,1)+AcY2_ornt(i,1))/2 ;
    AcZ_ornt(i,1) = (AcZ1_ornt(i,1)+AcZ2_ornt(i,1))/2 ;
    GyX_ornt(i,1) = (GyX1_ornt(i,1)+GyX2_ornt(i,1))/2 ;
    GyY_ornt(i,1) = (GyY1_ornt(i,1)+GyY2_ornt(i,1))/2 ;
    GyZ_ornt(i,1) = (GyZ1_ornt(i,1)+GyZ2_ornt(i,1))/2 ;
end

%% CORRECTED ORIENTATION - Y & Z Axes only

% Seat Tube Angle Offset Corrector
M = inv([1,0,0;
         0,cos(seat_tube_angle_rad),-sin(seat_tube_angle_rad);
         0,sin(seat_tube_angle_rad),cos(seat_tube_angle_rad)]) ;
     
% Body Reference to Global Reference Transformation


 
Acc1_ornt = [AcX1_ornt,AcY1_ornt,AcZ1_ornt]' ;
Acc2_ornt = [AcX2_ornt,AcY2_ornt,AcZ2_ornt]' ;

GyO1_ornt = [GyX1_ornt,GyY1_ornt,GyZ1_ornt]' ;
GyO2_ornt = [GyX2_ornt,GyY2_ornt,GyZ2_ornt]' ;

Acc1_ontc = zeros(3,length(orntdata)) ;
Acc2_ontc = zeros(3,length(orntdata)) ;

GyO1_ontc = zeros(3,length(orntdata)) ;
GyO2_ontc = zeros(3,length(orntdata)) ;

i = 1 ;
for i = 1:length(orntdata)
    Acc1_ontc(:,i) = M*Acc1_ornt(:,i) ;
    Acc2_ontc(:,i) = M*Acc2_ornt(:,i) ;
    
    GyO1_ontc(:,i) = M*GyO1_ornt(:,i) ;
    GyO2_ontc(:,i) = M*GyO2_ornt(:,i) ;
end

AcY1_ontc = Acc1_ontc(2,:)' ;
AcZ1_ontc = Acc1_ontc(3,:)' ;
AcY2_ontc = Acc2_ontc(2,:)' ;
AcZ2_ontc = Acc2_ontc(3,:)' ;

GyY1_ontc = GyO1_ontc(2,:)' ;
GyZ1_ontc = GyO1_ontc(3,:)' ;
GyY2_ontc = GyO2_ontc(2,:)' ;
GyZ2_ontc = GyO2_ontc(3,:)' ;

AcY_ontc = zeros(length(orntdata),1) ;
AcZ_ontc = zeros(length(orntdata),1) ;

GyY_ontc = zeros(length(orntdata),1) ;
GyZ_ontc = zeros(length(orntdata),1) ;

i = 1 ;
for i = 1:length(orntdata)
    AcY_ontc(i,1) = (AcY1_ontc(i,1)+AcY2_ontc(i,1))/2 ;
    AcZ_ontc(i,1) = (AcZ1_ontc(i,1)+AcZ2_ontc(i,1))/2 ;
    GyY_ontc(i,1) = (GyY1_ontc(i,1)+GyY2_ontc(i,1))/2 ;
    GyZ_ontc(i,1) = (GyZ1_ontc(i,1)+GyZ2_ontc(i,1))/2 ;
end

%% STATIONARY ORIENTATION - ROLL & PITCH ONLY
% RAW
stat_rolxr1 = zeros(length(AcY_ornt),1);
stat_pitzr1 = zeros(length(AcY_ornt),1);
stat_rolxr2 = zeros(length(AcY_ornt),1);
stat_pitzr2 = zeros(length(AcY_ornt),1);
stat_rolxrm = zeros(length(AcY_ornt),1);
stat_pitzrm = zeros(length(AcY_ornt),1);

i = 1 ;
for i = 1:length(AcY_ornt)
    stat_rolxr1(i,1) = asin((AcX1_ornt(i,1)/g)) ;
    stat_pitzr1(i,1) = asin((AcZ1_ornt(i,1)/g)) ;
    
    stat_rolxr2(i,1) = asin((AcX2_ornt(i,1)/g)) ;
    stat_pitzr2(i,1) = asin((AcZ2_ornt(i,1)/g)) ;
    
    stat_rolxrm(i,1) = asin((AcX_ornt(i,1)/g)) ;
    stat_pitzrm(i,1) = asin((AcZ_ornt(i,1)/g)) ;

end
    
% CORRECTED
stat_rolxc1 = zeros(length(AcY_ontc),1);
stat_pitzc1 = zeros(length(AcY_ontc),1);
stat_rolxc2 = zeros(length(AcY_ontc),1);
stat_pitzc2 = zeros(length(AcY_ontc),1);
stat_rolxcm = zeros(length(AcY_ontc),1);
stat_pitzcm = zeros(length(AcY_ontc),1);

i = 1 ;
for i = 1:length(AcY_ontc) 
    stat_rolxc1(i,1) = asin((AcX1_ornt(i,1)/g)) ;
    stat_pitzc1(i,1) = asin((AcZ1_ontc(i,1)/g)) ;
    
    stat_rolxc2(i,1) = asin((AcX2_ornt(i,1)/g)) ;
    stat_pitzc2(i,1) = asin((AcZ2_ontc(i,1)/g)) ;
    
    stat_rolxcm(i,1) = asin((AcX_ornt(i,1)/g)) ;
    stat_pitzcm(i,1) = asin((AcZ_ontc(i,1)/g)) ;
end

%% PLOTS - ORIENTATION
close all
% RAW
figure
%plot AcX
subplot(231)
hold on
grid on
grid minor
plot(time_ornt, AcX1_ornt, 'g-')
plot(time_ornt, AcX2_ornt, 'b-')
plot(time_ornt, AcX_ornt, 'r-')
title('Orientation Test X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(232)
hold on
grid on
grid minor
plot(time_ornt, AcY1_ornt, 'g-')
plot(time_ornt, AcY2_ornt, 'b-')
plot(time_ornt, AcY_ornt, 'r-')
title('Orientation Test Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(233)
hold on
grid on
grid minor
plot(time_ornt, AcZ1_ornt, 'g-')
plot(time_ornt, AcZ2_ornt, 'b-')
plot(time_ornt, AcZ_ornt, 'r-')
title('Orientation Test Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(234)
hold on
grid on
grid minor
plot(time_ornt, GyX1_ornt, 'g-')
plot(time_ornt, GyX2_ornt, 'b-')
plot(time_ornt, GyX_ornt, 'r-')
title('Orientation Test X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(235)
hold on
grid on
grid minor
plot(time_ornt, GyY1_ornt, 'g-')
plot(time_ornt, GyY2_ornt, 'b-')
plot(time_ornt, GyY_ornt, 'r-')
title('Orientation Test Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(236)
hold on
grid on
grid minor
plot(time_ornt, GyZ1_ornt, 'g-')
plot(time_ornt, GyZ2_ornt, 'b-')
plot(time_ornt, GyZ_ornt, 'r-')
title('Orientation Test Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Corrected
figure
%plot AcX
subplot(231)
hold on
grid on
grid minor
plot(time_ornt, AcX1_ornt, 'g-')
plot(time_ornt, AcX2_ornt, 'b-')
plot(time_ornt, AcX_ornt, 'r-')
title('Corrected X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(232)
hold on
grid on
grid minor
plot(time_ornt, AcY1_ontc, 'g-')
plot(time_ornt, AcY2_ontc, 'b-')
plot(time_ornt, AcY_ontc, 'r-')
title('Corrected Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(233)
hold on
grid on
grid minor
plot(time_ornt, AcZ1_ontc, 'g-')
plot(time_ornt, AcZ2_ontc, 'b-')
plot(time_ornt, AcZ_ontc, 'r-')
title('Corrected Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(234)
hold on
grid on
grid minor
plot(time_ornt, GyX1_ornt, 'g-')
plot(time_ornt, GyX2_ornt, 'b-')
plot(time_ornt, GyX_ornt, 'r-')
title('Corrected X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(235)
hold on
grid on
grid minor
plot(time_ornt, GyY1_ontc, 'g-')
plot(time_ornt, GyY2_ontc, 'b-')
plot(time_ornt, GyY_ontc, 'r-')
title('Corrected Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(236)
hold on
grid on
grid minor
plot(time_ornt, GyZ1_ontc, 'g-')
plot(time_ornt, GyZ2_ontc, 'b-')
plot(time_ornt, GyZ_ontc, 'r-')
title('Corrected Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Attitude Plot
figure
%Raw
subplot(2,2,1)
hold on
grid on
grid minor
plot(time_ornt, rad2deg(stat_rolxr1), 'g-')
plot(time_ornt, rad2deg(stat_rolxr2), 'b-')
plot(time_ornt, rad2deg(stat_rolxrm), 'r-')
title('Raw Derived Roll Angle')
ylabel('Roll Angle (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(2,2,3)
hold on
grid on
grid minor
plot(time_ornt, rad2deg(stat_pitzr1), 'g-')
plot(time_ornt, rad2deg(stat_pitzr2), 'b-')
plot(time_ornt, rad2deg(stat_pitzrm), 'r-')
title('Raw Derived Pitch Angle')
ylabel('Pitch Angle (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%Corrected
subplot(2,2,2)
hold on
grid on
grid minor
plot(time_ornt, rad2deg(stat_rolxc1), 'g-')
plot(time_ornt, rad2deg(stat_rolxc2), 'b-')
plot(time_ornt, rad2deg(stat_rolxcm), 'r-')
title('Corrected Derived Roll Angle')
ylabel('Roll Angle (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(2,2,4)
hold on
grid on
grid minor
plot(time_ornt, rad2deg(stat_pitzc1), 'g-')
plot(time_ornt, rad2deg(stat_pitzc2), 'b-')
plot(time_ornt, rad2deg(stat_pitzcm), 'r-')
title('Corrected Derived Pitch Angle')
ylabel('Pitch Angle (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([0,150])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% BIAS/ERROR ANALYSIS
cd 'C:\Users\Vince\OneDrive - Loughborough University\1. University\4. Part C\Modules\FYP\Proj\Data Logger\Data\20200407 TEST 3'
biasdata = csvread('DATALOG.CSV') ;
file_biasSum = 'biasSummary.csv' ;

time_stat = biasdata(:,1) ;

AcX1_stat = biasdata(:,7) ;
AcY1_stat = biasdata(:,8) ;
AcZ1_stat = biasdata(:,9) ;
AcX2_stat = biasdata(:,14) ;
AcY2_stat = biasdata(:,15) ;
AcZ2_stat = biasdata(:,16) ;

GyX1_stat = biasdata(:,10) ;
GyY1_stat = biasdata(:,11) ;
GyZ1_stat = biasdata(:,12) ;
GyX2_stat = biasdata(:,17) ;
GyY2_stat = biasdata(:,18) ;
GyZ2_stat = biasdata(:,19) ;

AcX1_bias = sum(AcX1_stat)/length(AcX1_stat) ;
AcY1_bias = sum(AcY1_stat)/length(AcY1_stat) ;
AcZ1_bias = sum(AcZ1_stat)/length(AcZ1_stat) ;
GyX1_bias = sum(GyX1_stat)/length(GyX1_stat) ;
GyY1_bias = sum(GyY1_stat)/length(GyY1_stat) ;
GyZ1_bias = sum(GyZ1_stat)/length(GyZ1_stat) ;
AcX2_bias = sum(AcX2_stat)/length(AcX2_stat) ;
AcY2_bias = sum(AcY2_stat)/length(AcY2_stat) ;
AcZ2_bias = sum(AcZ2_stat)/length(AcZ2_stat) ;
GyX2_bias = sum(GyX2_stat)/length(GyX2_stat) ;
GyY2_bias = sum(GyY2_stat)/length(GyY2_stat) ;
GyZ2_bias = sum(GyZ2_stat)/length(GyZ2_stat) ;

AcX1_svar = sum((AcX1_stat-(ones(length(AcX1_stat),1).*AcX1_bias)).^2)/length(AcX1_stat) ;
AcY1_svar = sum((AcY1_stat-(ones(length(AcY1_stat),1).*AcY1_bias)).^2)/length(AcY1_stat) ;
AcZ1_svar = sum((AcZ1_stat-(ones(length(AcZ1_stat),1).*AcZ1_bias)).^2)/length(AcZ1_stat) ;
GyX1_svar = sum((GyX1_stat-(ones(length(GyX1_stat),1).*GyX1_bias)).^2)/length(GyX1_stat) ;
GyY1_svar = sum((GyY1_stat-(ones(length(GyY1_stat),1).*GyY1_bias)).^2)/length(GyY1_stat) ;
GyZ1_svar = sum((GyZ1_stat-(ones(length(GyZ1_stat),1).*GyZ1_bias)).^2)/length(GyZ1_stat) ;
AcX2_svar = sum((AcX2_stat-(ones(length(AcX2_stat),1).*AcX2_bias)).^2)/length(AcX2_stat) ;
AcY2_svar = sum((AcY2_stat-(ones(length(AcY2_stat),1).*AcY2_bias)).^2)/length(AcY2_stat) ;
AcZ2_svar = sum((AcZ2_stat-(ones(length(AcZ2_stat),1).*AcZ2_bias)).^2)/length(AcZ2_stat) ;
GyX2_svar = sum((GyX2_stat-(ones(length(GyX2_stat),1).*GyX2_bias)).^2)/length(GyX2_stat) ;
GyY2_svar = sum((GyY2_stat-(ones(length(GyY2_stat),1).*GyY2_bias)).^2)/length(GyY2_stat) ;
GyZ2_svar = sum((GyZ2_stat-(ones(length(GyZ2_stat),1).*GyZ2_bias)).^2)/length(GyZ2_stat) ;

AcX1_ssdv = sqrt(AcX1_svar) ;
AcY1_ssdv = sqrt(AcY1_svar) ;
AcZ1_ssdv = sqrt(AcZ1_svar) ;
GyX1_ssdv = sqrt(GyX1_svar) ;
GyY1_ssdv = sqrt(GyY1_svar) ;
GyZ1_ssdv = sqrt(GyZ1_svar) ;
AcX2_ssdv = sqrt(AcX2_svar) ;
AcY2_ssdv = sqrt(AcY2_svar) ;
AcZ2_ssdv = sqrt(AcZ2_svar) ;
GyX2_ssdv = sqrt(GyX2_svar) ;
GyY2_ssdv = sqrt(GyY2_svar) ;
GyZ2_ssdv = sqrt(GyZ2_svar) ;

% Fusion of MPU1 & MPU2 Data using Traditional Mean
AcX_stat = zeros(length(biasdata),1) ;
AcY_stat = zeros(length(biasdata),1) ;
AcZ_stat = zeros(length(biasdata),1) ;
GyX_stat = zeros(length(biasdata),1) ;
GyY_stat = zeros(length(biasdata),1) ;
GyZ_stat = zeros(length(biasdata),1) ;

i = 1 ;
for i = 1:length(biasdata)
    AcX_stat(i,1) = (AcX1_stat(i,1)+AcX2_stat(i,1))/2 ;
    AcY_stat(i,1) = (AcY1_stat(i,1)+AcY2_stat(i,1))/2 ;
    AcZ_stat(i,1) = (AcZ1_stat(i,1)+AcZ2_stat(i,1))/2 ;
    GyX_stat(i,1) = (GyX1_stat(i,1)+GyX2_stat(i,1))/2 ;
    GyY_stat(i,1) = (GyY1_stat(i,1)+GyY2_stat(i,1))/2 ;
    GyZ_stat(i,1) = (GyZ1_stat(i,1)+GyZ2_stat(i,1))/2 ;
end

AcX_bias = sum(AcX_stat)/length(AcX_stat) ;
AcY_bias = sum(AcY_stat)/length(AcY_stat) ;
AcZ_bias = sum(AcZ_stat)/length(AcZ_stat) ;
GyX_bias = sum(GyX_stat)/length(GyX_stat) ;
GyY_bias = sum(GyY_stat)/length(GyY_stat) ;
GyZ_bias = sum(GyZ_stat)/length(GyZ_stat) ;

AcX_svar = sum((AcX_stat-(ones(length(AcX_stat),1).*AcX_bias)).^2)/length(AcX_stat) ;
AcY_svar = sum((AcY_stat-(ones(length(AcY_stat),1).*AcY_bias)).^2)/length(AcY_stat) ;
AcZ_svar = sum((AcZ_stat-(ones(length(AcZ_stat),1).*AcZ_bias)).^2)/length(AcZ_stat) ;
GyX_svar = sum((GyX_stat-(ones(length(GyX_stat),1).*GyX_bias)).^2)/length(GyX_stat) ;
GyY_svar = sum((GyY_stat-(ones(length(GyY_stat),1).*GyY_bias)).^2)/length(GyY_stat) ;
GyZ_svar = sum((GyZ_stat-(ones(length(GyZ_stat),1).*GyZ_bias)).^2)/length(GyZ_stat) ;

AcX_ssdv = sqrt(AcX_svar) ;
AcY_ssdv = sqrt(AcY_svar) ;
AcZ_ssdv = sqrt(AcZ_svar) ;
GyX_ssdv = sqrt(GyX_svar) ;
GyY_ssdv = sqrt(GyY_svar) ;
GyZ_ssdv = sqrt(GyZ_svar) ;

%% CORRECTED STATIC DATA
file_biasCor = 'biasCorrect.csv' ;

Acc1_bias = [biasdata(:,7)';biasdata(:,8)';biasdata(:,9)'] ;
Acc2_bias = [biasdata(:,14)';biasdata(:,15)';biasdata(:,16)'] ;

GyO1_bias = [biasdata(:,10)';biasdata(:,11)';biasdata(:,12)'] ;
GyO2_bias = [biasdata(:,17)';biasdata(:,18)';biasdata(:,19)'] ;

Acc1_biac = zeros(3,length(biasdata)) ;
Acc2_biac = zeros(3,length(biasdata)) ;
GyO1_biac = zeros(3,length(biasdata)) ;
GyO2_biac = zeros(3,length(biasdata)) ;

i = 1 ;
for i = 1:length(biasdata)
    Acc1_biac(:,i) = M*Acc1_bias(:,i) ;
    Acc2_biac(:,i) = M*Acc2_bias(:,i) ;

    GyO1_biac(:,i) = M*GyO1_bias(:,i) ;
    GyO2_biac(:,i) = M*GyO2_bias(:,i) ;
end

AcX1_stac = Acc1_biac(1,:)' ;
AcY1_stac = Acc1_biac(2,:)' ;
AcZ1_stac = Acc1_biac(3,:)' ;
GyX1_stac = GyO1_biac(1,:)' ;
GyY1_stac = GyO1_biac(2,:)' ;
GyZ1_stac = GyO1_biac(3,:)' ;
AcX2_stac = Acc2_biac(1,:)' ;
AcY2_stac = Acc2_biac(2,:)' ;
AcZ2_stac = Acc2_biac(3,:)' ;
GyX2_stac = GyO2_biac(1,:)' ;
GyY2_stac = GyO2_biac(2,:)' ;
GyZ2_stac = GyO2_biac(3,:)' ;

AcX1_biac = sum(AcX1_stac)/length(AcX1_stac) ;
AcY1_biac = sum(AcY1_stac)/length(AcY1_stac) ;
AcZ1_biac = sum(AcZ1_stac)/length(AcZ1_stac) ;
GyX1_biac = sum(GyX1_stac)/length(GyX1_stac) ;
GyY1_biac = sum(GyY1_stac)/length(GyY1_stac) ;
GyZ1_biac = sum(GyZ1_stac)/length(GyZ1_stac) ;
AcX2_biac = sum(AcX2_stac)/length(AcX2_stac) ;
AcY2_biac = sum(AcY2_stac)/length(AcY2_stac) ;
AcZ2_biac = sum(AcZ2_stac)/length(AcZ2_stac) ;
GyX2_biac = sum(GyX2_stac)/length(GyX2_stac) ;
GyY2_biac = sum(GyY2_stac)/length(GyY2_stac) ;
GyZ2_biac = sum(GyZ2_stac)/length(GyZ2_stac) ;

AcX1_svrc = sum((AcX1_stac-(ones(length(AcX1_stac),1).*AcX1_biac)).^2)/length(AcX1_stac) ;
AcY1_svrc = sum((AcY1_stac-(ones(length(AcY1_stac),1).*AcY1_biac)).^2)/length(AcY1_stac) ;
AcZ1_svrc = sum((AcZ1_stac-(ones(length(AcZ1_stac),1).*AcZ1_biac)).^2)/length(AcZ1_stac) ;
GyX1_svrc = sum((GyX1_stac-(ones(length(GyX1_stac),1).*GyX1_biac)).^2)/length(GyX1_stac) ;
GyY1_svrc = sum((GyY1_stac-(ones(length(GyY1_stac),1).*GyY1_biac)).^2)/length(GyY1_stac) ;
GyZ1_svrc = sum((GyZ1_stac-(ones(length(GyZ1_stac),1).*GyZ1_biac)).^2)/length(GyZ1_stac) ;
AcX2_svrc = sum((AcX2_stac-(ones(length(AcX2_stac),1).*AcX2_biac)).^2)/length(AcX2_stac) ;
AcY2_svrc = sum((AcY2_stac-(ones(length(AcY2_stac),1).*AcY2_biac)).^2)/length(AcY2_stac) ;
AcZ2_svrc = sum((AcZ2_stac-(ones(length(AcZ2_stac),1).*AcZ2_biac)).^2)/length(AcZ2_stac) ;
GyX2_svrc = sum((GyX2_stac-(ones(length(GyX2_stac),1).*GyX2_biac)).^2)/length(GyX2_stac) ;
GyY2_svrc = sum((GyY2_stac-(ones(length(GyY2_stac),1).*GyY2_biac)).^2)/length(GyY2_stac) ;
GyZ2_svrc = sum((GyZ2_stac-(ones(length(GyZ2_stac),1).*GyZ2_biac)).^2)/length(GyZ2_stac) ;

AcX1_sdvc = sqrt(AcX1_svrc) ;
AcY1_sdvc = sqrt(AcY1_svrc) ;
AcZ1_sdvc = sqrt(AcZ1_svrc) ;
GyX1_sdvc = sqrt(GyX1_svrc) ;
GyY1_sdvc = sqrt(GyY1_svrc) ;
GyZ1_sdvc = sqrt(GyZ1_svrc) ;
AcX2_sdvc = sqrt(AcX2_svrc) ;
AcY2_sdvc = sqrt(AcY2_svrc) ;
AcZ2_sdvc = sqrt(AcZ2_svrc) ;
GyX2_sdvc = sqrt(GyX2_svrc) ;
GyY2_sdvc = sqrt(GyY2_svrc) ;
GyZ2_sdvc = sqrt(GyZ2_svrc) ;

% Fusion of MPU1 & MPU2 Data using Traditional Mean
AcX_stac = zeros(length(biasdata),1) ;
AcY_stac = zeros(length(biasdata),1) ;
AcZ_stac = zeros(length(biasdata),1) ;
GyX_stac = zeros(length(biasdata),1) ;
GyY_stac = zeros(length(biasdata),1) ;
GyZ_stac = zeros(length(biasdata),1) ;

i = 1 ;
for i = 1:length(biasdata)
    AcX_stac(i,1) = (AcX1_stac(i,1)+AcX2_stac(i,1))/2 ;
    AcY_stac(i,1) = (AcY1_stac(i,1)+AcY2_stac(i,1))/2 ;
    AcZ_stac(i,1) = (AcZ1_stac(i,1)+AcZ2_stac(i,1))/2 ;
    GyX_stac(i,1) = (GyX1_stac(i,1)+GyX2_stac(i,1))/2 ;
    GyY_stac(i,1) = (GyY1_stac(i,1)+GyY2_stac(i,1))/2 ;
    GyZ_stac(i,1) = (GyZ1_stac(i,1)+GyZ2_stac(i,1))/2 ;
end

AcX_biac = sum(AcX_stac)/length(AcX_stac) ;
AcY_biac = sum(AcY_stac)/length(AcY_stac) ;
AcZ_biac = sum(AcZ_stac)/length(AcZ_stac) ;
GyX_biac = sum(GyX_stac)/length(GyX_stac) ;
GyY_biac = sum(GyY_stac)/length(GyY_stac) ;
GyZ_biac = sum(GyZ_stac)/length(GyZ_stac) ;

AcX_svrc = sum((AcX_stac-(ones(length(AcX_stac),1).*AcX_biac)).^2)/length(AcX_stac) ;
AcY_svrc = sum((AcY_stac-(ones(length(AcY_stac),1).*AcY_biac)).^2)/length(AcY_stac) ;
AcZ_svrc = sum((AcZ_stac-(ones(length(AcZ_stac),1).*AcZ_biac)).^2)/length(AcZ_stac) ;
GyX_svrc = sum((GyX_stac-(ones(length(GyX_stac),1).*GyX_biac)).^2)/length(GyX_stac) ;
GyY_svrc = sum((GyY_stac-(ones(length(GyY_stac),1).*GyY_biac)).^2)/length(GyY_stac) ;
GyZ_svrc = sum((GyZ_stac-(ones(length(GyZ_stac),1).*GyZ_biac)).^2)/length(GyZ_stac) ;

AcX_sdvc = sqrt(AcX_svrc) ;
AcY_sdvc = sqrt(AcY_svrc) ;
AcZ_sdvc = sqrt(AcZ_svrc) ;
GyX_sdvc = sqrt(GyX_svrc) ;
GyY_sdvc = sqrt(GyY_svrc) ;
GyZ_sdvc = sqrt(GyZ_svrc) ;

%% STAT TEST SUMMARY

meas = ["AcX1";"AcY1";"AcZ1";"GyX1";"GyY1";"GyZ1";"0";"AcX2";"AcY2";"AcZ2";"GyX2";"GyY2";"GyZ2";"0";"AcX";"AcY";"AcZ";"GyX";"GyY";"GyZ"] ;
head = ["Measurement","Mean","Variance","Std. Dev."] ;
bias = [AcX1_bias;AcY1_bias;AcZ1_bias;GyX1_bias;GyY1_bias;GyZ1_bias;0;AcX2_bias;AcY2_bias;AcZ2_bias;GyX2_bias;GyY2_bias;GyZ2_bias;0;AcX_bias;AcY_bias;AcZ_bias;GyX_bias;GyY_bias;GyZ_bias] ;
svar = [AcX1_svar;AcY1_svar;AcZ1_svar;GyX1_svar;GyY1_svar;GyZ1_svar;0;AcX2_svar;AcY2_svar;AcZ2_svar;GyX2_svar;GyY2_svar;GyZ2_svar;0;AcX_svar;AcY_svar;AcZ_svar;GyX_svar;GyY_svar;GyZ_svar] ;
ssdv = [AcX1_ssdv;AcY1_ssdv;AcZ1_ssdv;GyX1_ssdv;GyY1_ssdv;GyZ1_ssdv;0;AcX2_ssdv;AcY2_ssdv;AcZ2_ssdv;GyX2_ssdv;GyY2_ssdv;GyZ2_ssdv;0;AcX_ssdv;AcY_ssdv;AcZ_ssdv;GyX_ssdv;GyY_ssdv;GyZ_ssdv] ;

stat_test = table(meas,bias,svar,ssdv) ;
writetable(stat_test,file_biasSum,'Delimiter',',')
disp(stat_test)

% corrected
biac = [AcX1_biac;AcY1_biac;AcZ1_biac;GyX1_biac;GyY1_biac;GyZ1_biac;0;AcX2_biac;AcY2_biac;AcZ2_biac;GyX2_biac;GyY2_biac;GyZ2_biac;0;AcX_biac;AcY_biac;AcZ_biac;GyX_biac;GyY_biac;GyZ_biac] ;
svrc = [AcX1_svrc;AcY1_svrc;AcZ1_svrc;GyX1_svrc;GyY1_svrc;GyZ1_svrc;0;AcX2_svrc;AcY2_svrc;AcZ2_svrc;GyX2_svrc;GyY2_svrc;GyZ2_svrc;0;AcX_svrc;AcY_svrc;AcZ_svrc;GyX_svrc;GyY_svrc;GyZ_svrc] ;
sdvc = [AcX1_sdvc;AcY1_sdvc;AcZ1_sdvc;GyX1_sdvc;GyY1_sdvc;GyZ1_sdvc;0;AcX2_sdvc;AcY2_sdvc;AcZ2_sdvc;GyX2_sdvc;GyY2_sdvc;GyZ2_sdvc;0;AcX_sdvc;AcY_sdvc;AcZ_sdvc;GyX_sdvc;GyY_sdvc;GyZ_sdvc] ;

stac_test = table(meas,biac,svrc,sdvc) ;
writetable(stac_test,file_biasCor,'Delimiter',',')
disp(stac_test)

%% PLOTS - STATIC TEST
close all
% Un moderated
figure
%plot AcX
subplot(231)
hold on
grid on
grid minor
plot(time_stat, AcX1_stat, 'g-')
plot(time_stat, AcX2_stat, 'b-')
plot(time_stat, AcX_stat, 'r-')
title('Static Test X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(232)
hold on
grid on
grid minor
plot(time_stat, AcY1_stat, 'g-')
plot(time_stat, AcY2_stat, 'b-')
plot(time_stat, AcY_stat, 'r-')
title('Static Test Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(233)
hold on
grid on
grid minor
plot(time_stat, AcZ1_stat, 'g-')
plot(time_stat, AcZ2_stat, 'b-')
plot(time_stat, AcZ_stat, 'r-')
title('Static Test Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(234)
hold on
grid on
grid minor
plot(time_stat, GyX1_stat, 'g-')
plot(time_stat, GyX2_stat, 'b-')
plot(time_stat, GyX_stat, 'r-')
title('Static Test X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(235)
hold on
grid on
grid minor
plot(time_stat, GyY1_stat, 'g-')
plot(time_stat, GyY2_stat, 'b-')
plot(time_stat, GyY_stat, 'r-')
title('Static Test Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(236)
hold on
grid on
grid minor
plot(time_stat, GyZ1_stat, 'g-')
plot(time_stat, GyZ2_stat, 'b-')
plot(time_stat, GyZ_stat, 'r-')
title('Static Test Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Corrected
figure
%plot AcX
subplot(231)
hold on
grid on
grid minor
plot(time_stat, AcX1_stat, 'g-')
plot(time_stat, AcX2_stat, 'b-')
plot(time_stat, AcX_stat, 'r-')
title('Static Test X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(232)
hold on
grid on
grid minor
plot(time_stat, AcY1_stac, 'g-')
plot(time_stat, AcY2_stac, 'b-')
plot(time_stat, AcY_stac, 'r-')
title('Static Test Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(233)
hold on
grid on
grid minor
plot(time_stat, AcZ1_stac, 'g-')
plot(time_stat, AcZ2_stac, 'b-')
plot(time_stat, AcZ_stac, 'r-')
title('Static Test Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(234)
hold on
grid on
grid minor
plot(time_stat, GyX1_stat, 'g-')
plot(time_stat, GyX2_stat, 'b-')
plot(time_stat, GyX_stat, 'r-')
title('Static Test X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(235)
hold on
grid on
grid minor
plot(time_stat, GyY1_stac, 'g-')
plot(time_stat, GyY2_stac, 'b-')
plot(time_stat, GyY_stac, 'r-')
title('Static Test Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(236)
hold on
grid on
grid minor
plot(time_stat, GyZ1_stac, 'g-')
plot(time_stat, GyZ2_stac, 'b-')
plot(time_stat, GyZ_stac, 'r-')
title('Static Test Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
% xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% -------- VARIANT
%% FIELD TEST RAW DATA

% field test 1
cd 'C:\Users\Vince\OneDrive - Loughborough University\1. University\4. Part C\Modules\FYP\Proj\Data Logger\Data\20200402 TEST 1'
dump1 = csvread('DATALOG.CSV') ;
% data = dump1(3866:25701,:) ;
% filename = 'test1summary.csv' ;

% field test 2
cd 'C:\Users\Vince\OneDrive - Loughborough University\1. University\4. Part C\Modules\FYP\Proj\Data Logger\Data\20200404 TEST 2'
dump2 = csvread('DATALOG.CSV') ;
data = dump2(2572:24998,:) ;
filename = 'test2summary.csv' ;

cd 'C:\Users\Vince\OneDrive - Loughborough University\MATLAB\Part C\FYP - Autonomous Electric Bicycle\Proj\Kalman Filter'

timestamp = data(:,1) ;
U_ms = data(:,2) ;
U_km = data(:,3) ;
S_ms = data(:,4) ;
S_km = data(:,5) ;

Temp1 = data(:,6) ;
Temp2 = data(:,13) ;

AcX1_raw = data(:,7) ;
AcY1_raw = data(:,8) ;
AcZ1_raw = data(:,9) ;
AcX2_raw = data(:,14) ;
AcY2_raw = data(:,15) ;
AcZ2_raw = data(:,16) ;

Acc1_test = data(:,7:9)' ;
Acc2_test = data(:,14:16)' ;

GyX1_raw = data(:,10) ;
GyY1_raw = data(:,11) ;
GyZ1_raw = data(:,12) ;
GyX2_raw = data(:,17) ;
GyY2_raw = data(:,18) ;
GyZ2_raw = data(:,19) ;

GyO1_test = data(:,10:12)' ;
GyO2_test = data(:,17:19)' ;

%% FIELD TEST CORRECTED DATA
Acc1_corr = zeros(3,length(data)) ; 
Acc2_corr = zeros(3,length(data)) ;
GyO1_corr = zeros(3,length(data)) ;
GyO2_corr = zeros(3,length(data)) ;

i = 1 ;
for i = 1:length(data)
    Acc1_corr(:,i) = M*Acc1_test(:,i) ;
    Acc2_corr(:,i) = M*Acc2_test(:,i) ;

    GyO1_corr(:,i) = M*GyO1_test(:,i) ;
    GyO2_corr(:,i) = M*GyO2_test(:,i) ;
end

AcX1_crt = Acc1_corr(1,:)' ;
AcY1_crt = Acc1_corr(2,:)' ;
AcZ1_crt = Acc1_corr(3,:)' ;
AcX2_crt = Acc2_corr(1,:)' ;
AcY2_crt = Acc2_corr(2,:)' ;
AcZ2_crt = Acc2_corr(3,:)' ;

GyX1_crt = GyO1_corr(1,:)' ;
GyY1_crt = GyO1_corr(2,:)' ;
GyZ1_crt = GyO1_corr(3,:)' ;
GyX2_crt = GyO2_corr(1,:)' ;
GyY2_crt = GyO2_corr(2,:)' ;
GyZ2_crt = GyO2_corr(3,:)' ;

check_0 = zeros(length(GyO1_corr),1) ;

i = 1 ;
for i = 1:length(GyO1_corr)
    if GyY1_crt(i,1) == 0 && GyX1_crt(i,1) == 0
        check_0(i,1) = 1 ;
        disp("Zero Event:")
        disp(i)
        GyX1_crt(i,1) = sign(GyX1_crt(i-1,1))*0.0001 ;
        GyY1_crt(i,1) = sign(GyY1_crt(i-1,1))*0.0001 ;
        disp("Zero Event Corrected:")
        disp([' X ','     |||     ',' Y '])
        disp([num2str(GyX1_crt(i,1)),' ||| ',num2str(GyY1_crt(i,1))])
    end
end

Z0s = sum(check_0) ;
disp("Total Zero Events:")
disp(Z0s)

%% VELOCITY & TIME INTERVAL CALCULATION
% Time Interval, dt
dtv = zeros(length(data),1) ;
dtv(1,1) = timestamp(1,1) ;

for t = 2:length(data)
    dtv(t,1) = timestamp(t,1)-timestamp(t-1,1) ;
end

dtm = sum(dtv)/length(dtv) ;

% Forward Velocity ~0.5 sec Moving Average
U_ma = zeros(length(U_ms),1) ;
U_ma(1:12,1) = U_ms(1:12,1) ;
U_ka = zeros(length(U_km),1) ;
U_ka(1:12,1) = U_km(1:12,1) ;

for a = 13:length(U_ms)
    U_ma(a,1) = sum(U_ms(a-12:a,1))/13 ;
    U_ka(a,1) = sum(U_km(a-12:a,1))/13 ;
end

% % Forward Velocity 1 sec Moving Average
% U_ma = zeros(length(U_ms),1) ;
% U_ma(1:24,1) = U_ms(1:24,1) ;
% U_ka = zeros(length(U_km),1) ;
% U_ka(1:24,1) = U_km(1:24,1) ;
% 
% for a = 25:length(U_ms)
%     U_ma(a,1) = sum(U_ms(a-24:a,1))/25 ;
%     U_ka(a,1) = sum(U_km(a-24:a,1))/25 ;
% end

% % Forward Velocity 2 sec Moving Average
% U_ma = zeros(length(U_ms),1) ;
% U_ma(1:49,1) = U_ms(1:49,1) ;
% U_ka = zeros(length(U_km),1) ;
% U_ka(1:49,1) = U_km(1:49,1) ;
% 
% for a = 50:length(U_ms)
%     U_ma(a,1) = sum(U_ms(a-49:a,1))/50 ;
%     U_ka(a,1) = sum(U_km(a-49:a,1))/50 ;
% end

%% SENSOR FUSION
% Fusion of MPU1 & MPU2 Data using Traditional Mean
AcX_raw = zeros(length(data),1) ;
AcY_raw = zeros(length(data),1) ;
AcZ_raw = zeros(length(data),1) ;
GyX_raw = zeros(length(data),1) ;
GyY_raw = zeros(length(data),1) ;
GyZ_raw = zeros(length(data),1) ;

i = 1 ;
for i = 1:length(data)
    AcX_raw(i,1) = (AcX1_raw(i,1)+AcX2_raw(i,1))/2 ;
    AcY_raw(i,1) = (AcY1_raw(i,1)+AcY2_raw(i,1))/2 ;
    AcZ_raw(i,1) = (AcZ1_raw(i,1)+AcZ2_raw(i,1))/2 ;
    GyX_raw(i,1) = (GyX1_raw(i,1)+GyX2_raw(i,1))/2 ;
    GyY_raw(i,1) = (GyY1_raw(i,1)+GyY2_raw(i,1))/2 ;
    GyZ_raw(i,1) = (GyZ1_raw(i,1)+GyZ2_raw(i,1))/2 ;
end

% Fusion of Corrected MPU1 & MPU2 Data using Traditional Mean
AcX_crt = zeros(length(data),1) ;
AcY_crt = zeros(length(data),1) ;
AcZ_crt = zeros(length(data),1) ;
GyX_crt = zeros(length(data),1) ;
GyY_crt = zeros(length(data),1) ;
GyZ_crt = zeros(length(data),1) ;

i = 1 ;
for i = 1:length(data)
    AcX_crt(i,1) = (AcX1_crt(i,1)+AcX2_crt(i,1))/2 ;
    AcY_crt(i,1) = (AcY1_crt(i,1)+AcY2_crt(i,1))/2 ;
    AcZ_crt(i,1) = (AcZ1_crt(i,1)+AcZ2_crt(i,1))/2 ;
    GyX_crt(i,1) = (GyX1_crt(i,1)+GyX2_crt(i,1))/2 ;
    GyY_crt(i,1) = (GyY1_crt(i,1)+GyY2_crt(i,1))/2 ;
    GyZ_crt(i,1) = (GyZ1_crt(i,1)+GyZ2_crt(i,1))/2 ;
end

%% PLOTS - FIELD TEST
% close all
% Roundabout 1
figure
%plot speed
subplot(241)
hold on
grid on
grid minor
plot(timestamp, U_km, 'r-')
plot(timestamp, U_ka, 'g-')
title('Field Test Forward Speed')
ylabel('Speed (km/h)')
xlabel('Time Stamp (seconds)')
legend('Real Time','Mvg. Avg.','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcX
subplot(242)
hold on
grid on
grid minor
plot(timestamp, AcX1_raw, 'g-')
plot(timestamp, AcX2_raw, 'b-')
plot(timestamp, AcX_raw, 'r-')
title('Field Test X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(243)
hold on
grid on
grid minor
plot(timestamp, AcY1_raw, 'g-')
plot(timestamp, AcY2_raw, 'b-')
plot(timestamp, AcY_raw, 'r-')
title('Field Test Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(244)
hold on
grid on
grid minor
plot(timestamp, AcZ1_raw, 'g-')
plot(timestamp, AcZ2_raw, 'b-')
plot(timestamp, AcZ_raw, 'r-')
title('Field Test Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(246)
hold on
grid on
grid minor
plot(timestamp, GyX1_raw, 'g-')
plot(timestamp, GyX2_raw, 'b-')
plot(timestamp, GyX_raw, 'r-')
title('Field Test X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(247)
hold on
grid on
grid minor
plot(timestamp, GyY1_raw, 'g-')
plot(timestamp, GyY2_raw, 'b-')
plot(timestamp, GyY_raw, 'r-')
title('Field Test Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(248)
hold on
grid on
grid minor
plot(timestamp, GyZ1_raw, 'g-')
plot(timestamp, GyZ2_raw, 'b-')
plot(timestamp, GyZ_raw, 'r-')
title('Field Test Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot Temp
subplot(245)
hold on
grid on
grid minor
plot(timestamp, Temp1, 'g-')
plot(timestamp, Temp2, 'b-')
title('Field Test Temperature')
ylabel('Temperature (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','location','best')
xlim([225,270])
% ylim([0,14])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Roundabout 2
figure
%plot speed
subplot(241)
hold on
grid on
grid minor
plot(timestamp, U_km, 'r-')
plot(timestamp, U_ka, 'g-')
title('Field Test Forward Speed')
ylabel('Speed (km/h)')
xlabel('Time Stamp (seconds)')
legend('Real Time','Mvg. Avg.','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcX
subplot(242)
hold on
grid on
grid minor
plot(timestamp, AcX1_raw, 'g-')
plot(timestamp, AcX2_raw, 'b-')
plot(timestamp, AcX_raw, 'r-')
title('Field Test X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(243)
hold on
grid on
grid minor
plot(timestamp, AcY1_raw, 'g-')
plot(timestamp, AcY2_raw, 'b-')
plot(timestamp, AcY_raw, 'r-')
title('Field Test Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(244)
hold on
grid on
grid minor
plot(timestamp, AcZ1_raw, 'g-')
plot(timestamp, AcZ2_raw, 'b-')
plot(timestamp, AcZ_raw, 'r-')
title('Field Test Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(246)
hold on
grid on
grid minor
plot(timestamp, GyX1_raw, 'g-')
plot(timestamp, GyX2_raw, 'b-')
plot(timestamp, GyX_raw, 'r-')
title('Field Test X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(247)
hold on
grid on
grid minor
plot(timestamp, GyY1_raw, 'g-')
plot(timestamp, GyY2_raw, 'b-')
plot(timestamp, GyY_raw, 'r-')
title('Field Test Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(248)
hold on
grid on
grid minor
plot(timestamp, GyZ1_raw, 'g-')
plot(timestamp, GyZ2_raw, 'b-')
plot(timestamp, GyZ_raw, 'r-')
title('Field Test Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot Temp
subplot(245)
hold on
grid on
grid minor
plot(timestamp, Temp1, 'g-')
plot(timestamp, Temp2, 'b-')
title('Field Test Temperature')
ylabel('Temperature (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','location','best')
xlim([675,720])
% ylim([0,14])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% CORRECTED
% Roundabout 1
figure
%plot speed
subplot(241)
hold on
grid on
grid minor
plot(timestamp, U_km, 'r-')
plot(timestamp, U_ka, 'g-')
title('Field Test Forward Speed')
ylabel('Speed (km/h)')
xlabel('Time Stamp (seconds)')
legend('Real Time','Mvg. Avg.','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcX
subplot(242)
hold on
grid on
grid minor
plot(timestamp, AcX1_crt, 'g-')
plot(timestamp, AcX2_crt, 'b-')
plot(timestamp, AcX_crt, 'r-')
title('Field Test X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(243)
hold on
grid on
grid minor
plot(timestamp, AcY1_crt, 'g-')
plot(timestamp, AcY2_crt, 'b-')
plot(timestamp, AcY_crt, 'r-')
title('Field Test Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(244)
hold on
grid on
grid minor
plot(timestamp, AcZ1_crt, 'g-')
plot(timestamp, AcZ2_crt, 'b-')
plot(timestamp, AcZ_crt, 'r-')
title('Field Test Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(246)
hold on
grid on
grid minor
plot(timestamp, GyX1_crt, 'g-')
plot(timestamp, GyX2_crt, 'b-')
plot(timestamp, GyX_crt, 'r-')
title('Field Test X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(247)
hold on
grid on
grid minor
plot(timestamp, GyY1_crt, 'g-')
plot(timestamp, GyY2_crt, 'b-')
plot(timestamp, GyY_crt, 'r-')
title('Field Test Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(248)
hold on
grid on
grid minor
plot(timestamp, GyZ1_crt, 'g-')
plot(timestamp, GyZ2_crt, 'b-')
plot(timestamp, GyZ_crt, 'r-')
title('Field Test Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot Temp
subplot(245)
hold on
grid on
grid minor
plot(timestamp, Temp1, 'g-')
plot(timestamp, Temp2, 'b-')
title('Field Test Temperature')
ylabel('Temperature (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','location','best')
xlim([225,270])
% ylim([0,14])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Roundabout 2
figure
%plot speed
subplot(241)
hold on
grid on
grid minor
plot(timestamp, U_km, 'r-')
plot(timestamp, U_ka, 'g-')
title('Field Test Forward Speed')
ylabel('Speed (km/h)')
xlabel('Time Stamp (seconds)')
legend('Real Time','Mvg. Avg.','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcX
subplot(242)
hold on
grid on
grid minor
plot(timestamp, AcX1_crt, 'g-')
plot(timestamp, AcX2_crt, 'b-')
plot(timestamp, AcX_crt, 'r-')
title('Field Test X-Axis Acceleration')
ylabel('X-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcY
subplot(243)
hold on
grid on
grid minor
plot(timestamp, AcY1_crt, 'g-')
plot(timestamp, AcY2_crt, 'b-')
plot(timestamp, AcY_crt, 'r-')
title('Field Test Y-Axis Acceleration')
ylabel('Y-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot AcZ
subplot(244)
hold on
grid on
grid minor
plot(timestamp, AcZ1_crt, 'g-')
plot(timestamp, AcZ2_crt, 'b-')
plot(timestamp, AcZ_crt, 'r-')
title('Field Test Z-Axis Acceleration')
ylabel('Z-Axis Acceleration (m/s^2)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyX
subplot(246)
hold on
grid on
grid minor
plot(timestamp, GyX1_crt, 'g-')
plot(timestamp, GyX2_crt, 'b-')
plot(timestamp, GyX_crt, 'r-')
title('Field Test X-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyY
subplot(247)
hold on
grid on
grid minor
plot(timestamp, GyY1_crt, 'g-')
plot(timestamp, GyY2_crt, 'b-')
plot(timestamp, GyY_crt, 'r-')
title('Field Test Y-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot GyZ
subplot(248)
hold on
grid on
grid minor
plot(timestamp, GyZ1_crt, 'g-')
plot(timestamp, GyZ2_crt, 'b-')
plot(timestamp, GyZ_crt, 'r-')
title('Field Test Z-Axis Gyro')
ylabel('Angular Rate (rad/s)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%plot Temp
subplot(245)
hold on
grid on
grid minor
plot(timestamp, Temp1, 'g-')
plot(timestamp, Temp2, 'b-')
title('Field Test Temperature')
ylabel('Temperature (deg)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','location','best')
xlim([675,720])
% ylim([0,14])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% ----- DERRIVED DATA
%% SMALL ROLL ANGLE APPROXIMATION (FROM ANGULAR RATE)

% raw
phi_d1r = zeros(length(GyY1_raw),1) ;
phi_d2r = zeros(length(GyY2_raw),1) ;
phi_dr = zeros(length(GyY_raw),1) ;

for ii = 1:length(GyY1_raw)
    phi_d1r(ii,1) = atan((GyY1_raw(ii,1)*U_ma(ii,1))/g) ; % 
    phi_d2r(ii,1) = atan((GyY2_raw(ii,1)*U_ma(ii,1))/g) ; % 
    phi_dr(ii,1) = atan((GyY_raw(ii,1)*U_ma(ii,1))/g) ; % 
end

% corrected
phi_d1c = zeros(length(GyY1_crt),1) ;
phi_d2c = zeros(length(GyY2_crt),1) ;
phi_dc = zeros(length(GyY_crt),1) ;

for ii = 1:length(GyY1_crt)
    phi_d1c(ii,1) = atan((GyY1_crt(ii,1)*U_ma(ii,1))/g) ; % 
    phi_d2c(ii,1) = atan((GyY2_crt(ii,1)*U_ma(ii,1))/g) ; % 
    phi_dc(ii,1) = atan((GyY_crt(ii,1)*U_ma(ii,1))/g) ; % 
end

%% LARGE ROLL ANGLE APPROXIMATION

% raw
phi_w1r = zeros(length(GyY1_raw),1) ;
phi_w2r = zeros(length(GyY2_raw),1) ;
phi_wr = zeros(length(GyY_raw),1) ;

for iii = 1:length(GyY1_raw)
    phi_w1r(iii,1) = -sign(GyY1_raw(iii,1))*asin(GyX1_raw(iii,1)/sqrt((GyX1_raw(iii,1)^2)+(GyY1_raw(iii,1)^2))) ;
    phi_w2r(iii,1) = -sign(GyY2_raw(iii,1))*asin(GyX2_raw(iii,1)/sqrt((GyX2_raw(iii,1)^2)+(GyY2_raw(iii,1)^2))) ;
    phi_wr(iii,1) = -sign(GyY_raw(iii,1))*asin(GyX_raw(iii,1)/sqrt((GyX_raw(iii,1)^2)+(GyY_raw(iii,1)^2))) ;
    
%     phi_w1r(iii,1) = -atan(GyX1_raw(iii,1)/GyY1_raw(iii,1)) ;
%     phi_w2r(iii,1) = -atan(GyX2_raw(iii,1)/GyY2_raw(iii,1)) ;
%     phi_wr(iii,1) = -atan(GyX_raw(iii,1)/GyY_raw(iii,1)) ;
end

% corrected
phi_w1c = zeros(length(GyY1_crt),1) ;
phi_w2c = zeros(length(GyY2_crt),1) ;
phi_wc = zeros(length(GyY_crt),1) ;

for iii = 1:length(GyY1_crt)
    phi_w1c(iii,1) = -sign(GyY1_crt(iii,1))*asin(GyX1_crt(iii,1)/sqrt((GyX1_crt(iii,1)^2)+(GyY1_crt(iii,1)^2))) ;
    phi_w2c(iii,1) = -sign(GyY2_crt(iii,1))*asin(GyX2_crt(iii,1)/sqrt((GyX2_crt(iii,1)^2)+(GyY2_crt(iii,1)^2))) ;
    phi_wc(iii,1) = -sign(GyY_crt(iii,1))*asin(GyX_crt(iii,1)/sqrt((GyX_crt(iii,1)^2)+(GyY_crt(iii,1)^2))) ;
    
%     phi_w1c(iii,1) = -atan(GyX1_crt(iii,1)/GyY1_crt(iii,1)) ;
%     phi_w2c(iii,1) = -atan(GyX2_crt(iii,1)/GyY2_crt(iii,1)) ;
%     phi_wc(iii,1) = -atan(GyX_crt(iii,1)/GyY_crt(iii,1)) ;
end

%% PLOTS
close all
% Raw
% Roundabout 1
figure
subplot(121)
hold on
grid on
grid minor
plot(timestamp, phi_d1r, 'g-')
plot(timestamp, phi_d2r, 'b-')
plot(timestamp, phi_dr, 'r-')
title('Small Roll Angle Approximation')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(122)
hold on
grid on
grid minor
plot(timestamp, phi_w1r, 'g-')
plot(timestamp, phi_w2r, 'b-')
plot(timestamp, phi_wr, 'r-')
title('Large Roll Angle Approximation')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Roundabout 2
figure
subplot(121)
hold on
grid on
grid minor
plot(timestamp, phi_d1r, 'g-')
plot(timestamp, phi_d2r, 'b-')
plot(timestamp, phi_dr, 'r-')
title('Small Roll Angle Approximation')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(122)
hold on
grid on
grid minor
plot(timestamp, phi_w1r, 'g-')
plot(timestamp, phi_w2r, 'b-')
plot(timestamp, phi_wr, 'r-')
title('Large Roll Angle Approximation')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Corrected
% Roundabout 1
figure
subplot(121)
hold on
grid on
grid minor
plot(timestamp, phi_d1c, 'g-')
plot(timestamp, phi_d2c, 'b-')
plot(timestamp, phi_dc, 'r-')
title('Corrected Small Roll Angle Approx.')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(122)
hold on
grid on
grid minor
plot(timestamp, phi_w1c, 'g-')
plot(timestamp, phi_w2c, 'b-')
plot(timestamp, phi_wc, 'r-')
title('Corrected Large Roll Angle Approx.')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

% Roundabout 2
figure
subplot(121)
hold on
grid on
grid minor
plot(timestamp, phi_d1c, 'g-')
plot(timestamp, phi_d2c, 'b-')
plot(timestamp, phi_dc, 'r-')
title('Corrected Small Roll Angle Approx.')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(122)
hold on
grid on
grid minor
plot(timestamp, phi_w1c, 'g-')
plot(timestamp, phi_w2c, 'b-')
plot(timestamp, phi_wc, 'r-')
title('Corrected Large Roll Angle Approx.')
ylabel('Roll Angle (rad)')
xlabel('Time Stamp (seconds)')
legend('MPU1 Data','MPU2 Data','Fused Data','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% ------- FILTERING
%% BLENDING FUNCTION
% close all

% Blending Fcn
phi_bar = sqrt(0.05) ;
phi_est = deg2rad([-60:0.5:60]') ;

W_bfce = zeros(length(phi_est),1) ;

i = 1 ;
for i = 1:length(phi_est)
    W_bfce(i,1) = exp(-(phi_est(i,1)^2)/(phi_bar^2)) ;
end

figure
hold on
grid on
grid minor
plot(rad2deg(phi_est), W_bfce, 'r-')
title('Blending Co-Efficient, W')
ylabel('Blending Co-efficient, W')
xlabel('Angle (deg)')
% xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% KALMAN SETUP
 
phi_bar = sqrt(0.05) ;

% Define Covariance Matricies of Plant and Sensor Noise

Q_1 = [10e-7,0;
        0,1e-8] ;
Q_2 = [10e-7,0;
        0,1e-8] ;
Q_m = [10e-7,0;
        0,1e-8] ;

% Q_1 = [5e-7,0;
%         0,1e-8] ;
% Q_2 = [5e-7,0;
%         0,1e-8] ;
% Q_m = [5e-7,0;
%         0,1e-8] ;

R_1 = cov(GyZ1_stac) ;
R_2 = cov(GyZ2_stac) ;
R_m = cov(GyZ_stac) ;

% PREALLOCATION
xkp_1 = zeros(2,length(GyZ1_crt)) ;
xk_1 = zeros(2,length(GyZ1_crt)) ;

xkp_2 = zeros(2,length(GyZ1_crt)) ;
xk_2 = zeros(2,length(GyZ1_crt)) ;

xkp_w = zeros(2,length(GyZ1_crt)) ;
xk_w = zeros(2,length(GyZ1_crt)) ;

W = zeros(length(GyZ1_crt),1) ;
W(1,1) = 1 ;

% Set initial values xk (estimate) & Pk (prediction)
xkp_1(1,1) = 0 ;
xkp_1(2,:) = ones(1,length(GyZ1_crt)).*GyZ1_biac ;
xkp_2(1,1) = 0 ;
xkp_2(2,:) = ones(1,length(GyZ1_crt)).*GyZ1_biac ;
xkp_w(1,1) = 0 ;
xkp_w(2,:) = ones(1,length(GyZ1_crt)).*GyZ1_biac ;

xk_1(:,1) = [0 ;
            GyZ1_biac] ;
xk_2(:,1) = [0 ;
            GyZ1_biac] ;
xk_w(:,1) = [0 ;
            GyZ1_biac] ;

Pk_1 = [5e-7,0;
        0,1e-8] ;
Pk_2 = [5e-7,0;
        0,1e-8] ;
Pk_w = [5e-7,0;
        0,1e-8] ;

% Define Model Inputs, U - sensor roll rate (rad/s)
U = GyZ1_crt ;

% Define Input Measurements - must be an angle (rad) not a "rate" (rad/s)
zk_1 = phi_d1c ; % MPU1 Small Angle Approximation, SAA
zk_2 = phi_w1c ; % MPU1 Large Angle Approximation, LAA
zk_w = zeros(length(U),1) ; 

%% KALMAN ALGORTHM

for f1 = 2:length(U)
% SETUP
    dt = 0.04 ;
%     dt = mean(dtv)
%     dt = dtv(iv-1,1) ;

    % Define System Model Variables
    A = [1,-dt;
    0,1] ;

    G = [dt;0] ;
    
    H = 1 ;
    
% 1) Predict state (xk) and error co-variance (Pk)
    xkp_1(:,f1) = A*xk_1(:,f1-1) + G*U(f1-1,1) ;
    xkp_2(:,f1) = A*xk_2(:,f1-1) + G*U(f1-1,1) ;
    xkp_w(:,f1) = A*xk_w(:,f1-1) + G*U(f1-1,1) ;
    
%   New 'measurement' of roll angle  
    W(f1,1) = exp(-(xkp_w(1,f1)^2)/(phi_bar^2)) ; % Blending functio co-eff.
    zk_w(f1,1) = W(f1,1)*phi_d1c(f1,1) + (1 - W(f1,1))*phi_w1c(f1,1) ;
    
%     xkp_1(2,f1) = GyZ1_biac ;
%     xkp_2(2,f1) = GyZ1_biac ;
    
    Pkp_1 = A*Pk_1*A' + Q_1 ;
    Pkp_2 = A*Pk_2*A' + Q_2 ;
    Pkp_w = A*Pk_w*A' + Q_1 ;
    
% 2) Compute Kalman gain (Kk)
    Kk_1 = (Pkp_1*H') / (H*Pkp_1*H' + R_1) ;
    Kk_2 = (Pkp_2*H') / (H*Pkp_2*H' + R_2) ;
    Kk_w = (Pkp_w*H') / (H*Pkp_w*H' + R_1) ;
    
% 3) Compute estimate (xhatk) <------------- Feed in measurement
    xk_1(:,f1) = xkp_1(:,f1) + Kk_1*(zk_1(f1,1) - H*xkp_1(:,f1)) ;
    xk_2(:,f1) = xkp_2(:,f1) + Kk_2*(zk_2(f1,1) - H*xkp_2(:,f1)) ;
    xk_w(:,f1) = xkp_w(:,f1) + Kk_w*(zk_w(f1,1) - H*xkp_w(:,f1)) ;
    

%     xk_1(2,f1) = GyZ1_biac ;
%     xk_2(2,f1) = GyZ1_biac ;
    
% 4) Compute error covariance(Pk) ---------> Feed Pk back to step 1 as Pk-1
    Pk_1 = Pkp_1 - Kk_1*H*Pkp_1 ;
    Pk_2 = Pkp_2 - Kk_2*H*Pkp_2 ;
    Pk_w = Pkp_w - Kk_w*H*Pkp_w ;
end


%% PLOT W (BLENDING CO-EFF.)
% close all
figure
subplot(1,2,1)
hold on
grid on
grid minor
plot(timestamp, W, 'r-')
title('Blending Co-Efficient, W')
ylabel('Blending Co-efficient, W')
xlabel('Time Stamp (seconds)')
% legend('MPU1 Raw','MPU2 Raw','Fused Raw','location','best')
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(1,2,2)
hold on
grid on
grid minor
plot(timestamp, W, 'r-')
title('Blending Co-Efficient, W')
ylabel('Blending Co-efficient, W')
xlabel('Time Stamp (seconds)')
% legend('MPU1 Raw','MPU2 Raw','Fused Raw','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% KALMAN PLOT - RAA
figure
subplot(1,2,1)
hold on
grid on
grid minor
% plot(timestamp, xkp_m(1,:), 'r-') %rad
% plot(timestamp, xkp_1(1,:), 'g-')
% plot(timestamp, xkp_2(1,:), 'b-')
% plot(timestamp, rad2deg(xkp_m(1,:)), 'r-') %deg
plot(timestamp, rad2deg(xkp_1(1,:)), 'g-')
plot(timestamp, rad2deg(xkp_2(1,:)), 'b-')
title('Kalman Prediction, "xkp"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('SAA','LAA','location','best')
% xlim([0,300])
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(1,2,2)
hold on
grid on
grid minor
% plot(timestamp, xk_m(1,:), 'r-')            %rad
% plot(timestamp, xk_1(1,:), 'g-')
% plot(timestamp, xk_2(1,:), 'b-')
% plot(timestamp, rad2deg(xk_m(1,:)), 'r-')   %deg
plot(timestamp, rad2deg(xk_1(1,:)), 'g-')
plot(timestamp, rad2deg(xk_2(1,:)), 'b-')
title('Kalman Filter, "xk"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('SAA','LAA','location','best')
% xlim([0,300])
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

figure
subplot(1,2,1)
hold on
grid on
grid minor
% plot(timestamp, xkp_m(1,:), 'r-')           %rad
% plot(timestamp, xkp_1(1,:), 'g-')
% plot(timestamp, xkp_2(1,:), 'b-')
% plot(timestamp, rad2deg(xkp_m(1,:)), 'r-')  %deg
plot(timestamp, rad2deg(xkp_1(1,:)), 'g-')
plot(timestamp, rad2deg(xkp_2(1,:)), 'b-')
title('Kalman Prediction, "xkp"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('SAA','LAA','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(1,2,2)
hold on
grid on
grid minor
% plot(timestamp, xk_m(1,:), 'r-')          %rad
% plot(timestamp, xk_1(1,:), 'g-')
% plot(timestamp, xk_2(1,:), 'b-')
% plot(timestamp, rad2deg(xk_m(1,:)), 'r-')   %deg
plot(timestamp, rad2deg(xk_1(1,:)), 'g-')
plot(timestamp, rad2deg(xk_2(1,:)), 'b-')
title('Kalman Filter, "xk"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('SAA','LAA','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

%% KALMAN PLOT - KF AND BLENDED MEASUREMENTS
% close all

figure
subplot(1,2,1)
hold on
grid on
grid minor
% plot(timestamp, xkp_m(1,:), 'r-') %rad
% plot(timestamp, xkp_1(1,:), 'g-')
% plot(timestamp, xkp_2(1,:), 'b-')
% plot(timestamp, rad2deg(xkp_1(1,:)), 'g-')
% plot(timestamp, rad2deg(xkp_2(1,:)), 'b-')
plot(timestamp, rad2deg(zk_w), 'r-')
plot(timestamp, rad2deg(xkp_w(1,:)), 'k-')
title('Kalman Prediction, "xkp"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('"Measurement"','Filter Estimate','location','best')
% xlim([0,300])
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(1,2,2)
hold on
grid on
grid minor
% plot(timestamp, xk_m(1,:), 'r-')            %rad
% plot(timestamp, xk_1(1,:), 'g-')
% plot(timestamp, xk_2(1,:), 'b-')
% plot(timestamp, rad2deg(xkp_1(1,:)), 'g-')
% plot(timestamp, rad2deg(xkp_2(1,:)), 'b-')
plot(timestamp, rad2deg(zk_w), 'r-')
plot(timestamp, rad2deg(xk_w(1,:)), 'k-')
title('Kalman Filter, "xk"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('"Measurement"','Filter Estimate','location','best')
% xlim([0,300])
xlim([225,270])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')
savefig('Roundabouts - First Pass')

figure
subplot(1,2,1)
hold on
grid on
grid minor
% plot(timestamp, xkp_m(1,:), 'r-')           %rad
% plot(timestamp, xkp_1(1,:), 'g-')
% plot(timestamp, xkp_2(1,:), 'b-')
% plot(timestamp, rad2deg(xkp_1(1,:)), 'g-')
% plot(timestamp, rad2deg(xkp_2(1,:)), 'b-')
plot(timestamp, rad2deg(zk_w), 'r-')
plot(timestamp, rad2deg(xkp_w(1,:)), 'k-')
title('Kalman Prediction, "xkp"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('"Measurement"','Filter Estimate','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')

subplot(1,2,2)
hold on
grid on
grid minor
% plot(timestamp, xk_m(1,:), 'r-')          %rad
% plot(timestamp, xk_1(1,:), 'g-')
% plot(timestamp, xk_2(1,:), 'b-')
% plot(timestamp, rad2deg(xkp_1(1,:)), 'g-')
% plot(timestamp, rad2deg(xkp_2(1,:)), 'b-')
plot(timestamp, rad2deg(zk_w), 'r-')
plot(timestamp, rad2deg(xk_w(1,:)), 'k-')
title('Kalman Filter, "xk"')
ylabel('Roll Angle')
xlabel('Time Stamp (seconds)')
legend('"Measurement"','Filter Estimate','location','best')
xlim([675,720])
% ylim([0,45])
% set(gca,'XAxisLocation','origin')
% set(gca,'YAxisLocation','origin')
savefig('Roundabouts - Second Pass')
    
%% FOOTER
disp(' ')
disp('Script written by:')
disp('    Vincent Renders, B627526')
disp('    Part C - BEng. Aeronautical Engineering')
disp('    Loughborough University')
disp(' ')
disp('Supervised by:')
disp('    Dr George Mavros MSc, PhD')
disp('    Senior Lecturer')
disp('    Dept. of Aeronautical & Automotive Engineering')
disp('    Loughborough University')
toc