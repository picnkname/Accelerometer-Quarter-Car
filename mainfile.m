%
% Figure-eight Track Simulation Demo
%
clear all;
close all;
Speed           = 90;
TrackLength     = Speed*60;
CrossOverHeight = 0.001;
Pinfty = zeros(9);
k  = 0;
dt = .01;
GCorrection = [zeros(8,1);9.8];
RMSDeltaV   = zeros(3,1);
Zposn=[0 0];Zvel=[0 0];
for t=0:dt:10,
    k = k + 1;
    Time(k) = t;
    VehState   = Fig8TrackSim(t,TrackLength,Speed,CrossOverHeight);
    if k>1
        DeltaVN(k) = VehState(4) - VelN(k-1);
        DeltaVE(k) = VehState(5) - VelE(k-1);
        DeltaVD(k) = VehState(6) - VelD(k-1);
        RMSDeltaV  = RMSDeltaV + [DeltaVN(k)^2;DeltaVE(k)^2;DeltaVD(k)^2];
    end;
    PosN(k)    = VehState(1);
    PosE(k)    = VehState(2);
    PosD(k)    = VehState(3);
    VelN(k)    = VehState(4);
    VelE(k)    = VehState(5);
    VelD(k)    = VehState(6);
    AccN(k)    = VehState(7);
    AccE(k)    = VehState(8);
    AccD(k)    = VehState(9);
    Roll(k)    = VehState(10);
    Pitch(k)   = VehState(11);
    Heading(k) = VehState(12);
    RollRate(k) = VehState(13);
    PitchRate(k) = VehState(14);
    YawRate(k) = VehState(15);
    AccR(k)    = VehState(16);
    AccP(k)    = VehState(17);
    AccY(k)    = VehState(18);
    Pinfty     = Pinfty + (VehState(1:9)+GCorrection)*(VehState(1:9)+GCorrection)';
    Zposn=[Zposn;k -1*floor(VehState(3))];
    Zvel=[Zvel;k floor(VehState(6))];
end;
for t=10:dt:11,
    k = k + 1;
    Time(k) = t;CrossOverHeight=500;
    VehState   = Fig8TrackSim(t,TrackLength,Speed,CrossOverHeight);
    if k>1
        DeltaVN(k) = VehState(4) - VelN(k-1);
        DeltaVE(k) = VehState(5) - VelE(k-1);
        DeltaVD(k) = VehState(6) - VelD(k-1);
        RMSDeltaV  = RMSDeltaV + [DeltaVN(k)^2;DeltaVE(k)^2;DeltaVD(k)^2];
    end;
    PosN(k)    = VehState(1);
    PosE(k)    = VehState(2);
    PosD(k)    = VehState(3);
    VelN(k)    = VehState(4);
    VelE(k)    = VehState(5);
    VelD(k)    = VehState(6);
    AccN(k)    = VehState(7);
    AccE(k)    = VehState(8);
    AccD(k)    = VehState(9);
    Roll(k)    = VehState(10);
    Pitch(k)   = VehState(11);
    Heading(k) = VehState(12);
    RollRate(k) = VehState(13);
    PitchRate(k) = VehState(14);
    YawRate(k) = VehState(15);
    AccR(k)    = VehState(16);
    AccP(k)    = VehState(17);
    AccY(k)    = VehState(18);
    Pinfty     = Pinfty + (VehState(1:9)+GCorrection)*(VehState(1:9)+GCorrection)';
    Zposn=[Zposn;k -1*floor(VehState(3))];
    Zvel=[Zvel;k floor(VehState(6))];
end;
for t=11:dt:60,
    k = k + 1;
    Time(k) = t;CrossOverHeight=0.001;
    VehState   = Fig8TrackSim(t,TrackLength,Speed,CrossOverHeight);
    if k>1
        DeltaVN(k) = VehState(4) - VelN(k-1);
        DeltaVE(k) = VehState(5) - VelE(k-1);
        DeltaVD(k) = VehState(6) - VelD(k-1);
        RMSDeltaV  = RMSDeltaV + [DeltaVN(k)^2;DeltaVE(k)^2;DeltaVD(k)^2];
    end;
    PosN(k)    = VehState(1);
    PosE(k)    = VehState(2);
    PosD(k)    = VehState(3);
    VelN(k)    = VehState(4);
    VelE(k)    = VehState(5);
    VelD(k)    = VehState(6);
    AccN(k)    = VehState(7);
    AccE(k)    = VehState(8);
    AccD(k)    = VehState(9);
    Roll(k)    = VehState(10);
    Pitch(k)   = VehState(11);
    Heading(k) = VehState(12);
    RollRate(k) = VehState(13);
    PitchRate(k) = VehState(14);
    YawRate(k) = VehState(15);
    AccR(k)    = VehState(16);
    AccP(k)    = VehState(17);
    AccY(k)    = VehState(18);
    Pinfty     = Pinfty + (VehState(1:9)+GCorrection)*(VehState(1:9)+GCorrection)';
    Zposn=[Zposn;k -1*floor(VehState(3))];
    Zvel=[Zvel;k floor(VehState(6))];
end;
Zposn=[Zposn;k+1 0;k+2 0;k+3 0;k+4 0;k+5 0;k+6 0;k+7 0;k+8 0;k+9 0;k+10 0];
Zvel=[Zvel;k+1 0;k+2 0;k+3 0;k+4 0;k+5 0;k+6 0;k+7 0;k+8 0;k+9 0;k+10 0];

%
% Calculate RMS Vehicle State
%
NSamp     = k;
RMSDeltaV = sqrt(RMSDeltaV/(NSamp-1));
Pinfty    = Pinfty/(NSamp-1);
RMSVS     = sqrt([Pinfty(1,1);Pinfty(2,2);Pinfty(3,3);Pinfty(4,4);Pinfty(5,5);Pinfty(6,6);Pinfty(7,7);Pinfty(8,8);Pinfty(9,9)]);
disp(['RMS N-S Position Excursion   = ',num2str(RMSVS(1)),' meter']);
disp(['RMS E-W Position Excursion   = ',num2str(RMSVS(2)),' meter']);
disp(['RMS Vert. Position Excursion = ',num2str(RMSVS(3)),' meter']);
disp(['RMS N-S Velocity             = ',num2str(RMSVS(4)),' m/s']);
disp(['RMS E-W Velocity             = ',num2str(RMSVS(5)),' m/s']);
disp(['RMS Vert. Velocity           = ',num2str(RMSVS(6)),' m/s']);
disp(['RMS N-S Acceleration         = ',num2str(RMSVS(7)),' m/s/s']);
disp(['RMS E-W Acceleration         = ',num2str(RMSVS(8)),' m/s/s']);
disp(['RMS Vert. Acceleration       = ',num2str(RMSVS(9)),' m/s/s']);
disp(['RMS Delta Velocity North     = ',num2str(RMSDeltaV(1)),' m/s at Delta t = ',num2str(dt),' sec.']);
disp(['RMS Delta Velocity East      = ',num2str(RMSDeltaV(2)),' m/s at Delta t = ',num2str(dt),' sec.']);
disp(['RMS Delta Velocity Down      = ',num2str(RMSDeltaV(3)),' m/s at Delta t = ',num2str(dt),' sec.']);

% Calculate and plot autocorrelation functions

k = 0;
AutoCorr = zeros(9,floor(NSamp/2)+1);
for lag=0:floor(NSamp/2),
    k          = k + 1;
    LagTime(k) = lag*dt;
    N          = 0;
    for j=1:NSamp-lag;
        N             = N + 1;
        AutoCorr(1,k) = AutoCorr(1,k) + PosN(j)*PosN(j+lag);
        AutoCorr(2,k) = AutoCorr(2,k) + PosE(j)*PosE(j+lag);
        AutoCorr(3,k) = AutoCorr(3,k) + PosD(j)*PosD(j+lag);
        AutoCorr(4,k) = AutoCorr(4,k) + VelN(j)*VelN(j+lag);
        AutoCorr(5,k) = AutoCorr(5,k) + VelE(j)*VelE(j+lag);
        AutoCorr(6,k) = AutoCorr(6,k) + VelD(j)*VelD(j+lag);
        AutoCorr(7,k) = AutoCorr(7,k) + AccN(j)*AccN(j+lag);
        AutoCorr(8,k) = AutoCorr(8,k) + AccE(j)*AccE(j+lag);
        AutoCorr(9,k) = AutoCorr(9,k) + (AccD(j)+9.8)*(AccD(j+lag)+9.8);
    end;
    Threshold(k) = 1/exp(1);
    for j=1:9,
        AutoCorr(j,k) = AutoCorr(j,k)/N;
    end;
end;
NLags = k;
%
for j=1:9,
    Normalizer = AutoCorr(j,1);
    for k=1:NLags,
        AutoCorr(j,k) = AutoCorr(j,k)/Normalizer;
    end;
end;
TH      = 1/exp(1);
TauPosN = interp1(AutoCorr(1,:),LagTime,TH);
TauPosE = interp1(AutoCorr(2,:),LagTime,TH);
TauPosD = interp1(AutoCorr(3,:),LagTime,TH);
TauVelN = interp1(AutoCorr(4,:),LagTime,TH);
TauVelE = interp1(AutoCorr(5,:),LagTime,TH);
TauVelD = interp1(AutoCorr(6,:),LagTime,TH);
TauAccN = interp1(AutoCorr(7,:),LagTime,TH);
TauAccE = interp1(AutoCorr(8,:),LagTime,TH);
TauAccD = interp1(AutoCorr(9,:),LagTime,TH);
disp(['N. Position Correlation Time = ',num2str(TauPosN),' sec']);
disp(['E. Position Correlation Time = ',num2str(TauPosE),' sec']);
disp(['Vertical Position Corr. Time = ',num2str(TauPosD),' sec']);
disp(['N. Velocity Correlation Time = ',num2str(TauVelN),' sec']);
disp(['E. Velocity Correlation Time = ',num2str(TauVelE),' sec']);
disp(['Vertical Velocity Corr. Time = ',num2str(TauVelD),' sec']);
disp(['N. Acceler. Correlation Time = ',num2str(TauAccN),' sec']);
disp(['E. Acceler. Correlation Time = ',num2str(TauAccE),' sec']);
disp(['Vertical Acceler. Corr. Time = ',num2str(TauAccD),' sec']);
figure;
plot(LagTime,AutoCorr(1,:),'k-',LagTime,AutoCorr(2,:),'k--',LagTime,AutoCorr(3,:),'k:',LagTime,Threshold,'k-');
legend('N','E','D','1/e');
title('Position Autocorrelation Functions for Figure-8 Track Demo');
xlabel('Lag Time [sec]');
ylabel('Autocorrelation');
%
figure;
plot(LagTime,AutoCorr(4,:),'k-',LagTime,AutoCorr(5,:),'k--',LagTime,AutoCorr(6,:),'k:',LagTime,Threshold,'k-');
legend('N','E','D','1/e');
title('Velocity Autocorrelation Functions for Figure-8 Track Demo');
xlabel('Lag Time [sec]');
ylabel('Autocorrelation');
%
figure;
plot(LagTime,AutoCorr(7,:),'k-',LagTime,AutoCorr(8,:),'k--',LagTime,AutoCorr(9,:),'k:',LagTime,Threshold,'k-');
legend('N','E','D','1/e');
title('Acceleration Autocorrelation Functions for Figure-8 Track Demo');
xlabel('Lag Time [sec]');
ylabel('Autocorrelation');

% Plot 3D trajectory in ENU coordinates
%
[az,el] = view;
sA = sin(az*pi/180);
cA = cos(az*pi/180);
sE = sin(el*pi/180);
cE = cos(el*pi/180);
cuv = [sA*cE;-cA*cE;sE];
%result = notsofatarrow([PosE(1);PosN(1);-PosD(1)+CrossOverHeight/2],[PosE(1)+6*VelE(1);PosN(1)+6*VelN(1);-PosD(1)+CrossOverHeight/2-6*VelD(1)],cuv);
j=0; 
for t=0:50*dt:2
    subplot(4,2,1);
    j=j+50;
    plot3(PosE,PosN,-PosD+CrossOverHeight/2,'k-');
    title([num2str(TrackLength),'m figure-eight track with ',num2str(CrossOverHeight),'m crossover']);
    xlabel('Easting [m]');
    ylabel('Northing [m]');
    hold on;
    axis equal;
    plot3(PosE(j),PosN(j),-PosD(j)+CrossOverHeight/2,'ko');
    hold off;
    getframe;
    %
    subplot(4,2,2);
    plot(Time,PosN,'k-',Time,PosE,'k--',Time,PosD,'k:');
    ylabel('Position [m]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),PosN(j),'o',Time(j),PosE(j),'o',Time(j),PosD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,3);
    plot(Time,VelN,'k-',Time,VelE,'k--',Time,VelD,'k:');
    ylabel('Velocity [m/s]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),VelN(j),'o',Time(j),VelE(j),'o',Time(j),VelD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,4);
    plot(Time,AccN,'k-',Time,AccE,'k--',Time,AccD,'k:');
    ylabel('Acceleration[m/s/s]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),AccN(j),'o',Time(j),AccE(j),'o',Time(j),AccD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,5);
    plot(Time,180/pi*Roll,'k-',Time,180/pi*Pitch,'k--',Time,180/pi*Heading,'k:');
    ylabel('Vehicle Attitude[deg]');
    legend('Roll','Pitch','Heading');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),180/pi*Roll(j),'o',Time(j),180/pi*Pitch(j),'o',Time,180/pi*Heading(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,6);
    plot(Time,180/pi*RollRate,'k-',Time,180/pi*PitchRate,'k--',Time,180/pi*YawRate,'k:');
    ylabel('Rotation Rates [deg/sec]');
    legend('Roll','Pitch','Yaw');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),180/pi*RollRate(j),'o',Time(j),180/pi*PitchRate(j),'o',Time(j),180/pi*YawRate(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,7)
    plot(Time,AccR,'k-',Time,AccP,'k--',Time,AccY,'k:');
    ylabel('Acceleration in RPY Coordinates [m/s/s]');
    legend('Roll','Pitch','Yaw');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),AccR(j),'o',Time(j),AccP(j),'o',Time(j),AccY(j),'o');
    hold off;
    getframe;
end
for t=2:50*dt:2.2
    subplot(4,2,1);
    j=j+50;CrossOverHeight=500;
    plot3(PosE,PosN,-PosD+CrossOverHeight/2,'k-');
    title([num2str(TrackLength),'m figure-eight track with ',num2str(CrossOverHeight),'m crossover']);
    xlabel('Easting [m]');
    ylabel('Northing [m]');
    hold on;
    axis equal;
    plot3(PosE(j),PosN(j),-PosD(j)+CrossOverHeight/2,'ko');
    hold off;
    getframe;
    %
    subplot(4,2,2);
    plot(Time,PosN,'k-',Time,PosE,'k--',Time,PosD,'k:');
    ylabel('Position [m]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),PosN(j),'o',Time(j),PosE(j),'o',Time(j),PosD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,3);
    plot(Time,VelN,'k-',Time,VelE,'k--',Time,VelD,'k:');
    ylabel('Velocity [m/s]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),VelN(j),'o',Time(j),VelE(j),'o',Time(j),VelD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,4);
    plot(Time,AccN,'k-',Time,AccE,'k--',Time,AccD,'k:');
    ylabel('Acceleration[m/s/s]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),AccN(j),'o',Time(j),AccE(j),'o',Time(j),AccD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,5);
    plot(Time,180/pi*Roll,'k-',Time,180/pi*Pitch,'k--',Time,180/pi*Heading,'k:');
    ylabel('Vehicle Attitude[deg]');
    legend('Roll','Pitch','Heading');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),180/pi*Roll(j),'o',Time(j),180/pi*Pitch(j),'o',Time,180/pi*Heading(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,6);
    plot(Time,180/pi*RollRate,'k-',Time,180/pi*PitchRate,'k--',Time,180/pi*YawRate,'k:');
    ylabel('Rotation Rates [deg/sec]');
    legend('Roll','Pitch','Yaw');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),180/pi*RollRate(j),'o',Time(j),180/pi*PitchRate(j),'o',Time(j),180/pi*YawRate(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,7)
    plot(Time,AccR,'k-',Time,AccP,'k--',Time,AccY,'k:');
    ylabel('Acceleration in RPY Coordinates [m/s/s]');
    legend('Roll','Pitch','Yaw');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),AccR(j),'o',Time(j),AccP(j),'o',Time(j),AccY(j),'o');
    hold off;
    getframe;
end
for t=2.2:50*dt:11.999
    subplot(4,2,1);
    j=j+50;CrossOverHeight=0.001;
    plot3(PosE,PosN,-PosD+CrossOverHeight/2,'k-');
    title([num2str(TrackLength),'m figure-eight track with ',num2str(CrossOverHeight),'m crossover']);
    xlabel('Easting [m]');
    ylabel('Northing [m]');
    hold on;
    axis equal;
    plot3(PosE(j),PosN(j),-PosD(j)+CrossOverHeight/2,'ko');
    hold off;
    getframe;
    %
    subplot(4,2,2);
    plot(Time,PosN,'k-',Time,PosE,'k--',Time,PosD,'k:');
    ylabel('Position [m]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),PosN(j),'o',Time(j),PosE(j),'o',Time(j),PosD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,3);
    plot(Time,VelN,'k-',Time,VelE,'k--',Time,VelD,'k:');
    ylabel('Velocity [m/s]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),VelN(j),'o',Time(j),VelE(j),'o',Time(j),VelD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,4);
    plot(Time,AccN,'k-',Time,AccE,'k--',Time,AccD,'k:');
    ylabel('Acceleration[m/s/s]');
    legend('N','E','D');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),AccN(j),'o',Time(j),AccE(j),'o',Time(j),AccD(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,5);
    plot(Time,180/pi*Roll,'k-',Time,180/pi*Pitch,'k--',Time,180/pi*Heading,'k:');
    ylabel('Vehicle Attitude[deg]');
    legend('Roll','Pitch','Heading');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),180/pi*Roll(j),'o',Time(j),180/pi*Pitch(j),'o',Time,180/pi*Heading(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,6);
    plot(Time,180/pi*RollRate,'k-',Time,180/pi*PitchRate,'k--',Time,180/pi*YawRate,'k:');
    ylabel('Rotation Rates [deg/sec]');
    legend('Roll','Pitch','Yaw');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),180/pi*RollRate(j),'o',Time(j),180/pi*PitchRate(j),'o',Time(j),180/pi*YawRate(j),'o');
    hold off;
    getframe;
    %
    subplot(4,2,7)
    plot(Time,AccR,'k-',Time,AccP,'k--',Time,AccY,'k:');
    ylabel('Acceleration in RPY Coordinates [m/s/s]');
    legend('Roll','Pitch','Yaw');
    title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
    xlabel('Time [sec]');
    hold on;
    plot(Time(j),AccR(j),'o',Time(j),AccP(j),'o',Time(j),AccY(j),'o');
    hold off;
    getframe;
end


% eval('print -dbmp ''Fig8fig1.bmp'';');
% figure;
% plot(Time,PosN,'k-',Time,PosE,'k--',Time,PosD,'k:');
% ylabel('Position [m]');
% legend('N','E','D');
% title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
% xlabel('Time [sec]');
% eval('print -dbmp ''Fig8fig2.bmp'';');
% %
% figure;
% plot(Time,VelN,'k-',Time,VelE,'k--',Time,VelD,'k:');
% ylabel('Velocity [m/s]');
% legend('N','E','D');
% title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
% xlabel('Time [sec]');
% eval('print -dbmp ''Fig8fig3.bmp'';');
% %
% figure;
% plot(Time,AccN,'k-',Time,AccE,'k--',Time,AccD,'k:');
% ylabel('Acceleration in NED Coordinates [m/s/s]');
% legend('N','E','D');
% title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
% xlabel('Time [sec]');
% eval('print -dbmp ''Fig8fig4.bmp'';');
% %
% figure;
% plot(Time,180/pi*Roll,'k-',Time,180/pi*Pitch,'k--',Time,180/pi*Heading,'k:');
% ylabel('Vehicle Attitude in Euler Angles [deg]');
% legend('Roll','Pitch','Heading');
% title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
% xlabel('Time [sec]');
% eval('print -dbmp ''Fig8fig5.bmp'';');
% %
% figure;
% plot(Time,180/pi*RollRate,'k-',Time,180/pi*PitchRate,'k--',Time,180/pi*YawRate,'k:');
% ylabel('Vehicle-fixed Rotation Rates [deg/sec]');
% legend('Roll','Pitch','Yaw');
% title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
% xlabel('Time [sec]');
% eval('print -dbmp ''Fig8fig6.bmp'';');
% %
% figure;
% plot(Time,AccR,'k-',Time,AccP,'k--',Time,AccY,'k:');
% ylabel('Acceleration in RPY Coordinates [m/s/s]');
% legend('Roll','Pitch','Yaw');
% title([num2str(TrackLength),'m figure-eight track at ',num2str(Speed),' m/s']);
% xlabel('Time [sec]');
% eval('print -dbmp ''Fig8fig7.bmp'';