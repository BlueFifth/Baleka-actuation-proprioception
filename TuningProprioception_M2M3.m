clc;
clear;
Tuning2 = load("Data/M2M3ID_Test5_actuation_compensated.mat");
Tuning2 = Tuning2.data;
%%
% Search for testing times via force plate z readings:
FPz = Tuning2{3}.Values;

Start_times = [];
End_times = [];
valid = false;
for i =1:length(FPz.Data)
    if valid
        if FPz.Data(i)>-5
            End_times = [End_times, FPz.Time(i)-5]; % Chop off the robot being picked up
            valid = false;
        end    
    else
        if FPz.Data(i)<-30
            Start_times = [Start_times, FPz.Time(i)+5];% Chop off the robot being put down
            valid = true;
        end
    end
end
% End_times(5) = End_times(5); % Strange artifact at the end of final sample - something went wring with lifting off the force plate.

%%
Test_time = 0;
for i = 1:length(Start_times)
 Test_time = Test_time + End_times(i) - Start_times(i);
end

%%
% Torques calculated via force plate forces using the jacobian, 
FPy = [];
FPz = [];
MRq = [];
MLq = [];
MRt_read = [];
MLt_read = [];
MRt_cmd = [];
MLt_cmd = [];
ts = 1000;
Start_times =int32(Start_times*ts);
End_times = int32(End_times*ts);



% Resamples
MR_q_resample = resample(Tuning2{5}.Values.q, Tuning2{1}.Values.Time);
ML_q_resample = resample(Tuning2{6}.Values.q, Tuning2{1}.Values.Time);
MR_t_resample = resample(Tuning2{5}.Values.t, Tuning2{1}.Values.Time);
ML_t_resample = resample(Tuning2{6}.Values.t, Tuning2{1}.Values.Time);
MR_t_cmd_resample = resample(Tuning2{9}.Values.tff, Tuning2{1}.Values.Time);
ML_t_cmd_resample = resample(Tuning2{8}.Values.tff, Tuning2{1}.Values.Time);
for i = 1:length(Start_times)
    % Force plate
    FPy = [FPy; Tuning2{2}.Values.Data(Start_times(i): End_times(i))];
    FPz = [FPz; Tuning2{3}.Values.Data(Start_times(i): End_times(i))];
    % Motor angles
    
    MRq = [MRq; MR_q_resample.Data(Start_times(i): End_times(i))];
    MLq = [MLq; ML_q_resample.Data(Start_times(i): End_times(i))];

    % Motor torques (cmd)
    
    MRt_cmd = [MRt_cmd; MR_t_cmd_resample.Data(Start_times(i): End_times(i))]; 
    MLt_cmd = [MLt_cmd; ML_t_cmd_resample.Data(Start_times(i): End_times(i))];
    
    % Motor torques (read)
   
    MRt_read = [MRt_read; MR_t_resample.Data(Start_times(i): End_times(i))];
    MLt_read = [MLt_read; ML_t_resample.Data(Start_times(i): End_times(i))];

end
clear MR_q_resample ML_q_resample MR_t_resample ML_t_resample MR_t_cmd_resample ML_t_cmd_resample

%%
%[text] ## Transfer Motor torques into forces
% From the side of forces:

Fy_read = [];
Fz_read = [];
FP_tl = [];
FP_tr = [];
[J11, J12, J21, J22] = BalekaJacobianXY(MLq, MRq);
for i = 1:length(FPz)
    Jxy = [J11(i), J12(i); J21(i), J22(i)];
    JTxy = Jxy';

    F_read = JTxy\[MLt_read(i)*9*0.198; MRt_read(i)*9*0.198];
    Fy_read = [Fy_read; F_read(1)];
    Fz_read = [Fz_read; F_read(2)];
end
clear F_cmd F_read i J11 J12 J22 J21 times ts JTxy Jxy

%%
% forces:
plot(FPz)
hold on
plot(Fz_read)
legend('Force plate z', 'Motor read z');
title('Fz')
hold off

% errFzRead = mean((Fz_read - FPz).^2)
MSE_FzReadM2M3 =  mean((Fz_read - FPz).^2)
plot(FPy)
hold on
plot(Fy_read)
legend('Force plate y','Force plate Horizontal', 'Motor read Fy');
title('Horizontal force')
hold off
% errFxyRead = mean((Fy_read - FPy).^2)
MSE_FyReadM2M3 =  mean((Fy_read - FPy).^2)
%%
% Z force

[coeff, Sr] = polyfit(Fz_read, FPz, 1);
xFit = linspace(min(Fz_read), max(Fz_read), 1000);
yFit = polyval(coeff, xFit);

scatter(Fz_read, FPz, 'b','filled')
hold on
plot(xFit, yFit, 'r-', LineWidth=2)
title("Force plate vertical force vs motor sensed vertical force");
ylabel("Force plate force (N)");
xlabel("Motor sensed force (N)");
legend("Measured data", "Line of best fit")
axis([-500 0 -inf inf])
hold off
m1_vert = coeff(1)
c1_vert = coeff(2)
R1_vert = Sr.rsquared
%where line of best fit is y=mx +c

% Y force
[coeff, Sr] = polyfit(Fy_read, FPy, 1);
xFit = linspace(min(Fy_read), max(Fy_read), 1000);
yFit = polyval(coeff, xFit);

scatter(Fy_read, FPy, 'b','filled')
hold on
plot(xFit, yFit, 'r-', LineWidth=2)
title("Force plate horizontal force vs motor sensed horizontal force");
ylabel("Force plate force (N)");
xlabel("Motor measured force (N)");
legend("Measured data", "Line of best fit")
hold off
m2_hor = coeff(1)
c2_hor = coeff(2)
R2_hor = Sr.rsquared
Rsq = corrcoef(FPy, Fy_read).^2
%where line of best fit is y=mx +c

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.4}
%---
