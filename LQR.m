%% Linear Quadratic Regulator Attitude Control
%% Name: Abinay Brown
%% Email: abrown472@gatech.edu/abinay.joel@gmail.com
%% Initial Conditions
%Quaternion
clear; clc;
q0 = 1;
q1 = 0;
q2 = 0;
q3 = 0;
w1 = 0; 
w2 = 0; 
w3 = 0;
state = [q0, q1, q2, q3, w1, w2, w3];
% Moment of Inertia
MOI = [84.7089, 84.7089, 169.4178];

%% Target Conditions
% Quaternion
qt1 = 0.4438259;
qt2 = -0.6166626;
qt3 = -0.2226721;
qt0 = 0.6108706;
% Angular Velocity
wt1 = 0;
wt2 = 0;
wt3 = 0;

%% Linearized 'A' Matrix about Target Cond.

A = [0, -wt1, -wt2, -wt3, -qt1, -qt2, -qt3;...
     wt1, 0, wt3, -wt2, qt0, -qt3, qt2;...
     wt2, -wt3, 0, wt1, qt3, qt0, -qt1;...
     wt3, wt2, -wt1, 0, -qt2, qt1, qt0;...
     0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 0, 0];
 B = [0, 0, 0;...
      0, 0, 0;...
      0, 0, 0;...
      0, 0, 0;...
      1/MOI(1), 0, 0;...
      0, 1/MOI(2), 0;...
      0, 0, 1/MOI(3)];
%% Q & R matrices for Performance VS. Effort
 Q = [12.5,0,0,0,0,0,0;...
      0,12.5,0,0,0,0,0;...
      0,0,12.5,0,0,0,0;...
      0,0,0,12.5,0,0,0;...
      0,0,0,0,1,0,0;...
      0,0,0,0,0,1,0;...
      0,0,0,0,0,0,1]; 
 R = 1*[1, 0, 0;...
      0, 1, 0;...
      0, 0, 1];
%% Finding Optimal Gain and Running Solver 
 K = lqr(A,B,Q,R);
 dt = 0.1;
 t = 0:dt:120;
 options = odeset('RelTol',1e-10,'AbsTol',1e-10); 
 [t, y] = ode113(@(t,state)diffEqn(state,K, MOI),t, state, options);
[t1, t2, t3] = torque(y, K);
%% Plotting Results
 subplot(3,2,1);
 plot(t,y(:,1), 'LineWidth', 1)
 yline(qt0,'r--', 'LineWidth', 1);
 grid on;
 xlabel("time (s)");
 ylabel(" q0 ");
 
 subplot(3,2,2);
 plot(t,y(:,2), 'LineWidth', 1)
 yline(qt1,'r--', 'LineWidth', 1);
 grid on;
 xlabel("time (s)");
 ylabel(" q1 ");
 
 subplot(3,2,3);
 plot(t,y(:,3), 'LineWidth', 1)
 yline(qt2,'r--', 'LineWidth', 1);
 grid on;
 xlabel("time (s)");
 ylabel(" q2 ");
 
 subplot(3,2,4);
 plot(t,y(:,4), 'LineWidth', 1)
 yline(qt3,'r--', 'LineWidth', 1);
 grid on;
 xlabel("time (s)");
 ylabel(" q3 ");
 
 subplot(3,2,5);
 plot(t,y(:,5), 'LineWidth', 1)
 hold on;
 plot(t,y(:,6), 'LineWidth', 1)
 hold on;
 plot(t,y(:,7), 'LineWidth', 1)
 grid on;
 xlabel("time (s)");
 ylabel(" Angular Velocity (rad/s) ");
 legend(["\omega_1", "\omega_2", "\omega_3"]);
 %% Power Req Calc
 omega = y(:, 5:7);
 Trot = zeros(1, length(omega));
 for i = 1:length(omega)
    Trot(i) = 0.5*omega(i, 1:3)*diag(MOI)*omega(i, 1:3)';
 end
 Power = Trot/dt;
 disp(max(Power));
 %% 
 subplot(3,2,6);
 len = length(t);
 plot(t(1:len),t1(1:len), 'LineWidth', 1)
 hold on;
 plot(t(1:len),t2(1:len), 'LineWidth', 1)
 hold on;
 plot(t(1:len),t3(1:len), 'LineWidth', 1)
 grid on;
 xlabel("time (s)");
 ylabel(" Control Torque (N-m) ");
 legend(["\tau_1", "\tau_2", "\tau_3"]);
 sgtitle('Sample Target Quaternion: q0 = 0.6108706, q1 = 0.4438259, q2 = -0.6166626, q3 = -0.2226721');
 
 fileID = fopen('QuaternionData.txt','w');
 for i = 1:length(y)
    fprintf(fileID, '%0.4f,%0.4f,%0.4f,%0.4f, \n', y(i,1), y(i,2),y(i,3),y(i,4));
 end
 fclose(fileID);
function [t1, t2, t3] = torque(state, K)
    %disp(state);
    % quaternions
    q0 = state(:,1);
    q1 = state(:,2);
    q2 = state(:,3);
    q3 = state(:,4);
    
    u = zeros(length(state), 3);
    % Comanding Torque
    for i = 1:1:length(state)
        u(i,:) = -K*state(i,1:7)';
    end
    t1 = u(:,1);
    t2 = u(:,2);
    t3 = u(:,3);
end
 