function dy = diffEqn(state, K, MOI)
    %disp(state);
    % quaternions
    q0 = state(1);
    q1 = state(2);
    q2 = state(3);
    q3 = state(4);
    
    % Angular Velocities
    w1 = state(5);
    w2 = state(6);
    w3 = state(7);
    
    % Comanding Torque
    u = -K*state;
    t1 = u(1);
    t2 = u(2);
    t3 = u(3);
    
    % Moment of Inertia
    I1 = MOI(1);
    I2 = MOI(2);
    I3 = MOI(3);
    
    % State Derivative
    dy(1) = -(w1*q1)-(w2*q2)-(w3*q3);
    dy(2) = (w1*q0)+(w3*q2)-(w2*q3);
    dy(3) = (w2*q0)-(w3*q1)+(w1*q3);
    dy(4) = (w3*q0)+(w2*q1)-(w1*q2);
    dy(5) = t1/I1;
    dy(6) = t2/I2;
    dy(7) = t3/I3;
    dy = dy';
end