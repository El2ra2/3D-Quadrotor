function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
%F = 0;

% Moment
%M = zeros(3,1);


Kd1 =   120;
Kp1 =   550;
Kd2 =   120;
Kp2 =   550;
Kd3 =   120;
Kp3 =   550;
Kpphi = 4000;
Kdphi = 500;
Kpthi = 4000;
Kdthi = 500;
Kppsi = 4000;
Kdpsi = 500;

r_ddot_1des = (des_state.acc(1) + Kd1*(des_state.vel(1)-state.vel(1))+Kp1*(des_state.pos(1)-state.pos(1)));
r_ddot_2des = (des_state.acc(2) + Kd2*(des_state.vel(2)-state.vel(2))+Kp2*(des_state.pos(2)-state.pos(2)));
r_ddot_3des = (des_state.acc(3) + Kd3*(des_state.vel(3)-state.vel(3))+Kp3*(des_state.pos(3)-state.pos(3)));

phi_des = ((r_ddot_1des*sin(des_state.yaw)) - r_ddot_2des*cos(des_state.yaw))/params.gravity;
thi_des = ((r_ddot_1des*cos(des_state.yaw)) + r_ddot_2des*sin(des_state.yaw))/params.gravity;
psi_des = des_state.yaw;

u1 = params.mass*(params.gravity + r_ddot_3des);
u2 = [(Kpphi*(phi_des - state.rot(1))-Kdphi*(state.omega(1)));(Kpthi*(thi_des- state.rot(2))-Kdthi*(state.omega(2)));(Kppsi*(psi_des- state.rot(3))+Kdpsi*(des_state.yawdot- state.omega(3)))];


F = u1;
M = params.I*u2;

% =================== Your code ends here ===================

end
