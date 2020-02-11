clear; close all; clc;

% left distance
d_l = 0.153927;

% right distance
d_r = 0.152247;

% angle between the measurements
alfa = pi/4;

% distance across
d_across = sqrt(d_l^2 + d_r^2-2*d_l*d_r*cos(alfa));

% left angle
left_angle = acos((d_l^2+d_across^2-d_r^2)/(2*d_l*d_across));

% right angle
right_angle = acos((d_r^2+d_across^2-d_l^2)/(2*d_r*d_across));

% error angle
rad_error = (left_angle - right_angle)/2
degree_error = rad_error * 180 / pi