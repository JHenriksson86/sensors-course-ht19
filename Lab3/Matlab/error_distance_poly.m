clear; clc; close all;

X = [1 0.140500 0.140500^2;
     1 0.249250 0.249250^2;
     1 0.500046 0.500046^2;
     1 0.751712 0.751712^2;
     1 1.004596 1.004596^2;
     1 1.256318 1.256318^2;
     1 1.504394 1.504394^2];
 
y = [ 0.143 0.250 0.500 0.750 1.000 1.250 1.490 ]';

invX = (X'*X)\X';
estimate = invX*y
test = estimate(3)*X(:,2).^2 + estimate(2)*X(:,2) + estimate(1);