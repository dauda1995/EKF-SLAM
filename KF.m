% KF  kalman filter
% x+ = F_x *x + F_u *u + F_n * n
% y = H* x + v
%
% x : state vector       P : cova.matrix ( uncertainty associated to x)
% u : control vector
% n : perturbation vector   - Q : cov matrix (uncertainty due to noise)
% y : measurement vector
% v : measurement noise     - R : covariance matrix ( in measurement)
%
% F_x : transition matrix
% F_u : control matrix
% F_n: pert. matrix
% H  : measurement matrix
%
% chapter 2
% initialization of filter
%
% define F_x, F_n, F_u and H  (they define dynamic system)
% Precise x, P, Q and R  ( x and P determine initial state, Q and R
% determine initial perturbation
%
%3. Temporal loop
% 3a. Prediction of mean(x) and P at the arrival of u
%
%   x+ = F_x * x + F_u * u          ... ( + F_n * 0 )
%   P+ = F_x * P *F_x' + F_n * Q * F_n'
%
% 3b. correction of mean(x) and p at the arrival of y
%
%   e (expectation of measurement)
%   e = H * x
%   E = covariance of expectation
%   E = H * P * H'
%   
%   z = innovation ( difference between measurement and expectation)
%   z = y - e
%   Z = R + E  innovation covariance mat.
%   
%   K = P * H' * Z^-1   - Kalman gain
%
%   x+ = x + K* z  (optimal state)
%   P+ = P - K * H *P // P - K*Z*K'  // and Joseph form (positive)
%
%4. plot results
%
%5. how to setup KF examples
%
%   1. Simulate system, get x, u, and y trajectories
%
%   2. Estimate x with the KF. Get x and P trajectories.
%
%   3. Plot results
%