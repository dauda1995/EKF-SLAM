% EKF  Extended kalman filter - non linear gaussian functions
% x+ = f(x, u, n)
% y = h(x) + v  (noise v is additive)
%
% x : state vector       P : cova.matrix ( uncertainty associated to x)
% u : control vector
% n : perturbation vector   - Q : cov matrix (uncertainty due to noise)
% y : measurement vector
% v : measurement noise     - R : covariance matrix ( in measurement)
%
% f() : transition function
% h() : measurement function
%
% F_x : transition matrix
% F_u : control matrix
% F_n: pert. matrix
% H  : measurement matrix
%
% chapter 2
% initialization of filter
%
% define f() and h()  (they define dynamic system)
% Precise x, P, Q and R  ( x and P determine initial state, Q and R
% determine initial perturbation
%
%3. Temporal loop
% 3a. Prediction of mean(x) and P at the arrival of u
%
%   Jacobian computation
%
%   F_x : jac. of x+ wrt. state
%   F_u : jac. of x+ wrt. control
%   F_n:  jac. of x+ wrt.preturbation
%
%   x+ = f( x, u, 0)                        initial non- linear function
%   P+ = F_x * P *F_x' + F_n * Q * F_n'     covariance propagation i.e
%                                           linearize at the estimated points
%
% 3b. correction of mean(x) and p at the arrival of y
%
%   Jacobian computation
%   H  : jac. of y wrt. x      
%
%   e (expectation of measurement)
%   e = h( x )
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
%   sigma - uncertainty of distance
%   