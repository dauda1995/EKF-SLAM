% define system
% x = [px py vx vy]'    state space
% y = [d, a]'            measurement vector
%
% u = [ax, ay]'
% n = [nx, ny]'
% r = [rd, ra]'
%
% px+ = px + vx*dt
% py+ = py +vy*dt
% vx+ = vx +ax*dt +nx
% vy+ = vy +ay*dt + ny
%
% d = sqrt(px^2 + py^2) + rd     distance (measurement equations)
% a = atan2(py, px) + ra         bearing
%
% x+ = x + u * dt + n
% y = x + v
dt = 0.1;

Q = diag([0.1 0.01].^2);     % velocity increment by 0.01 each time step
R = diag([.01 1*pi/180].^2);  %(least square of 10 mters) laser range finder error 10cm , 1 deg error

% simulated variables
X = [2 1 -1 1]';   % state model ( x and y cordinates 2 1  with inital velocity -1 1 m/s)

% estimated variables
x = [3 3 0 0]';   % estimated initial state of the robot
P = diag([1 1 1 1].^2);  % estimated initial covariance ( initial state and velocity)

% trajectories
tt = 0:dt:4;
XX = zeros(4, size(tt, 2));
xx = zeros(4, size(tt, 2));   %initialize vector of state positions
yy =  zeros(2, size(tt, 2)); 
PP =  zeros(4, size(tt, 2)); 

%perturbation levels
q = sqrt(diag(Q))/2;
r = sqrt(diag(R))/2;

%start loop
i = 1;
for t = tt
    
   % read control
   switch t              % m/s u is velocity increments
       case 1
           u = [2 0]'/dt;
       case 2
           u = [-3 0]'/dt;
       case 3
           u = [1 -2]'/dt;
       case 4
           u = [-2 -1]'/dt;   
       otherwise
           u = [0 0]'/dt;
   end 
   
   % simulate
   n = q .* randn(2,1);
   X = ex02_f(X, u, n, dt);
   v = r .* randn(2,1);
   y = ex02_h(X) + v;
   
   % estimate - prediction
   [x, F_x, F_u, F_n] = ex02_f(x, zeros(2,1), zeros(2,1), dt);   % u has no uncertainty, control vector
   P = F_x * P *F_x' + F_n * Q * F_n';   % + F_u * U * F_u'
   
   %correction
   [e, H] = ex02_h(x);
   E = H * P * H';
   
   z = y - e;
   Z = R + E;
   
   K = P * H' * Z^-1;
   
   x = x + K * z;
   P = P - K * H * P;
   
   % collect data
   XX(:, i) = X;
   xx(:, i) = x;
   yy(:, i) = y;
   PP(:, i) = diag(P);
   
   %update index
   i = i + 1;
   
   %plots
   plot(X(1),X(2), '*r')    % truth
   axis([-2 4 0 6])
   hold on
   plot(x(1), x(2), '*b')  % estimation tracks polar cordinates
   %plot(xx(1,1:i), xx(2,1:i))
   %hold off
   pause(0.5)
   drawnow
end

%plot
%plot(tt,XX,tt,xx,tt,yy,tt,sqrt(PP));
legend('truth', 'estimation', 'truth', 'estimation')


