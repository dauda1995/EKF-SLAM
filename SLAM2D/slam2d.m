%slam algorithm

% I. INITIALIZE
%
%   0. System def.

%   System noise
q = [0.01; 0.01];  %perturbation signal
Q = diag(q.^2);
%   measurement noise
m = [.25; 2*pi/180];
M = diag(m.^2);
randn('seed', 1);

%   1. simulator
%       R: robot pose
%       u: control

R = [0;-2.5;0];  %starting pose
u = [0.1; 0.07];  % 0.1 meter movement, 0.05 rads
W = cloister(-4,4,-4,4);
y = zeros(2,size(W,2));

% 2. Estimator
robotSize = 3;
x = zeros(robotSize + numel(W), 1);
P = zeros(numel(x), numel(x));
mapspace = 1:numel(x);
l = zeros(2, size(W,2));

r = find(mapspace, robotSize);
mapspace(r) = 0;
x(r) = R;   % robot state
P(r,r) = 0; % covariance matrix


% 3. Graphics
mapFig = figure(1);
cla;
axis([-6 6 -6 6])
axis square
WG = line('parent', gca,...
    'linestyle', 'none', ...
    'marker', 'o', ...
    'color', 'r', ...
    'xdata', W(1,:), ...
    'ydata', W(2,:));

RG = line('parent', gca, ...
    'marker', '*', ...
    'color', 'r', ...
    'xdata', R(1), ...
    'ydata', R(2));

rG = line('parent', gca, ...
    'linestyle', 'none',...
    'marker', '*', ...
    'color', 'b', ...
    'xdata', x(r(1)), ...
    'ydata', x(r(2)));

lG = line('parent', gca, ...
    'linestyle', 'none', ...
    'marker', '+', ...
    'color', 'b', ...
    'xdata', [], ...
    'ydata', []);

eG = zeros(1,size(W,2));
for i = 1:size(W,2)
    eG(i) = line(...
        'parent', gca,...
        'color','g',...
        'xdata',[],...
        'ydata',[]);
end

for i = 1:numel(eG)
    eG(i) = line(...
        'parent', gca,...
        'color','c' ,...
        'xdata', [],...
        'ydata', []);
end

reG = line(...
    'parent', 'm',...
    'color', 'm',...
    'xdata',[],...
    'ydata',[]);

% II. Temporal loop

for t = 1:100
    
    % 1. simulator
    n = q.*randn(2,1);
    R = move(R, u, zeros(2,1));
    for lid = 1:size(W,2)       % lid == landmark id
        v = m.*randn(2,1);
        y(:,lid) = project(R, W(:,lid)) + v;
    end
    
    %2. Filter
    %   a. Prediction
    %   CAUTION this os sub-optimal in CPU time
    [x(r), R_r, R_n] = move(x(r), u, n);
    P_rr = P(r,r);
    P(r,:) = R_r*P(r,:);
    P(:,r) = P(r,:)';
    P(r,r) = R_r*P_rr*R_r' + R_n*Q*R_n';
    
    % b.correction
    %   i. known lmks
    % no visibility function and all landmarks are seen
    lids = find(l(1,:));
    for lid = lids
        %expectation
        [e, E_r, E_l] = project(x(r), x(l(:,lid)));
        E_rl = [E_r E_l];
        rl = [r l(:,lid)'];
        E = E_rl * P(rl, rl) * E_rl';
        
        %measurement
        yi = y(:,lid);
        
        
        %innovation
        z = yi - e;
        
        %dealing with angle differences
        if z(2) > pi
            z(2) = z(2) - 2*pi;
        end
        if z(2) < -pi
            z(2) = z(2) + 2*pi;
        end
        Z = M + E;
        
        %Kalman gain
        K = P(:, rl) * E_rl' * Z^-1;
        
        %update
        x = x + K *z;
        P = P - K * Z * K';  % high complexity
    end
    
    %   ii. init new lmks
    % check lmk availability
    lid = find(l(1,:)==0,1);      % data association
    if ~isempty(lid)
        s = find(mapspace, 2);
        if ~isempty(s)
            mapspace(s) = 0;
            l(:,lid) = s';
            %measurement
            yi = y(:,lid);
            
            [x(l(:,lid)), L_r, L_y] = backProject(x(r), yi);
            %computing co variance and cross variance
            P(s,:) = L_r * P(r,:);
            P(:,s) = P(s,:)';
            P(s,s) = L_r * P(r,r) * L_r' + L_y * M * L_y';
            
        end
    end
    
    %3. Graphics
    set(RG, 'xdata', R(1), 'ydata', R(2));
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2)));
    lids = find(l(1,:));
    lx = x(l(1,lids));
    ly = x(l(2,lids));
    set(lG, 'xdata', lx, 'ydata', ly);
    for lid = lids
        le = x(l(:,lid));
        LE = P(l(:,lid),l(:,lid));
        [X,Y] = cov2elli(le,LE,3,16);
        set(eG(lid),'xdata',X,'ydata',Y);
    end
    if t>1
        re = x(r(1:2));
        RE = P(r(1:2),r(1:2));
        [X,Y] = cov2elli(re,RE,3,16);
        set(reG,'xdata',X,'ydata',Y);
    end   
    drawnow;
    end
