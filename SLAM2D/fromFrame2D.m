function [p, P_r, P_pr] = fromFrame2D(r, p_r)

t = r(1:2);            % translation matrix
a = r(3);

R = [cos(a) -sin(a) ; sin(a) cos(a)];           % rotation matrix

p = R*p_r + t;

if nargout >1
    px = p_r(1);
    py = p_r(2);
    
    P_r =[...
        [ 1, 0, - py*cos(a) - px*sin(a)]
        [ 0, 1,   px*cos(a) - py*sin(a)]];
    
    P_pr = R;
    
    
end

end

%%
function f()
%%
syms x y a px py real
r = [x y a]';     %%make sure your terminal is in right directory before run (Evaluate section)
p_r = [px py]';
p = fromFrame2D(r, p_r);
P_r = jacobian(p, r)
end