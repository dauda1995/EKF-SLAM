%build observation function
% jacobians of the derivatives of y wrt x
% d = sqrt(px^2 + py^2) + rd     
% a = atan2(py, px) + ra  

function [y, Y_x] = scan(x)

px = x(1);
py = x(2);

d = sqrt(px^2 + py^2);
a = atan2(py, px);
%a = atan(py/px);   only for symbolic jacobian computation

y = [d;a];


if nargout > 1
    Y_x =[...
[     px/(px^2 + py^2)^(1/2), py/(px^2 + py^2)^(1/2), 0, 0]
[ -py/(px^2*(py^2/px^2 + 1)), 1/(px*(py^2/px^2 + 1)), 0, 0]];

    
    
end

end
%%
function f()
%%
syms px py vx vy real
x = [px;py;vx;vy];
y = scan(x);
Y_x = jacobian(y, x)

end


