% composició de frames; és a dir, expressar el nou frame obtingut a
% partir del frame F1 i el moviment fet F2 (relatiu a F1), en 
% coordenades globals; similar a trobar el vector C, suma dels 
% vectors A i B
function [F, J_f_f1, J_f_f2] = composeFrames2D(F1, F2)

T1 = F1(1:2);
th1 = F1(3);

R1 = [cos(th1) -sin(th1) ; sin(th1) cos(th1) ];

T2 = F2(1:2);
th2 = F2(3);

% R2 = [cos(th2) -sin(th2) ; sin(th2) cos(th2) ];

T = T1 + R1*T2;
th = th1 + th2;

% frame F2 expressat en coordenades globals (traslació i angle)
F = [T;th];

x2 = T2(1);
y2 = T2(2);

% Jacobiana de f respecte de F1
J_f_f1 =[...
[ 1, 0, - y2*cos(th1) - x2*sin(th1)]
[ 0, 1,   x2*cos(th1) - y2*sin(th1)]
[ 0, 0,                           1]
]; 
 
J_f_f2 =[...
[ cos(th1), -sin(th1), 0]
[ sin(th1),  cos(th1), 0]
[        0,         0, 1]
];

end

function f()
%%
syms x1 y1 th1 x2 y2 th2 real
F1 = [x1;y1;th1];
F2 = [x2;y2;th2];
F = composeFrames2D(F1,F2);
J_f_f1 = simplify(jacobian(F,F1))
J_f_f2 = simplify(jacobian(F,F2))
end