% error del moviment a partir de dos estats del robot i de la mesura feta
% sobre aquest moviment (drob)
function [e, J_e_rob1, J_e_rob2] = error_move(rob1, rob2, drobmeas)

% in:
%  rob1: pose of robot at time 1
%  rob2: pose of robto at time 2
%  drobmeas: motion measurement between times 1 and 2
%
% out:
%  e: expectation error
%  J_e_rob1: Jacobian of e wrt. rob1
%  J_e_rob2: Jacobian of e wrt. rob2

% expected measurement
[drobexp, J_drobexp_rob1, J_drobexp_rob2] = betweenFrames2D (rob1, rob2);

% expectation error
e = drobexp - drobmeas;
J_e_drobexp = eye(3); % matriu identitat de 3x3

% chain rule
J_e_rob1 = J_e_drobexp * J_drobexp_rob1;
J_e_rob2 = J_e_drobexp * J_drobexp_rob2;

end
