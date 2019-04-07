% error de l'estat d'un robot, de la seva pose respecte la mesurada
function [e, J_e_rob] = error_pose(rob, y)

% in:
%  rob: robot pose
%  y: measured pose
%
% out:
%  e: estimation error
%  J_e_rob: Jacobian of e wrt. robot pose

e = rob - y;

        while e(3) > pi
            e(3) = e(3) - 2*pi;
        end
        
        while e(3) < -pi
            e(3) = e(3) + 2*pi;
        end

J_e_rob = eye(3); % matriu identitat 3x3

end