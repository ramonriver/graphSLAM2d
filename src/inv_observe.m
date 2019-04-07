% inversa de la funció "observe", és a dir, trobar la posició del
% landmark en coordenades globals a partir de la posició del robot i la
% mesura y (respecte el robot)
function [lmk, J_lmk_rob, J_lmk_y] = inv_observe(rob, y)

% convert to Cartesian coordinates
[lmkrob, J_lmkrob_y] = p2c(y);

% Transform from robot frame to world frame
[lmk, J_lmk_rob, J_lmk_lmkrob] = fromFrame2D(rob, lmkrob);

% chain rule
J_lmk_y = J_lmk_lmkrob * J_lmkrob_y;

end