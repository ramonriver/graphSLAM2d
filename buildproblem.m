function [A, r] = buildproblem(states,factor)

K = numel(factor);

A = zeros(25 , 17); % = J
r = zeros(25 , 1); % = e

row = 1;

for k = 1:K
    
    y = factor{k}.measurement;
    W = (factor{k}.covariance)^-1; % matriu d'informació, inversa de la covariança
    Wt2 = chol(W, 'upper'); %sqrt(matriu)
    
    switch factor{k}.type
        case 'motion'
            i = factor{k}.index(1);
            j = factor{k}.index(2);
            rob1 = states{1+ i};% {} perquè els estats poden ser de diferents mides
            rob2 = states{1+ j};
            [e, J_e_rob1, J_e_rob2] = error_move(rob1.value, rob2.value, y);
               
            r(row : row + numel(y) -1) = Wt2 * e;
            A(row : row + numel(y) -1 , rob1.range) = Wt2 * J_e_rob1;
            A(row : row + numel(y) -1 , rob2.range) = Wt2 * J_e_rob2;
            
        case 'lmk'
            i = factor{k}.index(1);
            j = factor{k}.index(2);
            rob = states{1+ i};
            lmk = states{1+ j};
            [e, J_e_rob, J_e_lmk] = error_observe(rob.value, lmk.value, y);
            
            r(row : row + numel(y) -1) = Wt2 * e;
            A(row : row + numel(y) -1 , rob.range) = Wt2 * J_e_rob;
            A(row : row + numel(y) -1 , lmk.range) = Wt2 * J_e_lmk;
            

        case 'pose'
            i = factor{k}.index(1);
            rob = states{1+ i};
            [e, J_e_rob] = error_pose(rob.value, y);
                        
            r(row: row + numel(y) -1) = Wt2 * e;
            A(row: row + numel(y) -1 , rob.range) = Wt2 * J_e_rob;
            
            
    end
    
    row = row + numel(y); %avancem a la següent fila per omplir la Jacobiana
    
end