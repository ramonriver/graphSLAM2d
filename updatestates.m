function states = updatestates(states,dx)

K = numel(states);

for k = 1:K 
    
    states{k}.value = states{k}.value + dx(states{k}.range);
    
    if strcmp(states{k}.type , 'pose')
        
        if states{k}.value(3) > pi
            
            while states{k}.value(3) > pi
                states{k}.value(3) = states{k}.value(3) - 2*pi;
            end
        else
            while states{k}.value(3) < -pi
                states{k}.value(3) = states{k}.value(3) + 2*pi;
            end
        end
    end
end

end