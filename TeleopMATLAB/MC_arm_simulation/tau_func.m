function tau = tau_func(robot,t,q,qd,data,joints)
    
    % joints should be in format: list of any of [0,1,0],[1,0,0], etc
    [~,ii] = min(abs(data(:,1)-t));
    if (data(ii,1)>t)
        tau = data(ii-1,5:7);
    else % data(ii,1)<t
        tau = data(ii,5:7);
    end
    tau = tau.*joints; % only keep joint torques on desired joints
    
    if mod((10*t),5) <= 0.01 % display time once in a while to indicate progress
        disp(t)
    end
    
end


