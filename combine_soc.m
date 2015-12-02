function weighted_soc = combine_soc(SOCv, SOCc)

    % checks if all inputs are valid
    if or(SOCv > 100, SOCv < 0)
        weighted_soc = -1;
        return
    end
    
    if or(SOCc > 100, SOCc < 0)
        weighted_soc = -2;
        return
    end

    % if confirmed to be valid (AKA not returned from function) then
    % calculated weighted soc properly
    
    weighted_soc = 0.4 * SOCv + 0.6 * SOCc;

end

