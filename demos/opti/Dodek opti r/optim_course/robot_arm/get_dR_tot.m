function [dR_tot] = get_dR_tot(theta, R, dR, j)

dR_tot = eye(3);

for i = 1:length(R)
    if i == j
        dR_tot = %TODO
    else
        dR_tot = %TODO
    end
end

end