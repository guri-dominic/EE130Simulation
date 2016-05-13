function C = agentsLoc(a, index)
% Returns the location (with noise) of all the gents at a given time(step)
    N = length(a);
    C = zeros(2,N);
    for n=1:N
        C(:,n) = a(n).xn(:,index);
    end
end

