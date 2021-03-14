function value = bound_angle(x, m, M)

    x = round(x,4);
    m = round(m,4);
    M = round(M,4);

    value = x .* (x>=m) + m .* (x<m);
    value = value .* (value<=M) + M .* (value>M);
    
end