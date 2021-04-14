function psi = linearElasticity(x, K)
    
    q = x(1:2);
    theta = x(3:4);
    
    phi = q - theta;
    psi = K * phi;
end