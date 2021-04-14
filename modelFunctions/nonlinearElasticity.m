function psi = nonlinearElasticity(x, K1, K2)
    
    q = x(1:2);
    theta = x(3:4);
    phi = q - theta;
    
    linear = K1 * phi;
    cubic = K2 * (phi.^3);
    psi = linear + cubic;
end