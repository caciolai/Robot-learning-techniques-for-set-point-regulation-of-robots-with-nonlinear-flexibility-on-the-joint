function psi = nonlinearElasticity(phi, params)
    
    % phi = q - theta;
    K1 = params.K1;
    K2 = params.K2;
    
    linear = K1 * phi;
    cubic = K2 * (phi.^3);
    psi = linear + cubic;
end