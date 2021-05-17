function psi = linearElasticity(phi, params)
    
    % phi = q - theta;
    K = params.K1;
    
    psi = K * phi;
end