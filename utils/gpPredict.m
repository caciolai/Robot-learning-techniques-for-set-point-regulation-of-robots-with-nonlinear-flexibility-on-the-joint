function psi_pred = gpPredict(xk,gpMdl)
    q = xk(1:2);
    theta = xk(3:4);
    
    [psi_pred_1,~,~] = predict(gpMdl{1}, [q;theta]');
    [psi_pred_2,~,~] = predict(gpMdl{2}, [q;theta]');
    psi_pred = [psi_pred_1;psi_pred_2];
end

