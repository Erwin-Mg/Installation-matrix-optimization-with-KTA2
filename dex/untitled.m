for i=1:size(X_traj,1)
    [~,row_indice] = ismember (X_traj(i,:),D,"rows");
    row_indice = ia(row_indice);
    J = robot.jacob0(joint_angel(row_indice,:));
    manu_x(i) = sqrt(det(J*J'));
end