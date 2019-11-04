function y=partial_derivative( state_pred_before, meas_now, K, true_traj_now)
psivalue=Psi(state_pred_before);
a=meas_now-H*psivalue;
b=psivalue-true_traj_now;
y=2*(K*a+b)*a';
end