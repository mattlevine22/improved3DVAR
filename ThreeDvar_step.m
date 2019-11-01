% 1 function for running 3DVAR single step
function state_assim_now = ThreeDvar_step(state_pred_now, meas_now, K, H)
    N = length(H);
    %state_pred_now is Psi(mk)
	state_assim_now = (eye(N)-K*H)*state_pred_now + K * meas_now;
end