function Kopt = OptObservations(m0, obs_traj, dt, K0, Psi, H, drivers)

options = optimset('PlotFcns',@optimplotfval);

[K,FVAL,EXITFLAG,OUTPUT] = fminsearch(@(K) evalK(K, m0, obs_traj, dt, Psi, H, drivers), K0, options);

Kopt = K

end
