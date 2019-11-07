function [params, adj_vec] = get_sturis_params()
pmolperLpermU = 6.945; %conversion factor for insulin units (needed only when using Sturis)

params = struct();
params.Vp = 3; %'Vp' [l]
params.Vi = 11; %'Vi' [l]
params.Vg = 10; %'Vg' [l]
params.E = 0.2; %'E' [l min^-1]
params.tp = 6; %'tp' [min]
params.ti = 100; %'ti' [min]
params.td = 12; %'td' [min]
params.Rm = 209; %'Rm' [mU min^-1]
params.a1 = 6.67; %'a1' []
params.C1 = 300; %'C1' [mg l^-1]
params.C2 = 144; %'C2' [mg l^-1]
params.C3 = 100; %'C3' [mg l^-1]
params.C4 = 80; %'C4' [mU l^-1]
params.C5 = 26; %'C5' [mU l^-1]
params.Ub = 72; %'Ub' [mg min^-1]
params.U0 = 4; %'U0' [mg min^-1]
params.Um = 94; %'Um' [mg min^-1]
params.Rg = 180; %'Rg' [mg min^-1]
params.alpha = 7.5; %'alpha' []
params.beta = 1.77; %'beta' []
params.k_decay = 0.5; %'k_decay' []

adj_vec = [pmolperLpermU/params.Vp,pmolperLpermU/params.Vi,1/(10*params.Vg),1,1,1]';

end