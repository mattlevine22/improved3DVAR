function v1 = PsiGlucoseInsulin(v0, dt, drivers)

tspan = [0, dt];

% set parameters
[params, adj_vec] = get_sturis_params();

v0_model_units = v0./adj_vec;
[T,Y] = ode45(@rhs_GlucoseInsulin,tspan,v0_model_units, [], params, drivers);
v1_model_units = Y(end,:)';

v1 = v1_model_units.*adj_vec;

end


function dy = rhs_GlucoseInsulin(t,Y,params,drivers)
%% Sturis Ultradian Glucose Model from Keener Physiology Textbook
% Requires Meals to be prepped via function @SturisT2DMeal_Converter1
% B parameters should be ordered as specified in
% @write_SturisParameterOrder

%% Read in variables
% force states to be positive and real
Y = real(Y);
Y(Y<0) = 0;

% give names to each state
Ip = Y(1); %plasma insulin mass
Ii = Y(2); %interstitial insulin mass
G = Y(3);  %glucose mass
h1 = Y(4); %delay1
h2 = Y(5); %delay2
h3 = Y(6); %delay3

% give names to parameters in B
Vp = params.Vp;
Vi = params.Vi;
Vg = params.Vg;
E  = params.E;
tp = params.tp;
ti = params.ti;
td = params.td;
Rm = params.Rm;
a1 = params.a1;
C1 = params.C1;
C2 = params.C2;
C3 = params.C3;
C4 = params.C4;
C5 = params.C5;
Ub = params.Ub;
U0 = params.U0;
Um = params.Um;
Rg = params.Rg;
alpha = params.alpha;
beta = params.beta;
k_decay = params.k_decay;

%% Compute additional parameters
% compute kappa parameter as function of other parameters in B
kappa = (1/Vi + 1/(E*ti))/C4;

if ~isempty(drivers)
    % I am writing IG to be in mg/min, but should confirm that
    % 1) SturisNicuMeals outputs in mg/min
    % 2) Sturis model wants mg/min
    nutrition_oral = SturisDynaPhenoExtendedMeals(t,drivers,k_decay);
else
    nutrition_oral = 0;
end

% compute non-linear functions of parameters and states
F1 = Rm/(1 + exp(-G/(Vg*C1) + a1));
F2 = Ub*(1 - exp(-G/(Vg*C2)));
F3 = (U0 + (Um - U0)/(1 + (kappa*Ii)^-beta))/(Vg*C3);
F4 = Rg/(1 + exp(alpha*(h3/(Vp*C5)-1)));

%% System of 6 ODEs%%
dy = zeros(6,1);  % a column vector
%plasma insulin mU
dy(1) = F1 - (Ip/Vp - Ii/Vi)*E - Ip/tp; %dx/dt (mU/min)

%insterstitial insulin mU
dy(2) = (Ip/Vp - Ii/Vi)*E - Ii/ti; %dy/dt (mU/min)

%glucose in glucose space mg
dy(3) = F4 - F2 - G*F3 + nutrition_oral; %dz/dt (mg/min)

%delay process h1,h2,h3
dy(4) = (Ip-h1)/td;
dy(5) = (h1-h2)/td;
dy(6) = (h2-h3)/td;

end

function del = SturisDynaPhenoExtendedMeals(t_now,Meals,k)
%% Exogenous glucose delivery rate function
% from Albers Dynamical phenotyping paper
% for use in @SturisKeener_optimizedMex2 model
% t_now is in MINUTES
% Meals = [time(min),carbs(mg)]

% initialize total current glucose delivery rate, del.
del=0;

% find meal that occurred closest to t_now
[~,ind] = min(abs(t_now-(Meals(:,1))));

if Meals(ind,1) <= t_now
    % the closest meal is before/at t_now.
    % loop over all previous meals up to the closest meal, and sum their
    % contribution to the glucose delivery rate, del
    for i=1:ind
        mstart = Meals(i,1); % start time of ith meal
        %         I = Meals(i,3); % I constant for ith meal
        del = del + Meals(i,2)*exp(k*(mstart - t_now)/60); % add ith meal contribution to del
    end
elseif ind>1
    % the closest meal is after t_now
    % step back 1 meal to ind-1, and
    % loop over all previous meals, and sum their
    % contribution to the glucose delivery rate, del
    for i=1:(ind-1)
        mstart = Meals(i,1); % start time of ith meal
        %         I = Meals(i,3); % I constant for ith meal
        del = del + Meals(i,2)*exp(k*(mstart - t_now)/60); % add ith meal contribution to del
    end
end

del = k*del/60;

%if ind is empty, del=0
end