function v1 = PsiLinear(v0, t_init, dt, drivers)

tspan = [t_init, dt];
params = struct();
params.A = [-1.1269  -0.4940 0.1129;
            1.0000  0       0;
            0       -1.0000  0];
params.B = [-0.3832;
            0.5919;
            0.5191];
params.C = [1 0 0];
params.D = 0;
params.G = [0; 0; 0];
params.H = 0;
[T,Y] = ode45(@rhs_linear,tspan,v0, [], params);
v1 = Y(end,:)';
end


function dYdt = rhs_linear(t, Y, params)
	A = params.A;
	B = params.B;
	C = params.C;
    D = params.D;
    G = params.G;
    H = params.H;
    dimX = size(A,2);
    dimU = size(B,2);
    for i = 1:dimX
        x(i) = Y(i);
    end
    
    for i = 1:dimU
        u(i) = 0; %we dont have inputs
    end
    
	dYdt = A*x' + B*u';
end