function v1 = PsiL63(v0, dt, drivers)

tspan = [0, dt];
params = struct();
params.a = 10;
params.b = 28;
params.c = 8/3;
[T,Y] = ode45(@rhs_l63,tspan,v0, [], params);
v1 = Y(end,:)';
end


function dYdt = rhs_l63(t, Y, params)
	a = params.a;
	b = params.b;
	c = params.c;
	x = Y(1);
	y = Y(2);
	z = Y(3);
	dxdt = -a*x + a*y;
	dydt = b*x - y - x*z;
	dzdt = -c*z + x*y;
	dYdt = [dxdt; dydt; dzdt];
end