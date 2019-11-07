function init_vec = get_lorenz_inits(n)
%% uniform box random sampling for loren initial conditions not too far from the attractor
xmin = -10;
xmax = 10;
ymin = -20;
ymax = 30;
zmin = 10;
zmax = 40;

if nargin < 1
    n = 1;
end
init_vec = zeros(3, n);
for k=1:n
    xrand = xmin+(xmax-xmin)*rand;
    yrand = ymin+(ymax-ymin)*rand;
    zrand = zmin+(zmax-zmin)*rand;
    init_vec(:,k) = [xrand, yrand, zrand];
end

end


