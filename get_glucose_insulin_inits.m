function init_vec = get_glucose_insulin_inits(n)

[params, adj_vec] = get_sturis_params();

%% uniform box random sampling for initial conditions
% Ipmin = 20;
% Ipmax = 300;
% Iimin = 50;
% Iimax = 400;
% Gmin = 6000;
% Gmax = 30000;
% h1min = 10;
% h1max = 200;
% h2min = 10;
% h2max = 200;
% h3min = 10;
% h3max = 200;
Ipmin = 80;
Ipmax = 90;
Iimin = 150;
Iimax = 170;
Gmin = 9000;
Gmax = 11000;
h1min = 60;
h1max = 80;
h2min = 60;
h2max = 80;
h3min = 60;
h3max = 80;

if nargin < 1
    n = 1;
end
init_vec = zeros(6,n);
for k=1:n
    Iprand = Ipmin+(Ipmax-Ipmin)*rand;
    Iirand = Iimin+(Iimax-Iimin)*rand;
    Grand = Gmin+(Gmax-Gmin)*rand;
    h1rand = h1min+(h1max-h1min)*rand;
    h2rand = h2min+(h2max-h2min)*rand;
    h3rand = h3min+(h3max-h3min)*rand;
    init_vec(:,k) = [Iprand, Iirand, Grand, h1rand, h2rand, h3rand]'.*adj_vec;
end

end


