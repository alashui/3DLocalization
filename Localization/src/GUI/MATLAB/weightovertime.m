fid = fopen('../Meta/MetaData2.txt','r');  % Open text file
InputText=textscan(fid,'%s',1000,'delimiter','\n');
input = InputText{1};

li = 41;%length(input);

weights = zeros(li - 1, 1);
maxws = zeros(li - 1, 1);
xs = zeros(li - 1, 1);
ys = zeros(li - 1, 1);

for kk = 2 : li
    C = strsplit(char(input(kk)));
    weights(kk - 1) = str2double(char(C(2)));
    maxws(kk - 1) = str2double(char(C(5)));
    xs(kk - 1) = str2double(char(C(3)));
    ys(kk - 1) = str2double(char(C(4)));
end

its = 1:length(weights);
mdl = fitlm(its, weights);
Cos = mdl.Coefficients.Estimate;

mdltop = fitlm(its, maxws);
Costop = mdltop.Coefficients.Estimate;

f = figure('Color', [1 1 1]);
plot(maxws, 'r');
hold on
plot(its, its.*Costop(2) + Costop(1), 'r:');
plot(weights);
plot(its, its.*Cos(2) + Cos(1), ':');
legend('Top Weight','Top Fit','Average Weight','Avg Fit');
hold off
title('Average and Top Weight Over Time');
ylabel('Weight');
xlabel('Iteration');

print(f, '-dtiffn', 'weightgraph');

% f = figure(2);
% scatter(xs,ys, 100, weights);
