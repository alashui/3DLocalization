fid = fopen('../Meta/MetaData.txt','r');  % Open text file
InputText=textscan(fid,'%s',1000,'delimiter','\n');
input = InputText{1};

weights = zeros(length(input) - 1);
xs = zeros(length(input) - 1);
ys = zeros(length(input) - 1);

for kk = 2 : length(input)
    C = strsplit(char(input(kk)));
    weights(kk - 1) = str2num(char(C(2)));
    xs(kk - 1) = str2num(char(C(3)));
    ys(kk - 1) = str2num(char(C(4)));
end

figure('Color', [1 1 1]);
plot(weights);
title('Average Weight Over Time');
ylabel('Average Weight');
xlabel('Iteration');