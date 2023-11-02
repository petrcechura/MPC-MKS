hodnoty = csvread('ntc.csv')
t = hodnoty(:, 1)

R5 = 10;
rozl = 1024;

adc = zeros(2, length(hodnoty));
adc(2, :) = hodnoty(:, 2).*rozl ./ (hodnoty(:, 2) + R5);
adc(1, :) = t;

figure;
plot(adc(2, :), t)
p = polyfit(adc(2, :), t, 10);

ad2 = 0:1023;
t2 = round(polyval(p, ad2), 1);
hold on, plot(ad2, t2, 'r');

dlmwrite('data.dlm', t2*10, ',');
