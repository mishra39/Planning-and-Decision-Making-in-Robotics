figure
x = randn(10,10);
r = corrcoef(x);
surf(r)
colorbar