

h0 = 0;
hd = 1;
h=0;
x = 0.05;
i=2;
h(1)=h0;
while h<hd
    h(i) = h(i-1) + hd*x;
    i=i+1;
end

t=50/1000;
fprintf('tiempo de convergencia: %f segundos\n',i*t);