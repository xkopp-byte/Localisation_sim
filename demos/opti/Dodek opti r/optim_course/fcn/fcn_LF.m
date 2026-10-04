function [J,g] = fcn_LF(x,c)

J=c'*x;
g=c;

end