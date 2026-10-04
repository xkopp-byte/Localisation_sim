function [J,g,H] = fcn_QF(x,A,b,c)

    J = 1/2*x.'*A*x + b.'*x + c;
    g = A*x + b;
    H = A;

end