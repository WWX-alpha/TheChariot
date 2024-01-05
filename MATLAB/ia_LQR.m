function [Aa, Ba, Qa, Sa, R, ud] = ia_LQR(A, B, Q, R, S, xd)
    n = size(A,1);
    p = size(B,2);
    Ca = [eye(n) -eye(n)];
    Aa = [A eye(n)-A;
        zeros(n) eye(n)];
    Ba = [B;zeros(n,p)];
    Qa = transpose(Ca)*Q*Ca;
    Sa = transpose(Ca)*S*Ca;
    R = R;
    ud = mldivide(B, (eye(n)-A)*xd);
end

