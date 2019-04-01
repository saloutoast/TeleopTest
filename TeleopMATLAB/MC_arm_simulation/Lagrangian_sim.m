% % define manipulator equations based on calcs in mathematica notebook
% % ignoring terms with coefs <10^-18 or so
% 
% H = zeros(3,3);
% H(1,1) = 0.00624264 + 0.00135721*cos(2*qh) + 0.000452263*cos(qk) + 0.0000416957*cos(2*(qh+qk)) + 0.000452263*cos((2*qh)+qk);
% H(2,1) = 0.00121006*sin(qh) + 0.000124979*sin(qh+qk);
% H(3,1) = 0.000124979*sin(qh+qk);
% H(1,2) = 0.00121006*sin(qh) + 0.000124979*sin(qh+qk);
% H(2,2) = 0.00409032 + 0.0008705*cos(2*qa) + 0.000904526*cos(qk) + 0.000184*sin(2*qa);
% H(2,3) = 0.000156391 + 0.000073*cos(2*qa) + 0.000452263*cos(qk);
% H(3,1) = 0.000124979*sin(qh+qk);
% H(3,2) = 0.000156391 + 0.000073*cos(2*qa) + 0.000452263*cos(qk);
% H(3,3) = 0.000156391 + 0.000073*cos(2*qa);
% 
% Tg = zeros(3,1);
% Tg(1) = 0;
% Tg(2) = 9.81*(0.0249608*cos(qh) + 0.00216601*cos(qh+qk));
% Tg(3) = 0.0212486*cos(qh+qk);
% 
% Co = zeros(3,1);
% Co(1) = (-0.000184*cos(2*qa) + 0.00121006*cos(qh) + 0.000124979*cos(qh+qk) + 0.0008705*sin(2*qa))*dqh^2 ...
%         + (0.000249958*cos(qh+qk) + 0.000146*sin(2*qa))*dqh*dqk ...
%         + (0.000124979*cos(qh+qk) + 0.000073*sin(2*qa))*dqk^2 ...
%         + dqa*((-0.00271442*sin(2*qh) - 0.0000833914*sin(2*(qh+qk)) - 0.000904526*sin((2*qh)+qk))*dqh...
%         + (-0.000452263*sin(qk) - 0.0000833914*sin(2*(qh+qk)) - 0.000452263*sin((2*qh)+qk))*dqk);
% Co(2) = (0.00135721*sin(2*qh) + 0.0000416957*sin(2*(qh+qk)) + 0.000452263*sin((2*qh)+qk))*dqa^2 ...
%         + cos(2*qa)*dqh*0.000368*dqa + sin(2*qa)*dqa*(-0.001741*dqh - 0.000146*dqk) ...
%         + sin(qk)*(-0.000904526*dqh - 0.000452263*dqk)*dqk;
% Co(3) = (0.000226131*sin(qk) + 0.0000416957*sin(2*(qh+qk)) + 0.000226131*sin((2*qh)+qk))*dqa^2 ...
%         + 0.000452263*sin(qk)*dqh^2 + sin(2*qa)*dqa*(-0.000146*dqh - 0.000146*dqk);
%     
% Td = [ba*dqa; bh*dqh; bk&dqk];
% u = [Ta; Th; Tk];

% simulate forward using equations of motion and ode45
Tf = 2;
tspan = 0:0.001:Tf;
y0 = [0;pi/3;0;0;0;0];
sol = ode45(@odefun,tspan,y0);

ysol = deval(sol,tspan);
figure; plot(tspan,ysol(2,:))

fprintf('Done!\n')

function dydt = odefun(t,y)
    
    qa = y(1);
    qh = y(2);
    qk = y(3);
    
    dqa = y(4);
    dqh = y(5);
    dqk = y(6);

    H = zeros(3,3);
    H(1,1) = 0.00624264 + 0.00135721*cos(2*qh) + 0.000452263*cos(qk) + 0.0000416957*cos(2*(qh+qk)) + 0.000452263*cos((2*qh)+qk);
    H(2,1) = 0.00121006*sin(qh) + 0.000124979*sin(qh+qk);
    H(3,1) = 0.000124979*sin(qh+qk);
    H(1,2) = 0.00121006*sin(qh) + 0.000124979*sin(qh+qk);
    H(2,2) = 0.00409032 + 0.0008705*cos(2*qa) + 0.000904526*cos(qk) + 0.000184*sin(2*qa);
    H(2,3) = 0.000156391 + 0.000073*cos(2*qa) + 0.000452263*cos(qk);
    H(3,1) = 0.000124979*sin(qh+qk);
    H(3,2) = 0.000156391 + 0.000073*cos(2*qa) + 0.000452263*cos(qk);
    H(3,3) = 0.000156391 + 0.000073*cos(2*qa);

    Tg = zeros(3,1);
    Tg(1) = 0;
    Tg(2) = 9.81*(0.0249608*cos(qh) + 0.00216601*cos(qh+qk));
    Tg(3) = 0.0212486*cos(qh+qk);

    Co = zeros(3,1);
    Co(1) = (-0.000184*cos(2*qa) + 0.00121006*cos(qh) + 0.000124979*cos(qh+qk) + 0.0008705*sin(2*qa))*dqh^2 ...
            + (0.000249958*cos(qh+qk) + 0.000146*sin(2*qa))*dqh*dqk ...
            + (0.000124979*cos(qh+qk) + 0.000073*sin(2*qa))*dqk^2 ...
            + dqa*((-0.00271442*sin(2*qh) - 0.0000833914*sin(2*(qh+qk)) - 0.000904526*sin((2*qh)+qk))*dqh...
            + (-0.000452263*sin(qk) - 0.0000833914*sin(2*(qh+qk)) - 0.000452263*sin((2*qh)+qk))*dqk);
    Co(2) = (0.00135721*sin(2*qh) + 0.0000416957*sin(2*(qh+qk)) + 0.000452263*sin((2*qh)+qk))*dqa^2 ...
            + cos(2*qa)*dqh*0.000368*dqa + sin(2*qa)*dqa*(-0.001741*dqh - 0.000146*dqk) ...
            + sin(qk)*(-0.000904526*dqh - 0.000452263*dqk)*dqk;
    Co(3) = (0.000226131*sin(qk) + 0.0000416957*sin(2*(qh+qk)) + 0.000226131*sin((2*qh)+qk))*dqa^2 ...
            + 0.000452263*sin(qk)*dqh^2 + sin(2*qa)*dqa*(-0.000146*dqh - 0.000146*dqk);

    ba = 0;
    bh = 0;
    bk = 0;
        
    Td = [ba*dqa; bh*dqh; bk*dqk];
    u = zeros(3,1); %[Ta; Th; Tk];
    
    ddq = inv(H)*(-Co - Tg - Td + u);
    
    dydt = [dqa; dqh; dqk; ddq];
end


















