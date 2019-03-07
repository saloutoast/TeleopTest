% function to build mini-cheetah arm "bot" using robotics toolbox

function robot = build_arm(botname)

    offset_lengths = [0.0577, 0.2088, 0.175];

    % masses, coms, and I tensors from solidworks model

    masses = [0.56422, 0.66736, 0.05626]; % shoulder, upper, lower
    sh_com = [0.0, 0.00382, -0.034]; % x, y, z in shoulder DH frame
    up_com = [0.18944, 0.00131, 0.01742]; % x, y, z in upper frame
    lo_com = [0.1365, 0, 0]; % x, y, z in lower frame

    % at center of mass, aligned with link frame
    sh_I = [465542, 387177, 586815, 157, 63742, 782]*(10^(-9)); % Ixx, Iyy, Izz, Ixy, Iyz, Ixz
    up_I = [447044, 1888255, 2042453, 17455, 15737, 219153]*(10^(-9));
    lo_I = [5479, 190979, 194005, 0, 0, 0]*(10^(-9));

    % TODO: (from Link.Link in robot.pdf) set these params
    jf = [0.05, 0.05, 0.8]; %0.04; % 'B' joint friction (should be the same for each)... includes joint-space damping Kd 
    cf = [0, 0, 0]; %0.1; % 'Tc' coulomb friction (should be the same for each)
%     belt_jf = 0.1; %0.04; % add to coulomb friction for knee
    gr = 1; %1/6; % 'G' gear ratio (should be the same for each)
    belt_gr = 1/1.5556; % add to gear ratio for knee
    
    shoulder = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset',pi/2, 'm', masses(1));
    shoulder.r = sh_com;
    shoulder.I = sh_I;
    shoulder.B = jf(1);
    shoulder.Tc = cf(1);
    shoulder.G = gr;

    upper = Link('d', -offset_lengths(1), 'a', -offset_lengths(2), 'alpha', 0, 'm', masses(2));
    upper.r = up_com;
    upper.I = up_I;
    upper.B = jf(2);
    upper.Tc = cf(2);
    upper.G = gr;

    lower = Link('d', 0, 'a', -offset_lengths(3), 'alpha', 0, 'm', masses(3));
    lower.r = lo_com;
    lower.I = lo_I;
    lower.B = jf(3);
    lower.Tc = cf(2);
    lower.G = gr*belt_gr; % due to belt

    robot = SerialLink([shoulder, upper, lower], 'name', botname, 'gravity', [0,0,9.81]); 
    
end
