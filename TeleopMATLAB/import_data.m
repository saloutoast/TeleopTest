% to import UART data log from Tiva MCU
% format: [pos0, vel0, U0, I0, pos1, vel1, U1, I1]
filename = "test4_data.txt";
test4_data = load(filename);

time = (0.001:0.001:(0.001*size(test4_data,1)))';

pos0_raw = test4_data(:,1);
pos0_deg = pos0_raw*360/8192;
pos1_raw = test4_data(:,5);
pos1_deg = pos1_raw*360/8192;

vel0 = test4_data(:,2);
vel1 = test4_data(:,6);

U0 = test4_data(:,3);
U1 = test4_data(:,7);

I0_raw = test4_data(:,4);
I0_V = I0_raw*(1.65/2048);
I0_Amp = (I0_V-1)*(1/0.9);
I1_raw = test4_data(:,8);
I1_V = I1_raw*(1.65/2048);
I1_Amp = (I1_V-1)*(1/0.9);