%clearvars;
% Input Data
tDesc = 0.00002;
pulse_per_rev = 6533;

% Grab Scope Data%
data = readtable('Scope_Log_var_speed.DAT','Format','%f %f %f %f');
data = renamevars(data,["Var1","Var2","Var3","Var4"],["Seconds","CH1","CH2","CH3"]);
deg_per_pulse = 360/pulse_per_rev;


% Interruprt is 'High' when low. Wait until state is high.
FirstRow = 0;
for i = 1:height(data)
    if data{i,4} > 3 %3.3V logic
        FirstRow = i;
        break
    end
end
%11387

%Loop through table and count in TDesc steps between dips
tCnt = 0;
newTableRow = 1;
timeArray = zeros(1,floor((height(data)/2)));
for i = (FirstRow+1):height(data)
    tCnt = tCnt + tDesc;
    if data{i,4} < 0.5 %3.3V logic
        timeArray(newTableRow) = tCnt;
        tCnt = 0;
        newTableRow = newTableRow + 1;
    end
end
timeArray = timeArray(1:newTableRow); %Remove 0s
out_velocity = deg_per_pulse./timeArray;
out_vel_clean = out_velocity;
% Multi-Sample Data
indices  = find(out_vel_clean<300);
out_vel_clean(indices) = NaN;
out_vel_clean = rmmissing(out_vel_clean);
n = 50;
b = arrayfun(@(i) mean(out_vel_clean(i:i+n-1)),1:n:length(out_vel_clean)-n+1)';
plot(b);
% Run Curve Fitting on b in y-axis
% f(x) =  a1*sin(b1*x+c1) + a2*sin(b2*x+c2) + a3*sin(b3*x+c3) + 
%         a4*sin(b4*x+c4) + a5*sin(b5*x+c5) + a6*sin(b6*x+c6) + 
%         a7*sin(b7*x+c7)

hold on;
plot(out_vel_clean);




