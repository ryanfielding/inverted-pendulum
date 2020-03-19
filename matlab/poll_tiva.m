clc;
clear all;
close all;
%% Create serial object for TIVA
s = serial('/dev/cu.usbmodem0E2258731');

%% Connect the serial port to tiva board
s.InputBufferSize = 6; %critical value to avoid aliasing - '' is 2 bytes, add whatever else TIVA sends
s.BaudRate = 115200;
s.Parity = 'none';
s.DataBits = 8;
s.StopBits = 1;
s.Terminator = 'CR/LF';

try
    fopen(s);
catch err
    fclose(instrfind);
    error('Could not connect to Tiva board.');
end

%Poll Tiva board asynchronous

readasync(s);

% fprintf(s,'');
% theta = fread(s);
% thetaVal = theta(3) + theta(4)*16^2

%% Figure
Tmax = 15; % Total time for data collection (s)
figure,
grid on,
xlabel ('Time (s)'), ylabel('Data'),
axis([0 Tmax -10 5000]),

Ts = 0.1; % Sampling time (s)
i = 0;
data = 0;
t = 0;
tic % Start timer

while toc <= Tmax
    i = i + 1;
    %% Read buffer data
    fprintf(s,'');
    theta = fread(s);
    thetaVal(i) = theta(3) + theta(4)*16^2;
    data(i) = thetaVal(i);
    
    %% Read time stamp
    % If reading faster than sampling rate, force sampling time.
    % If reading slower than sampling rate, nothing can be done. Consider
    % decreasing the set sampling time Ts
    t(i) = toc;
    if i > 1
        T = toc - t(i-1);
        while T < Ts
            T = toc - t(i-1);
        end
    end
    t(i) = toc;
    %% Plot live data
    if i > 1
        line([t(i-1) t(i)],[data(i-1) data(i)])
        drawnow
    end
end

fclose(s);

