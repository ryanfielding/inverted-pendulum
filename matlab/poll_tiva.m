clc;
clear all;
close all;

%% Constants
scaleTheta = (7.25*3.14159/4)/4096; %Convert Potentiometer to Radians (rads/counts)
scalePos = 0.05/1170;

%% Create serial object for TIVA
s = serial('/dev/cu.usbmodem0E2258731');

%% Connect the serial port to tiva board
s.InputBufferSize = 10; %critical value to avoid aliasing - '' is 2 bytes, add whatever else TIVA sends
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
Tmax = 10; % Total time for data collection (s)
f = figure('Color',[0 0 0],'InvertHardcopy','off', 'units','normalized','outerposition',[0 0 1 1]);

axes1 = axes('Parent',f,...
        'YColor',[1 1 1],...
        'XColor',[1 1 1],...
        'Color',[0 0 0]); 
hold(axes1,'all');
grid on,
xlabel ('time (s)'), ylabel('Data'),
set(gca,'Color','k')
axis([0 Tmax -3000 3000]),
grid on


Ts = 0.02; % Sampling time (s)
i = 0;
data = 0;
t = 0;
tic % Start timer

while toc <= Tmax
    i = i + 1;
    %% Read buffer data
    fprintf(s,'');
    data = fread(s)
    theta(i) = data(3) + data(4)*16^2;
    pos(i) = data(7) + data(8)*16^2;
    %dc(i) = data(11) + data(12)*16^2 + data(13)*16^4;
    if (data(9) == 255)
        pos(i) = -(255-data(7)) - (255-data(8))*16^2;
        
    end
    if (data(6) == 255)
        theta(i) = -(255-data(3)) - (255-data(4))*16^2;
        
    end
    pos(i)=pos(i);
    %theta(i)=-theta(i);
    
%     if(data(14) == 255)
%         dc(i) = -(255-data(11)) - (255-data(12))*16^2 -(255-data(13))*16^4;
%     end
    %dc(i) = 5*dc(i)/100;
    
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
        line([t(i-1) t(i)],[theta(i-1) theta(i)], 'Color', 'g', 'LineWidth', 2)
        drawnow
        line([t(i-1) t(i)],[pos(i-1) pos(i)], 'Color', 'r', 'LineWidth', 2)
        drawnow
        %line([t(i-1) t(i)],[dc(i-1) dc(i)], 'Color', 'g')
        %drawnow
    end
end

fclose(s);

