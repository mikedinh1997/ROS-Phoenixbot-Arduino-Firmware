clear
clc

%User Defined Properties 
serialPort = '/dev/ttyACM0';           % define COM port #
baudeRate = 115200;
plotTitle = 'PID Loop output';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Velocity (ticks/s)';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -30000;                     % set y-min
max = 30000;                      % set y-max
setpoint = 0;                   % PID loop set point.
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .01;                    % make sure sample faster than resolution

%Define Function Variables
time = 0;
data = 0;

%Set up Plot
plotGraph = plot(time,data,'r');

title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);

%Open Serial COM Port
s = serial(serialPort, 'BaudRate',baudeRate)
disp('Close Plot to End Session');
fopen(s);

tic

while ishandle(plotGraph) %Loop when Plot is Active

    dat = fscanf(s,'%f'); %Read Data from Serial as Float

    if(~isempty(dat) && isfloat(dat)) %Make sure Data Type is Correct        
        count = count + 1;    
        time(count) = toc;    %Extract Elapsed Time
        data(:,count) = dat(:,1); %Extract data   

        %Set Axis according to Scroll Width
        if(scrollWidth > 0)
        set(plotGraph,'XData',time(time > time(count)-scrollWidth),...
            'YData',data(time > time(count)-scrollWidth));
        hold on;
        plot(xlim, [setpoint,setpoint],'b');
        axis([time(count)-scrollWidth time(count) min max]);
        else
        set(plotGraph,'XData',time,'YData',data); hold on;
        plot(xlim, [setpoint,setpoint],'b');
        axis([0 time(count) min max]);
        end

        %Allow MATLAB to Update Plot
        pause(delay);
    end
end

%Close Serial COM Port and Delete useless Variables
fclose(s);
clear count dat delay max min baudRate plotGraph plotGrid plotTitle s ...
        scrollWidth serialPort xLabel yLabel;


disp('Session Terminated...');