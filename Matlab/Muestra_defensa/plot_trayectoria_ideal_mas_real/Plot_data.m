clear all
% close all
clc

%% Create the serial object
% tcpObject = tcpip('10.42.43.2', 12345);
tcpObject = tcpip('164.73.38.204', 12345);
% tcpObject = tcpip('164.73.38.228', 12345);
% tcpObject = tcpip('10.42.43.2', 12345);
% tcpObject = tcpip('164.73.38.227', 12345);
fopen(tcpObject);

%% Set up the figure window

%figureHandle = figure('NumberTitle','off',...
%    'Name','Viteza unghiulara a motorului',...
%    'Color',[0 0 0],'Visible','off');

figureHandle = figure('NumberTitle','off',...
    'Name','Trayectoria',...
    'Visible','off');

% Set axes
%axesHandle = axes('Parent',figureHandle,...
%    'YGrid','on',...
%    'YColor',[0.9725 0.9725 0.9725],...
%    'XGrid','on',...
%    'XColor',[0.9725 0.9725 0.9725],...
%    'Color',[0 0 0]);

axesHandle = axes('Parent',figureHandle,'YGrid','on','XGrid','on');

hold on;
axis equal
plotHandle = plot(axesHandle, 0, 0, 'LineWidth', 2);

%xlim(axesHandle,[min(time) max(time+0.001)]);

% Create xlabel
xlabel('Eje x [m]','FontWeight','bold','FontSize',14,'Color',[0 0 1]);

% Create ylabel
ylabel('Eje y [m]','FontWeight','bold','FontSize',14,'Color',[0 0 1]);

% Create title
title('Trayectoria','FontSize',15,'Color',[0 0 1]);

%% Trayectoria ideal
plotPath(tcpObject);

%% Collect data
count = 1;

set(figureHandle,'Visible','on');

hf=figure('position',[0 0 eps eps],'menubar','none');

data = zeros(2,1000)';
while 1
    
    ch = get(hf,'currentcharacter');
    if strcmp(ch, 'q') %quit
        close(hf)
        break
    end
    
    data(count,:) = swapbytes(fread(tcpObject, 2, 'double'))';
    
    set(plotHandle, {'YData'}, {data(:,2)}, {'XData'}, {data(:,1)});
    drawnow;
    
    count = count + 1;
    if (count == 10000)
       break;
    end
end

%% Clean up the serial object
fclose(tcpObject);
delete(tcpObject);
clear tcpObject;
%clear all;
