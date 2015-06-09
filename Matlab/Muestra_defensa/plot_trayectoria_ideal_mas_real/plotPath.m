function plotPath(tcpObject)

    %cantidad de trayectorias

    %radio
    %dato xi
    %dato yi
    %angulo i
    %dato xf
    %dato yf
    %angulo f
    %tipo
    %xci
    %yci
    %rxi
    %ryi
    %rxf
    %ryf
    %xcf
    %ycf
    %Ci
    %S
    %Cf

    % cantidad de trayectorias a graficar
    n = swapbytes(fread(tcpObject, 1, 'double'));
%     swapbytes(n)
%     count
%     msg
    %vector = swapbytes(fread(tcpObject, n*19 , 'double'))
    
    for i=1:n
       
        % -----------------------
        % Datos de la trayectoria
        % -----------------------
       
        % Radio de cfas
        r = swapbytes(fread(tcpObject, 1, 'double'));
        % Way point inicial
        xi = swapbytes(fread(tcpObject, 1, 'double'));
        yi = swapbytes(fread(tcpObject, 1, 'double'));
        alpha = swapbytes(fread(tcpObject, 1, 'double'));
        % Way point final
        xf = swapbytes(fread(tcpObject, 1, 'double'));
        yf = swapbytes(fread(tcpObject, 1, 'double'));
        beta = swapbytes(fread(tcpObject, 1, 'double'));
        % Tipo de curva de Dubins (R=0, L=1)
        tipo = swapbytes(fread(tcpObject, 1, 'double'));
        if ((tipo == 0) || (tipo == 1) || (tipo == 5))
            cfa_i = 1;
        else
            cfa_i = 0;
        end
        if ((tipo == 0) || (tipo == 3) || (tipo == 5))
            cfa_f = 1;
        else
            cfa_f = 0;
        end
        % Coordenadas cfa inicial
        xci = swapbytes(fread(tcpObject, 1, 'double'));
        yci = swapbytes(fread(tcpObject, 1, 'double'));
        % Coordenadas recta
        xri = swapbytes(fread(tcpObject, 1, 'double'));
        yri = swapbytes(fread(tcpObject, 1, 'double'));
        xrf = swapbytes(fread(tcpObject, 1, 'double'));
        yrf = swapbytes(fread(tcpObject, 1, 'double'));
        % Coordenadas cfa final
        xcf = swapbytes(fread(tcpObject, 1, 'double'));
        ycf = swapbytes(fread(tcpObject, 1, 'double'));
        % Angulo de arco de cfa inicial
        l1 = mod2pi(swapbytes(fread(tcpObject, 1, 'double')));
        % Largo tramo recto
        l2 = swapbytes(fread(tcpObject, 1, 'double'));
        % Angulo de arco de cfa final
        l3 = mod2pi(swapbytes(fread(tcpObject, 1, 'double')));

        % ------------------------------------
        % Generacion de vectores para graficar
        % ------------------------------------
        
        % primera cfa
        %if l1 ~= 0
        if cfa_i == 0
             theta1 = linspace(alpha+(pi/2), alpha+(pi/2)-l1, 500);
        end
        if cfa_i == 1
             theta1 = linspace(alpha-(pi/2), alpha-(pi/2)+l1, 500);
        end
        x1 = xci + r*cos(theta1);
        y1 = yci + r*sin(theta1);
        %plot(x1,y1)
        %end

        %if l3 ~= 0
        % segunda cfa
        if cfa_f == 0
            theta2 = linspace(beta+(pi/2)+l3, beta+(pi/2), 500);
        end
        if cfa_f == 1
            theta2 = linspace(beta-(pi/2)-l3, beta-(pi/2), 500);
        end
        x2 = xcf + r*cos(theta2);
        y2 = ycf + r*sin(theta2);
        %plot(x2,y2)
        %end

        % recta
        x3 = linspace(xri,xrf,500);
        y3 = linspace(yri,yrf,500);

        % grafico trayectoria y waypoints
        plot(xi,yi, '*r')
        plot(xf,yf, '*r')
        plot(x1,y1,'k')
        plot(x2,y2,'k')
        plot(x3,y3,'k')

        end

 end
