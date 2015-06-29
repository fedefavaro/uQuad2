function  ang_2pi = mod2pi( angulo )

    while angulo < 0
        angulo = angulo + 2*pi;
    end
    while angulo >= 2*pi
        angulo = angulo - 2*pi;
    end
    
    ang_2pi = angulo;

end
