%{
Generates brake bias based on brake parameters. Need to convert from m/s to
ft/s. Need to create acceleration between two points.
%}

function brakeBias = calculateBrakeBias(v, dt)
    %calculate acceleration: dv/dt
    a = zeros(size(dt));
    for i = 1:length(dt)
        a(i) = (v(i+1) - v(i)) / dt(i);
    end

    %only get where it is braking, then make it absolute value
    velocity = [];
    gs = [];
    indexes = [];
    for i = 1:length(dt)
        if (a(i) < 0)
            velocity(end+1) = v(i+1);
            gs(end+1) = abs(a(i));
            indexes(end+1) = i;
        end
    end

    %convert to mph
    velocity = velocity .* 2.237;
    %convert to ft/s^2
    gs = gs .* 3.281;

    %given data 
    weightCarDriver = 600;
    Iwheel = 207;
    Rwheel = 10;
    rRearCaliper = 3.0;
    rFrontCaliper = 2.975;
    mu1 = 1.9;
    N1 = 50;
    mu2 = 1.51;
    N2 = 350;
    distFront = 0.5;
    distRear = 1-distFront;
    wheelBase = 60.5;
    HCog = 12.5;
    
    %calculate distance from COG to wheel bases
    LFront = distFront * wheelBase;
    LRear = distRear * wheelBase;
    
    %calculate downforce
    downforce = ((((velocity.^2) - (35^2))/(35^2))*110)+110;
    downforceVelocityTable = table(downforce, velocity);
    
    %calculate total weight with downforce
    weightTotal = downforce + weightCarDriver;
    mTotal = weightTotal./32.2;
    
    %calculate normal forces
    NBothRearTires = ((weightTotal.*LFront)-(HCog.*mTotal.*gs))/(LRear + LFront);
    NBothFrontTires = weightTotal - NBothRearTires;
    NRearTire = NBothRearTires .* 0.5;
    NFrontTire = NBothFrontTires .* 0.5;
    
    %calculate static coefficient of friction for front and rear tires
    muFront = mu1 +(((NFrontTire-N1)/(N2-N1)).*(mu2-mu1));
    muRear = mu1 +(((NRearTire-N1)/(N2-N1)).*(mu2-mu1));
    
    %calculate theoretical max logintudinal friction forces
    frictionFrontTireMax = muFront.*NFrontTire;
    frictionRearTireMax = muRear.*NRearTire;
    RatioTireFront = frictionFrontTireMax./(frictionFrontTireMax + frictionRearTireMax);
    
    %calculate front and rear tire longitudinal friction
    frictionFrontTire = RatioTireFront .* mTotal.*gs.*0.5;
    frictionRearTire = (mTotal .* gs .* 0.5) - frictionFrontTire;
    
    %calculate front and rear caliper friction
    angularAccel = gs./Rwheel;
    frictionFrontCaliper = ((Iwheel.*angularAccel)+(frictionFrontTire.*Rwheel))./rFrontCaliper;
    frictionRearCaliper = ((Iwheel.*angularAccel) + (frictionRearTire.*Rwheel))./rRearCaliper;
    
    %calculate brake bias
    brakeBiasInit = frictionFrontCaliper./(frictionFrontCaliper+frictionRearCaliper);

    %get brake bias as a table across the whole track
    brakeBias = [0];
    idx = 1;
    for i = 1:length(dt)
        if idx > length(indexes) | i ~= indexes(idx)
            brakeBias(end+1) = 0;
        else 
            brakeBias(end+1) = brakeBiasInit(idx);
            idx = idx + 1;
        end
    end
    %filter the brake biases for when it is coasting
    for j = 5:5:100
        for i = 1:j:length(brakeBias)-(j+1)
            thisA = (v(i+j)-v(i))/(sum(dt(i:i+j)));
            if thisA > -1
                baseBias(i:i+j) = 0;
            end
        end
    end
    for i = 1:1:length(a)
        if a(i) > -1
            brakeBias(i) = 0;
        end
    end
end