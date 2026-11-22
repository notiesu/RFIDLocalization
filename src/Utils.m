classdef Coord
    properties
        x(1,1) double
        y(1,1) double
    end
end

classdef tagState
    properties
        position Coord
        xdot(1,1) double
        ydot(1,1) double
    end
end

classdef ReaderState
    properties
        position Coord
        heading(1,1) double
        turnSpeed(1,1) double
        velocity(1,1) double
    end
end

classdef RFIDMeasurement
    properties
        Pr(1,1) double
        isValid(1,1) logical
        uncertainty(1,1) double
        bearing(1,1) double
    end
end

class controlCommand = 
    properties
        turnAngle(1,1) double
        velocity(1,1) double
    end
end

classdef Particle = 
    properties
        state TagState
        weight(1,1) double
    end
end
