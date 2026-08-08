function cases = boundary_pulse_cases(pitchDegList, pulseAmplitudeList, stopTime)
%BOUNDARY_PULSE_CASES Cases around the observed disturbance boundary.
%
% Default:
%   theta0 = [-10, 0, 10] deg
%   pulse  = [8, 8.5, 9] N
%   stop   = 8 s

if nargin < 1 || isempty(pitchDegList)
    pitchDegList = [-10, 0, 10];
end
if nargin < 2 || isempty(pulseAmplitudeList)
    pulseAmplitudeList = [8, 8.5, 9];
end
if nargin < 3 || isempty(stopTime)
    stopTime = 8.0;
end

pulseDelay = 2.0;
pulsePeriod = 10.0;
pulseWidthPercent = 5.0;

cases = repmat(emptyCase(), numel(pitchDegList) * numel(pulseAmplitudeList), 1);
caseIdx = 0;

for pitchDeg = pitchDegList
    for pulseAmplitude = pulseAmplitudeList
        caseIdx = caseIdx + 1;
        cases(caseIdx) = makeCase(pitchDeg, pulseAmplitude, stopTime, ...
            pulseDelay, pulsePeriod, pulseWidthPercent);
    end
end
end

function c = makeCase(pitchDeg, pulseAmplitude, stopTime, pulseDelay, ...
    pulsePeriod, pulseWidthPercent)
c = emptyCase();
c.initialPitchDeg = pitchDeg;
c.pulseAmplitudeN = pulseAmplitude;
c.pulseDelay = pulseDelay;
c.pulsePeriod = pulsePeriod;
c.pulseWidthPercent = pulseWidthPercent;
c.x0 = [0; 0; deg2rad(pitchDeg); 0; 0; 0];
c.name = sprintf("pitch_%gdeg_pulse_%gN", pitchDeg, pulseAmplitude);
c.description = sprintf("Initial pitch %g deg, %g N pulse at %.2f s.", ...
    pitchDeg, pulseAmplitude, pulseDelay);
c.stopTime = stopTime;
pulseDuration = pulsePeriod * pulseWidthPercent / 100;
c.pulseWindow = [pulseDelay, pulseDelay + pulseDuration];
end

function c = emptyCase()
c = struct();
c.name = "";
c.description = "";
c.x0 = zeros(6, 1);
c.stopTime = 8.0;
c.initialPitchDeg = 0;
c.pulseAmplitudeN = 0;
c.pulseDelay = 2.0;
c.pulsePeriod = 10.0;
c.pulseWidthPercent = 5.0;
c.pulseWindow = [NaN, NaN];
c.notes = "";
end
