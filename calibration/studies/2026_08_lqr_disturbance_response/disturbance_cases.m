function cases = disturbance_cases()
%DISTURBANCE_CASES Sweep cases for LQR + QP disturbance response.
%
% Default sweep:
%   initial pitch angle = [0, 1, 2, 3, 5] deg
%   pulse force         = [0, 2, 5, 10] N
%
% The Pulse Generator block is parameterized by base-workspace variables:
%   disturbancePulseAmplitude
%   disturbancePulsePeriod
%   disturbancePulseWidth
%   disturbancePulseDelay

pitchDegList = [0, 1, 2, 3, 5];
pulseAmplitudeList = [0, 2, 5, 10];

pulseDelay = 7.0;
pulsePeriod = 10.0;
pulseWidthPercent = 5.0;

cases = repmat(emptyCase(), numel(pitchDegList) * numel(pulseAmplitudeList), 1);
caseIdx = 0;

for pitchDeg = pitchDegList
    for pulseAmplitude = pulseAmplitudeList
        caseIdx = caseIdx + 1;
        cases(caseIdx) = makeCase(pitchDeg, pulseAmplitude, ...
            pulseDelay, pulsePeriod, pulseWidthPercent);
    end
end
end

function c = makeCase(pitchDeg, pulseAmplitude, pulseDelay, ...
    pulsePeriod, pulseWidthPercent)
c = emptyCase();
c.initialPitchDeg = pitchDeg;
c.pulseAmplitudeN = pulseAmplitude;
c.pulseDelay = pulseDelay;
c.pulsePeriod = pulsePeriod;
c.pulseWidthPercent = pulseWidthPercent;
c.x0 = [0; 0; deg2rad(pitchDeg); 0; 0; 0];

if pulseAmplitude == 0
    c.name = sprintf("pitch_%gdeg_pulse_0N", pitchDeg);
    c.description = sprintf("Initial pitch %g deg, no pulse.", pitchDeg);
    c.stopTime = 8.0;
    c.pulseWindow = [NaN, NaN];
else
    c.name = sprintf("pitch_%gdeg_pulse_%gN", pitchDeg, pulseAmplitude);
    c.description = sprintf("Initial pitch %g deg, %g N pulse at %.2f s.", ...
        pitchDeg, pulseAmplitude, pulseDelay);
    c.stopTime = 14.0;
    pulseDuration = pulsePeriod * pulseWidthPercent / 100;
    c.pulseWindow = [pulseDelay, pulseDelay + pulseDuration];
end
end

function c = emptyCase()
c = struct();
c.name = "";
c.description = "";
c.x0 = zeros(6, 1);
c.stopTime = 8.0;
c.initialPitchDeg = 0;
c.pulseAmplitudeN = 0;
c.pulseDelay = 7.0;
c.pulsePeriod = 10.0;
c.pulseWidthPercent = 5.0;
c.pulseWindow = [NaN, NaN];
c.notes = "";
end
