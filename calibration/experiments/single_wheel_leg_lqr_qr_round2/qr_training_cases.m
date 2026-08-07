function cases = qr_training_cases()
%QR_TRAINING_CASES Round-2 QR training cases.
%
% Keep the 0 N and 5 N cases as regression guardrails, then train the 10 N
% pressure cases symmetrically around initial pitch.

defs = [
    caseDef("pitch_0deg_pulse_0N", 0, 0, 5.0, 0.4)
    caseDef("pitch_-10deg_pulse_5N", -10, 5, 5.0, 0.9)
    caseDef("pitch_10deg_pulse_5N", 10, 5, 5.0, 0.9)
    caseDef("pitch_-10deg_pulse_10N", -10, 10, 5.0, 1.5)
    caseDef("pitch_0deg_pulse_10N", 0, 10, 5.0, 1.5)
    caseDef("pitch_10deg_pulse_10N", 10, 10, 5.0, 1.5)
    ];

cases = defs(:);
end

function c = caseDef(name, pitchDeg, pulseAmplitudeN, stopTime, weight)
pulseDelay = 2.0;
pulsePeriod = 10.0;
pulseWidthPercent = 5.0;

c = struct();
c.name = string(name);
c.initialPitchDeg = pitchDeg;
c.pulseAmplitudeN = pulseAmplitudeN;
c.pulseDelay = pulseDelay;
c.pulsePeriod = pulsePeriod;
c.pulseWidthPercent = pulseWidthPercent;
c.stopTime = stopTime;
c.weight = weight;
c.x0 = [0; 0; deg2rad(pitchDeg); 0; 0; 0];

if pulseAmplitudeN == 0
    c.pulseWindow = [NaN, NaN];
else
    pulseDuration = pulsePeriod * pulseWidthPercent / 100;
    c.pulseWindow = [pulseDelay, pulseDelay + pulseDuration];
end
end
