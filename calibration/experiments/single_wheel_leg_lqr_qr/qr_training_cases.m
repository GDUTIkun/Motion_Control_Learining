function cases = qr_training_cases()
%QR_TRAINING_CASES Representative cases for the first LQR Q/R tuning pass.

defs = [
    caseDef("pitch_0deg_pulse_0N", 0, 0, 5.0, 0.5)
    caseDef("pitch_10deg_pulse_5N", 10, 5, 5.0, 1.0)
    caseDef("pitch_-10deg_pulse_5N", -10, 5, 5.0, 1.0)
    caseDef("pitch_0deg_pulse_10N", 0, 10, 5.0, 1.4)
    caseDef("pitch_10deg_pulse_10N", 10, 10, 5.0, 1.6)
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
