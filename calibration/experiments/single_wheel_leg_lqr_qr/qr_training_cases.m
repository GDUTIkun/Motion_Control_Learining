function cases = qr_training_cases()
%QR_TRAINING_CASES Representative cases for the first LQR Q/R tuning pass.

defs = [
    caseDef("pitch_0deg_pulse_0N", 0, 0, 8.0, 0.5)
    caseDef("pitch_2deg_pulse_0N", 2, 0, 8.0, 1.0)
    caseDef("pitch_3deg_pulse_5N", 3, 5, 14.0, 1.1)
    caseDef("pitch_3deg_pulse_10N", 3, 10, 14.0, 1.2)
    caseDef("pitch_5deg_pulse_0N", 5, 0, 8.0, 1.4)
    ];

cases = defs(:);
end

function c = caseDef(name, pitchDeg, pulseAmplitudeN, stopTime, weight)
pulseDelay = 7.0;
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
