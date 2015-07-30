function angle = piTopi(angle)
    % Input: array of angles.
    % Output: normalised angles[-pi, pi].
    %%
    twopi = 2*pi;
    angle = angle - twopi*fix(angle/twopi); % this is a stripped-down version of rem(angle, 2*pi)

    i = find(angle >= pi);
    angle(i) = angle(i) - twopi;

    i = find(angle < -pi);
    angle(i) = angle(i) + twopi;
