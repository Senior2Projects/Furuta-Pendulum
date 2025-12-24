t = [];

for i = 1:logsout.numElements
    sig = logsout.get(i);

    % Skip unnamed signals
    if isempty(sig.Name)
        continue
    end

    % Save time only once
    if isempty(t)
        t = sig.Values.Time;
        assignin('base', 'time', t);
    end

    % Save signal data as variable with same name
    assignin('base', sig.Name, squeeze(sig.Values.Data));
end
