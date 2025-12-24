% Get all variables in base workspace
vars = evalin('base', 'whos');

for k = 1:length(vars)
    % Check size and type
    if isequal(vars(k).size, [20001 1]) && strcmp(vars(k).class, 'double')
        
        % Get variable data
        data = evalin('base', vars(k).name);
        
        % Create timeseries (no time vector -> sample-based)
        ts = timeseries(data, t);
        
        % New variable name
        ts_name = ['ts_' vars(k).name];
        
        % Assign to base workspace
        assignin('base', ts_name, ts);
    end
end

disp('All 20001x1 doubles converted to timeseries.');
