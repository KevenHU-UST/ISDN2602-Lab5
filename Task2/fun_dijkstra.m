function [path, cost] = fun_dijkstra(transition, from, to)
% No help available for this function.
% NOTE: from http://www.mathworks.com/matlabcentral/fileexchange/23605
% caution: transition costs of 0 are shortcircuits unless you sparsify.

% Allocate memory.
path = cell(numel(from), numel(to));
cost = zeros(numel(from), numel(to));

% Main loop.
count = min(size(transition));
parent = zeros(count, 1);
loss = inf(count, 1);
for i = 1:numel(from)
    
    % Initialize.
    m = from(i);
    if issparse(transition)
        index = transpose(full(transition(m, :) > 0.0));
        index(m) = true;
    else
        index = transpose(not(isinf(transition(m, :))));
    end
    loss(:) = inf;
    loss(index) = transpose(transition(m, index));
    parent(:) = 0;
    parent(index) = m;
    queue = find(index);
    
    % Explore graph.
    while not(isempty(queue))
        k = queue(1);
        distance = transpose(loss(k) + transition(k, :));
        index = distance < loss;
        if issparse(transition)
            index = and(index, full(transpose(transition(k, :) > 0)));
        end
        if any(index)
            loss(index) = distance(index);
            parent(index) = k;
            queue = cat(1, queue(2:end, :), find(index));
        else
            queue = queue(2:end, :);
        end
    end
    
    % Back track.
    for j = 1:numel(to)
        path{i, j} = zeros(size(parent));
        n = to(j);
        for k = numel(parent):(-1):1
            if or(eq(m, n), eq(n, 0))
                break
            end
            path{i, j}(k) = n;
            n = parent(n);
        end
        if eq(m, n)
            path{i, j}(k) = n;
            path{i, j} = path{i, j}(k:end);
        else
            path{i, j} = [];
        end
    end
    cost(i, :) = loss(to);
    
end

return