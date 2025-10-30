% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% use Dijkstra's algorithm to find the shortest path
% define the connectivity of the graph
% Graph(i,j) denotes the cost between node i and node j
% Graph(i,i) = 0;
% the cost between nodes i and j is N => Graph(i,j) = N; 
% nodes i and j not adjacent => Graph(i,j) = infty; 

%		  1   2   3   4   5   6
Graph = [ 0   2   6   1   inf inf; 		% 1
          2   0   3   2   inf inf;		% 2
          6   3   0   4   2   2;		% 3
          1   2   4   0   1   inf;		% 4
          inf inf 2   1   0   5;		% 5
          inf inf 2   inf 5   0];		% 6

% find the shortest path from source to destination          
source = 1;
dest = 3;

[path, cost] = fun_dijkstra(Graph, source, dest);
path = cell2mat(path)
cost