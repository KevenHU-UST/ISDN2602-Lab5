% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% use Dijkstra's algorithm to find the shortest path
% define the connectivity of the graph
% Graph(i,j) denotes the cost between node i and node j
% Graph(i,i) = 0;
% the cost between nodes i and j is N => Graph(i,j) = N; 
% nodes i and j not adjacent => Graph(i,j) = infty; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lab 6 Task 2 - Modify the code below
% 	Using the nodes defined in the lab manual, create the graph  
Graph = [ ];

% find the shortest path from North to Eastern          
source = ;
dest = ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[path, cost] = fun_dijkstra(Graph, source, dest);
path = cell2mat(path)
cost