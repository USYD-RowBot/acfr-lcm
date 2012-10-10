function ii = markov_blanket(A,jj)
%MARKOV_BLANKET
%   ii = MARKOV_BLANKET(A,jj) computes the index ii of nodes in the Markov
%   blanket covering the nodes indexed by jj where A is the adjacency
%   matrix, i.e. A(i,j) is nonzero iff nodes i and j have an edge linking
%   them.  Note that the calculated set ii is the index of all nodes which are
%   directly linked in the graph to nodes jj. Note since A must be symmetric,
%   the algorithm only uses the upper triangular portion of A.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-06-2004      rme         Created and written.
  
% the adjacency matrix A should be symmetric. therefore work with only the
% upper triangular portion of A to find all directly connected neighbors.
% ii is a binary vector where a entry of 1 indicates an element is directly
% connected to one of the nodes specified in index jj.  note that the set ii
% is a superset of the set jj.
ii = [any(A(jj,:),1) | any(A(:,jj),2)'];

% mask the set jj from the blanket
ii(jj) = false;

% find the index of all elements in the markov blanket of jj
ii = find(ii);

