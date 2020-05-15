% this is a script file
1;

function ret = edge(sz, r, c)
  ret = zeros(sz);
  ret(r + 1, c + 1) = 1.0;
end

function ret = upd(sz, val)
  ret = zeros(sz);
  ret(:, val + 1) = 1.0;
end

L0 = kron(eye(3), 1/2 * edge(3,0,1) + 1/2 * edge(3,0,2))
L1 = kron(eye(3), edge(3,1,2))
L2 = kron(upd(3,1), edge(3,0,2)) + kron(upd(3,2), edge(3,1,2))

semantics = L0 + L1 + L2