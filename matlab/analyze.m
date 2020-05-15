function ret = analyze(file)
  M = mmread(file);
  X0 = zeros(1, size(M)(1));
  X0(1) = 1;

  iterations = 0;
  curr = prev = X0;
  while true
    curr = prev * M;

    iterations = iterations + 1;
    if curr == prev
      break;
    end

    prev = curr;
  end

  fprintf('Iterations: %d\n', iterations);
  ret = curr;
end