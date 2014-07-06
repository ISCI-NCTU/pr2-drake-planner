function displayInfeasible(infeasible_constraint)
  if (~isempty(infeasible_constraint))
      disp('Infeasible constraints')
      for i=1:length(infeasible_constraint)
        fprintf('%s\n', infeasible_constraint{i});
      end
      fprintf('\n');
  end
end