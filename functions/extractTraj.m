function Csi_r = extractTraj(ref, N, count, trajectory_flag, T_gp, replanTrigger)

%Extract the whole planned trajectory
x_pr = ref(1,:); y_pr = ref(2,:);
persistent kt
if ~trajectory_flag
    Csi_r = repmat([x_pr(end);y_pr(end)],N,1);
else

    %If the  replanning trigger
    %is enabled, i reset the counter for trajectory extraction
    if (count == 1 ||  replanTrigger)
        kt = 2;
    end
    % Reference trajecotry
    Csi_r = repmat([x_pr(end);y_pr(end)],N,1);
    Csi_r(1:2:min(2*N,2*length(x_pr(kt:end)))) = x_pr(min(kt,length(x_pr)):min(kt+N-1,length(x_pr)));
    Csi_r(2:2:min(2*N,2*length(y_pr(kt:end)))) = y_pr(min(kt,length(x_pr)):min(kt+N-1,length(x_pr)));
    kt = kt+1;
end
end