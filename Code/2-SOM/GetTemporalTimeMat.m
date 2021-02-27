function timeMats = GetTemporalTimeMat(N,nodesInTime)%% time Stamp Matrix
ind = find(diff(nodesInTime) ~= 0);
codeInd = [0; ind];
tspentTran = diff(codeInd);
timesSpent = [];
for k = 1:size(tspentTran,1)
    if size(unique([timesSpent;tspentTran(k,1)]),1) ~= size(unique(timesSpent),1)
        timeMats{1,tspentTran(k)} = zeros(N,N);
        timesSpent = [timesSpent; tspentTran(k)];
    end
    timeMats{1,tspentTran(k)}(nodesInTime(ind(k),1),nodesInTime(ind(k)+1,1)) =...
        timeMats{1,tspentTran(k)}(nodesInTime(ind(k),1),nodesInTime(ind(k)+1,1)) + 1;
end

%%%%%
ind2 = find(diff(nodesInTime) == 0);
tspentSame = 1;
for k = 1:size(ind2,1)
    if k > 1
        if ind2(k) == ind2(k-1) + 1
            tspentSame = tspentSame + 1;
        else
            tspentSame = 1;
        end
    end
    
    if size(unique([timesSpent;tspentSame]),1) ~= size(unique(timesSpent),1)
        timeMats{1,tspentSame} = zeros(N,N);
        timesSpent = [timesSpent; tspentSame];
    end
    
    timeMats{1,tspentSame}(nodesInTime(ind2(k),1),nodesInTime(ind2(k)+1,1)) =...
        timeMats{1,tspentSame}(nodesInTime(ind2(k),1),nodesInTime(ind2(k)+1,1)) + 1;
end

end



