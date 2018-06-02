% set zero values to nan
for i = 1:65
   prc.(i)(prc.(i) == 0) = nan;
end

% find all satellites for which a PRC was calculated
sats = [];
for i = 2:2:65
    if any(prc.(i))
        sats = [sats i];
    end
end

mean_prc_vec = [];
for i = 1:length(sats)
   k = prc.(sats(i));
   mean_prc_vec = [mean_prc_vec k(~isnan(k))'];
end
mean_prc = mean(abs(mean_prc_vec))

% plot each PRC in its own plot
figure('Name', 'Single')
for i = 1:length(sats)
   subplot(4, 3, i)
   plot([prc.(sats(i)) prc.(sats(i)+1)]); 
   title('Sat ' + string(sats(i)/2))
   legend('Raw', 'Filtered')
end

legendtxt = cell(length(sats), 1);
for i = 1:length(sats)
   legendtxt{i} = 'Sat ' + string(sats(i)/2);
end

% plot all unfiltered PRCs
figure('Name', 'All Unfiltered')
hold on
for i = 1:length(sats)
   plot(prc.(sats(i))); 
end
hold off
title('Pseudorange Corrections Unfiltered')
xlabel('Time [s]')
ylabel('Pesudorange Correction [m]')
legend(legendtxt, 'Location', 'southwest')

% plot all filtered PRCs
figure('Name', 'All Filtered')
hold on
for i = 1:length(sats)
   plot(prc.(sats(i)+1)); 
end
hold off
title('Pseudorange Corrections with Moving Average Filter of Order 10')
xlabel('Time [s]')
ylabel('Pesudorange Correction [m]')
legend(legendtxt, 'Location', 'southwest')