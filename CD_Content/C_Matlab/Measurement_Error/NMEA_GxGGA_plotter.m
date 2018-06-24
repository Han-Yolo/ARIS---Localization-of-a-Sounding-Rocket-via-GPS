%% Reference Position
% Sonnenberg
lat_ref = 47.043853071;
lon_ref = 8.280169834;
h_ref = 693.842;

% Dreilinden
% lat_ref = 47.059947422;
% lon_ref = 8.320842799;
% h_ref = 593.666;

[x_ref, y_ref, z_ref] = llh2xyz(lat_ref, lon_ref, h_ref);

%% GPS
UTC_gps = gps.Hour*3600 + gps.Minute*60 + gps.Second;

% calc latitude and longitude and height
lat_gps = gps.Latitude_deg + gps.Latitude_min/60;
lon_gps = gps.Longitude_deg + gps.Longitude_min/60;
h_gps = gps.Altitude_mamsl + dgps.Geoid_Sep(1);

% remove corrected data
lat_gps(gps.Status ~= 1) = nan;
lon_gps(gps.Status ~= 1) = nan;
h_gps(gps.Status ~= 1) = nan;

% calc enu
[x_gps, y_gps, z_gps] = llh2xyz(lat_gps, lon_gps, h_gps);
[e_gps, n_gps, u_gps] = xyz2enu(lat_ref, lon_ref, h_ref, x_gps, y_gps, z_gps);

% calc horizontal euclidian error
en_gps = sqrt(e_gps.^2 + n_gps.^2);

% calc errors
bias_en_gps = mean(en_gps);
var_en_gps = var(en_gps);
rms_en_gps = sqrt(sum(en_gps.^2)./length(en_gps));

bias_u_gps = mean(u_gps);
var_u_gps = var(u_gps);
rms_u_gps = sqrt(sum(u_gps.^2)./length(u_gps));

%% DGPS
UTC_dgps = dgps.Hour*3600 + dgps.Minute*60 + dgps.Second;

% calc latitude and longitude and height
lat_dgps = dgps.Latitude_deg + dgps.Latitude_min/60;
lon_dgps = dgps.Longitude_deg + dgps.Longitude_min/60;
h_dgps = dgps.Altitude_mamsl + dgps.Geoid_Sep;

% remove non DGNSS data
lat_dgps(dgps.Status ~= 2) = nan;
lon_dgps(dgps.Status ~= 2) = nan;
h_dgps(dgps.Status ~= 2) = nan;

% calc enu
[x_dgps, y_dgps, z_dgps] = llh2xyz(lat_dgps, lon_dgps, h_dgps);
[e_dgps, n_dgps, u_dgps] = xyz2enu(lat_ref, lon_ref, h_ref, x_dgps, y_dgps, z_dgps);

% calc horizontal euclidian error
en_dgps = sqrt(e_dgps.^2 + n_dgps.^2);

% calc errors
bias_en_dgps = mean(en_dgps);
var_en_dgps = var(en_dgps);
rms_en_dgps = sqrt(sum(en_dgps.^2)./length(en_dgps));

bias_u_dgps = mean(u_dgps);
var_u_dgps = var(u_dgps);
rms_u_dgps = sqrt(sum(u_dgps.^2)./length(u_dgps));

%% Plot
start = max([UTC_gps(1) UTC_dgps(1)]);
finish = min([UTC_gps(end) UTC_dgps(end)]);

index_gps = find(UTC_gps==start):find(UTC_gps==finish);
index_dgps = find(UTC_dgps==start):find(UTC_dgps==finish);

timeVec_gps = (index_gps - find(UTC_gps==start) + 1)./60;
timeVec_dgps = (index_dgps - find(UTC_dgps==start) + 1)./60;


figure('Name', 'Scatterplot')
hold on
plot(e_gps(index_gps), n_gps(index_gps))
plot(e_dgps(index_dgps), n_dgps(index_dgps))
hold off
title('Horizontal Error Scatterplot')
xlabel('East [m]')
ylabel('North [m]')
legend('GPS', 'DGPS')


figure('Name', 'Horizontal / Vertical Errors')

subplot(121)
hold on
plot(timeVec_gps, en_gps(index_gps))
plot(timeVec_dgps, en_dgps(index_dgps))
hold off
title('Horizontal Error')
xlabel('Time [min]')
ylabel('Error [m]')
legend('GPS', 'DGPS')

subplot(122)
hold on
plot(timeVec_gps, u_gps(index_gps))
plot(timeVec_dgps, u_dgps(index_dgps))
hold off
title('Vertical Error')
xlabel('Time [min]')
ylabel('Error [m]')
legend('GPS', 'DGPS')

%% Debug
% figure('Name', 'LLH Diff')
% dlat_gps = lat_gps - lat_ref;
% dlon_gps = lon_gps - lon_ref;
% dh_gps = h_gps - h_ref;
% 
% subplot(1,3,1)
% plot(timeVec_gps, dlat_gps(index_gps))
% subplot(1,3,2)
% plot(timeVec_gps, dlon_gps(index_gps))
% subplot(1,3,3)
% plot(timeVec_gps, dh_gps(index_gps))
% 
% figure('Name', 'XYZ Diff')
% dx_gps = x_gps - x_ref;
% dy_gps = y_gps - y_ref;
% dz_gps = z_gps - z_ref;
% 
% subplot(1,3,1)
% plot(timeVec_gps, dx_gps(index_gps))
% subplot(1,3,2)
% plot(timeVec_gps, dy_gps(index_gps))
% subplot(1,3,3)
% plot(timeVec_gps, dz_gps(index_gps))
