function [e,n,u] = xyz2enu(refLat, refLong, refH, X, Y, Z)
  % convert ECEF coordinates to local east, north, up
 
  % find reference location in ECEF coordinates
  [Xr,Yr,Zr] = llh2xyz(refLat,refLong, refH);
 
  refLat = refLat/180*pi; %converting to radians
  refLong = refLong/180*pi; %converting to radians
  
  e = -sin(refLong).*(X-Xr) + cos(refLong).*(Y-Yr);
  n = -sin(refLat).*cos(refLong).*(X-Xr) - sin(refLat).*sin(refLong).*(Y-Yr) + cos(refLat).*(Z-Zr);
  u = cos(refLat).*cos(refLong).*(X-Xr) + cos(refLat).*sin(refLong).*(Y-Yr) + sin(refLat).*(Z-Zr);
end