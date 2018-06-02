lon = 8.280169834;
lat = 47.043853071;
h = 693.842;

[x, y, z] = llh2xyz(lat, lon, h);

printf('x = %.20f\n', x)
printf('y = %.20f\n', y)
printf('z = %.20f\n', z)