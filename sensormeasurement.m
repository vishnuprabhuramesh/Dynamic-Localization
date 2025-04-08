function measured_values = sensormeasurement(r_bar, phi_bar, sigma)

r = normrnd(r_bar, sigma);
phi = normrnd(phi_bar, sigma);
measured_values = [r, phi];

end