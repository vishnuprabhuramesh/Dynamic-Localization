function measured_values = sensormeasurement(z_bar, phi_bar, sigma)

z = normrnd(z_bar, sigma);
phi = normrnd(phi_bar, sigma);
measured_values = [z, phi];

end