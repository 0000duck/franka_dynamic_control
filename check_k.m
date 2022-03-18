

[k_data,d_data] = k_var(time);


figure()
plot(time,k_data(:,1));
xlabel('time [s]')
ylabel('k [N/m]')

figure()
plot(time,d_data(:,1));
xlabel('time [s]')
ylabel('d [Ns/m]')