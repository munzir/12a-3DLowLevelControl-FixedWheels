th_log = dlmread('/usr/local/share/krang/fixed_wheels/th_log');











time = th_log(:,1);
thref = th_log(:,2);
th = th_log(:,3);
dthref = th_log(:,4);
dth = th_log(:,5);

close all;
figure;
subplot(211);
plot(time, thref, time, th);
legend({'$$ \theta_{ref} $$', '$$ \theta $$'}, 'Interpreter', 'latex');
subplot(212);
plot(time, dthref, time, dth);
legend({'$$ \dot \theta_{ref} $$', '$$ \dot \theta $$'}, 'Interpreter', 'latex');

