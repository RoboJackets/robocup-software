function displayKalman(filename)

m = csvread(filename);
subplot(3,2,1)
plot(m(:,1),m(:,2:3));
subplot(3,2,2)
plot(m(:,1),m(:,4:5));

subplot(3,2,3)
plot(m(:,1),m(:,6:7));
subplot(3,2,4)
plot(m(:,1),m(:,8:9));

subplot(3,2,5)
plot(m(:,1),m(:,10:11));
subplot(3,2,6)
plot(m(:,1),m(:,12:13));