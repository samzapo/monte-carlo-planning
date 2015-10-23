dir = 'jump-heightmap/'
x1 = dlmread([dir,'data1.mat'],',');
x2 = dlmread([dir,'data2.mat'],',');

plot3(x1(:,19-6),x1(:,19-5),x1(:,19-4),'r.')
hold on
plot3(x2(:,19-6),x2(:,19-5),x2(:,19-4),'b.')

axis equal;
