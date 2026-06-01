%close all;
x_target = path3D_master(:,1);
y_target = path3D_master(:,2);
z_target = path3D_master(:,3);

x_out = out.x(1:length(x_target));
y_out = out.y(1:length(y_target));
z_out = out.z(1:length(z_target));

x_error = x_target - x_out*1000;
y_error = y_target - y_out*1000;
z_error = z_target - z_out*1000;

t = out.tout(1:length(x_error));

% figure('Name','Position errors on the gripper')
% subplot(2,3,1)
figure;
plot(t, x_error); grid on;
xlabel('Time (s)'); ylabel('error (mm)'); title('x error');
figure;
% subplot(2,3,2)
plot(t, y_error); grid on;
xlabel('Time (s)'); ylabel('error (mm)'); title('y error');
figure;
% subplot(2,3,3)
plot(t, z_error); grid on;
xlabel('Time (s)'); ylabel('error (mm)'); title('z error');