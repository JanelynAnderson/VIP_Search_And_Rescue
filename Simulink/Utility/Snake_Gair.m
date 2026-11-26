clc;
close all;

% Snake Properies
num_segments = 6;

% Sampling Parameters
do_animation = true;
Time_Intervel = [0 10];
T_s = 1/50;

% Constants
spatial_frequency_odd = 1;                                    % Omega_odd
spatial_frequency_even = spatial_frequency_odd;               % Omega_even
temporal_constant_odd = 2;                                    % omega_odd
temporal_constant_even = temporal_constant_odd;               % omega_even
phase_shift = -pi/2;                                          % delta
amplitude_odd = 1;                                          % A_odd
amplitude_even = amplitude_odd;                               % A_even
offset_even = 0;                                              % Beta_even
offset_odd = offset_even;                                     % Beta_odd

% Servo Properties
do_export = false;
servo_min = 150;
servo_max = 600;

%% Computations
syms n t;
% frequency (odd,even)
frequency_odd(n, t) = (spatial_frequency_odd * n + temporal_constant_odd * t);
frequency_even(n, t) = (spatial_frequency_even * n + temporal_constant_even * t);
% alpha (odd,even)
alpha_even(n,t) = offset_odd + amplitude_odd * sin(frequency_odd); 
alpha_odd(n, t) = offset_even + amplitude_even * sin(frequency_even + phase_shift);
% alpha
alpha = piecewise(mod(n, 2) == 1, alpha_odd, mod(n, 2) == 0, alpha_even);




%% Sampling
num_samples = (Time_Intervel(2) - Time_Intervel(1))/T_s;

vec_n = 1:1:num_segments;
output = zeros(size(vec_n, 2), num_samples);
for s = 1:num_samples
    time = Time_Intervel(1) + s*T_s;
    output(:, s) = subs(alpha,{n t}, {vec_n time});
end
% flip negative joints
mask = ones(size(output));
for i = 3:4:size(output, 1)
    mask(i,:) = -1*ones(1, size(output, 2));
end
for i = 4:4:size(output, 1)
    mask(i,:) = -1*ones(1, size(output, 2));
end

output = output .* mask;



% Exporting
if(do_export)
    servo_data = cast(round(servo_min + ((output(:,(1:1:((2*pi)/(gcd(temporal_constant_odd, temporal_constant_even)*T_s))))+pi./2)./pi).*(servo_max-servo_min)), "uint16");
    text = "static constexpr uint16_t servodata[";
    text = text + string(size(servo_data,2)*size(servo_data,1));
    text = text + "] = {";
    spaces = "";
    for i = 1:(strlength(text))
        spaces = spaces + " ";
    end
    for i = 1:size(servo_data,2)
        for j = 1:size(servo_data,1)
            text = text + string(servo_data(j, i));
            if( or(~isequal(i, size(servo_data,2)),~isequal(j, 1:size(servo_data,1))))
                text = text + ", ";
            end
        end
        text = text + newline + spaces;
    end
    text = text + "};";
    text = text + newline + "static constexpr uint16_t num_samples = " + size(servo_data, 2)*size(servo_data, 1) + ";";
    text = text + newline + "static constexpr uint8_t num_segments = " + size(servo_data, 1) + ";";
    text = text + newline + "static constexpr uint32_t sample_frequency = " + cast(round(1/T_s), "uint8") + ";";

    fileID = fopen("%Appdata%/../../Downloads/Servodata_Export.txt","W");
    if fileID < 0
        error("Unable to open file: %s",msg);
    end
    fwrite(fileID,text);
    fclose(fileID);
    disp("A file named 'Servodata_Export.txt' is in your downloads folder");
end

% Plotting
if(do_animation)
    snake = buildTiledRobot(num_segments);
    ax = show(snake,transpose(output(:,1)),FastUpdate=true,PreservePlot=false,DisplayBodyDetails="off");
    hold on;
    rc = rateControl(1/T_s); % 60 Hz 
    for i = 1:num_samples
        show(snake,transpose(output(:,i)),Parent=ax,FastUpdate=true,PreservePlot=false,DisplayBodyDetails="off");
        drawnow;
        waitfor(rc);
    end
    hold off;
end



clear Time_Intervel T_s spatial_frequency_odd spatial_frequency_even ...
      temporal_constant_odd temporal_constant_even phase_shift ...
      amplitude_even amplitude_odd offset_even offset_odd n t ...
      frequency_odd frequency_even alpha_even alpha_odd alpha ...
      num_samples vec_n output time s num_segments snake servo_min ...
      servo_max ax i rc j text spaces do_animation do_export fileID ...
      len