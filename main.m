close all;
if(exist('cam','var')==1)
    delete(cam);
end
cam = webcam;

% Load your camera calibration parameters
load('Calib_Results.mat');

% These are the 3D coordinates of the virtual object points
cara_inf = [0 1 1 0; 0 0 1/2 1/2; -1/3 -1/3 -1/3 -1/3; 1 1 1 1];
cara_sup = [0 1 1 0; 0 0 1/2 1/2; -1 -1 -1 -1; 1 1 1 1];
cara_tras = [0 0 1 1; 1/2 1/2 1/2 1/2; -1/3 -1 -1 -1/3 ; 1 1 1 1];
cara_lat = [1 1 1 1; 0 0 1/2 1/2; -1/3 -1 -1 -1/3; 1 1 1 1];

while(1)
    % Take a RGB photo
    rgb = snapshot(cam);

    % Turn it to B&W
    gray = rgb2gray(rgb);

    % Binarize the image and use the imclose function to polish the pattern borders
    bin = gray < 60;
    se = strel('disk',5);
    ero = imclose(bin, se);

    % We need to calculate the circularity of each detected region to know if they are the circles we are interested in
    areas = regionprops(ero, 'Area');
    perims = regionprops(ero, 'Perimeter');
    % And we also get the centroids
    centroids = regionprops(ero, 'Centroid');
    
    A = [];
    cx = [];
    cy = [];

    % Iterate through all the region and determine if they are circles and which one they are (small or big)
    for i = 1:(length(areas)-1)
        a = areas(i).Area;
        p = perims(i).Perimeter;
        c = centroids(i).Centroid;

        circ = (4*a*pi)/(p^2);
        if(circ > 0.85 && circ < 1.15 && a > 500)
            A = [A a];
            cx = [cx c(1)];
            cy = [cy c(2)];
        end
    end
    
    % Show the captured frame
    figure(1);
    imshow(rgb);
    hold on;
        
    if(length(A) == 4)
        % We search for the biggest circle
        [~, grande] = max(A);
        cxg = cx(grande); cyg = cy(grande);
        cx(grande) = []; cy(grande) = [];

        % And for the smallest one that is in the opposite corner
        dists = [sqrt((cxg-cx).^2 + (cyg-cy).^2)];
        
        [~, lejos] = max(dists);
        cxl = cx(lejos); cyl = cy(lejos);
        cx(lejos) = []; cy(lejos) = [];
        
        dists = [sqrt((cxg-cx).^2 + (cyg-cy).^2)];
        
        [~, cerca] = min(dists);
        cxc = cx(cerca); cyc = cy(cerca);
        cx(cerca) = []; cy(cerca) = [];
        
        % Calculate the camera extrinsics based on the centroids of the circles
        mp = [cxg cx cxl cxc; cyg cy cyl cyc];
        MP = [0 1 1 0; 0 0 1/2 1/2; 0 0 0 0];
        [~,cto,cRo,~] = compute_extrinsic(mp, MP, fc, cc, kc, alpha_c);
        
        % Homogeneus transformation matrix
        cTo = [cRo, cto; 0 0 0 1];
        
        % Extrinsics parameters matrix
        A = [fc(1), alpha_c*fc(1), cc(1); 0, fc(2), cc(2); 0 0 1];
        
        % Homogeneus coordinates to pixels
        cara_inf_c_h = A*cTo(1:3,:)*cara_inf;
        cara_sup_c_h = A*cTo(1:3,:)*cara_sup;
        cara_tras_c_h = A*cTo(1:3,:)*cara_tras;
        cara_lat_c_h = A*cTo(1:3,:)*cara_lat;
        
        % Finally from homogeneous pixels to pixels
        cara_inf_c = cara_inf_c_h(1:2,:)./cara_inf_c_h(3,:);
        cara_sup_c = cara_sup_c_h(1:2,:)./cara_sup_c_h(3,:);
        cara_tras_c = cara_tras_c_h(1:2,:)./cara_tras_c_h(3,:);
        cara_lat_c = cara_lat_c_h(1:2,:)./cara_lat_c_h(3,:);

        % Plot the circle centroids
        plot(cxg, cyg, 'g*', 'LineWidth', 6);
        plot(cxl, cyl, 'c*', 'LineWidth', 6);
        plot(cxc, cyc, 'y*', 'LineWidth', 6);
        plot(cx, cy, 'r*', 'LineWidth', 6);

        % Plot lines between the 3D points
        line([cxg cara_inf_c(1,1)], [cyg cara_inf_c(2,1)]);
        line([cx cara_inf_c(1,2)], [cy cara_inf_c(2,2)]);
        line([cxl cara_inf_c(1,3)], [cyl cara_inf_c(2,3)]);
        line([cxc cara_inf_c(1,4)], [cyc cara_inf_c(2,4)]);

        % Plot the faces of the cube
        fill(cara_inf_c(1,:), cara_inf_c(2,:), 'r');
        %fill(cara_tras_c(1,:), cara_tras_c(2,:), 'b');
        fill(cara_lat_c(1,:), cara_lat_c(2,:), 'b');
        fill(cara_sup_c(1,:), cara_sup_c(2,:), 'g');

        hold off;
    end
    
    pause(eps);
    %clf();
end