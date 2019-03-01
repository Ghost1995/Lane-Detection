function Project_1(filename)

warning('off');

% Make two Video objects for input and output each
inputVideo = VideoReader(['..\input\' filename]);
outputVideo = VideoWriter(['..\output\' filename(1:end-4)]);
outputVideo.FrameRate = 30;
open(outputVideo)

% Yellow Threshold
% H = [25:50]; S = [25:255]; V = [125:255]
% White Threshold
% H = [0:75]; S = [0:25]; V = [175:255]

center = [];
oldCorners = [];
oldDirection = [];
while hasFrame(inputVideo)
    % Read a frame from the input video
    Iin = readFrame(inputVideo);
    % Convert RGB to HSV with uint8 type data
    Ihsv = im2uint8(rgb2hsv(Iin));
    % Do thresholding to identify the white and yellow lane markers
    Ibin = ((Ihsv(:,:,1)>=25)&(Ihsv(:,:,1)<=50)&(Ihsv(:,:,2)>=25)&(Ihsv(:,:,3)>=125))|...
            ((Ihsv(:,:,1)<=75)&(Ihsv(:,:,2)<=25)&(Ihsv(:,:,3)>=175));
    % Select region of interest
    Ibin(1:floor(0.6*size(Ibin,1)),:) = 0;
    m1 = (size(Ibin,1)-floor(0.54*size(Ibin,1)))/(1-floor(0.5*size(Ibin,2)));
    m2 = (size(Ibin,1)-floor(0.54*size(Ibin,1)))/(size(Ibin,2)-floor(0.5*size(Ibin,2)));
    c1 = (floor(0.54*size(Ibin,1))-size(Ibin,1)*floor(0.5*size(Ibin,2)))/(1-floor(0.5*size(Ibin,2)));
    c2 = (floor(0.54*size(Ibin,1))*size(Ibin,2)-size(Ibin,1)*floor(0.5*size(Ibin,2)))/(size(Ibin,2)-floor(0.5*size(Ibin,2)));
    for i=1:size(Ibin,2)
        for j=ceil(0.6*size(Ibin,1)):size(Ibin,1)
            if (j<(m1*i+c1))||(j<(m2*i+c2))
                Ibin(j,i) = 0;
            end
        end
    end
    % Perform morphological closing operation to combine sparse pixels
    Ibin = bwmorph(Ibin,'close');
    % Identify the center of the lane
    corners = zeros(4,2); % [bootomLeft; topLeft; topRight; bottomRight]
    for i=ceil(0.7*size(Ibin,1)):floor(0.9*size(Ibin,1))
        for j=floor(0.5*size(Ibin,2)):-1:1
            if Ibin(i,j)
                if sum(corners(2,:))==0
                    corners(2,:) = [j i];
                elseif (j>corners(2,1))&&(i<corners(2,2))
                    corners(2,:) = [j i];
                end
                if sum(corners(1,:))==0
                    corners(1,:) = [j i];
                elseif (j<corners(1,1))&&(i>corners(1,2))
                    corners(1,:) = [j i];
                end
                break;
            end
        end
        for j=ceil(0.5*size(Ibin,2)):size(Ibin,2)
            if Ibin(i,j)
                if sum(corners(3,:))==0
                    corners(3,:) = [j i];
                elseif (j<corners(3,1))&&(i<corners(3,2))
                    corners(3,:) = [j i];
                end
                if sum(corners(4,:))==0
                    corners(4,:) = [j i];
                elseif (j>corners(4,1))&&(i>corners(4,2))
                    corners(4,:) = [j i];
                end
                break;
            end
        end
    end
    oldCorners(:,:,1) = corners;
    lastwarn('');
    l1 = polyfit(corners(1:2,1),corners(1:2,2),1);
    l2 = polyfit(corners(3:4,1),corners(3:4,2),1);
    x = (l1(2)-l2(2))/(l2(1)-l1(1));
    if ((isempty(center))||(abs(center-x)<10))&&(isempty(lastwarn))
        center = floor(x);
    end
    % Identify the lanes
    corners(2:3,:) = 0;
    for i=ceil(0.63*size(Ibin,1)):floor(0.76*size(Ibin,1))
        for j=center:-1:1
            if Ibin(i,j)
                if (sum(corners(2,:))==0)
                    corners(2,:) = [j i];
                elseif j>corners(2,1)
                    corners(2,:) = [j i];
                end
                break;
            end
        end
        for j=center:size(Ibin,2)
            if Ibin(i,j)
                if (sum(corners(3,:))==0)
                    corners(3,:) = [j i];
                elseif j<corners(3,1)
                    corners(3,:) = [j i];
                end
                break;
            end
        end
    end
    diff = false;
    if size(oldCorners,3)~=2
        oldCorners(:,:,2) = corners;
        diff = true;
    else
        if all(all(abs(oldCorners(1:2,:,2) - corners(1:2,:))<20))
            oldCorners(1:2,:,2) = corners(1:2,:);
            diff = true;
        else
            corners(1:2,:) = oldCorners(1:2,:,2);
        end
        if all(all(abs(oldCorners(3:4,:,2) - corners(3:4,:))<20))
            oldCorners(3:4,:,2) = corners(3:4,:);
            diff = true;
        else
            corners(3:4,:) = oldCorners(3:4,:,2);
        end
    end
    % Identify direction
    l1 = polyfit(corners(1:2,1),corners(1:2,2),1);
    l2 = polyfit(corners(3:4,1),corners(3:4,2),1);
    if diff
        direction = l1(1) + l2(1);
        oldDirection = direction;
    else
        direction = oldDirection;
    end
    % Extend lanes to entire ROI
    corners = [(ceil(0.94*size(Ibin,1))-l1(2))/l1(1) ceil(0.94*size(Ibin,1));...
               (floor(0.63*size(Ibin,1))-l1(2))/l1(1) floor(0.63*size(Ibin,1));...
               (floor(0.63*size(Ibin,1))-l2(2))/l2(1) floor(0.63*size(Ibin,1));...
               (ceil(0.94*size(Ibin,1))-l2(2))/l2(1) ceil(0.94*size(Ibin,1))];
    % Insert lane markers, lane region, direction
    Iin = insertShape(Iin,'Line',[corners(1,:) corners(2,:); corners(3,:) corners(4,:)],'LineWidth',5,'Color','red');
    Iin = insertShape(Iin,'FilledPolygon',[corners(1,:) corners(2,:) corners(3,:) corners(4,:)],'Color','green','Opacity',0.5);
    if direction < -0.25
        Iin = insertText(Iin,[floor(0.65*size(Iin,1)), 50],'Left Turn Ahead','FontSize',40);
    elseif direction > -0.15
        Iin = insertText(Iin,[floor(0.65*size(Iin,1)), 50],'Right Turn Ahead','FontSize',40);
    end
    % Write the frame to the output video
    writeVideo(outputVideo,Iin)
end
close(outputVideo)

end