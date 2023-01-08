%%All contents herein were created by Robert Lott for Robotics Algorithms
%%Assignment 3 due on November 20th 2021

function im_align_RANSAC(image, filename, saveFile, ransacCount)
%IM_ALIGN3 - Aligns the image by first using harris corner detection filter
%to look for corners and return the points. Once the best corners are
%returned, we compare a random feature between two channels, count the
%number of inliers, and the best after X runs gives us a hopefully best-fit
%image using ransac and harris corner detection.
if ~exist('saveFile','var')
    saveFile = false
end

if ~exist('ransacCount','var')
    % third parameter does not exist, so default it to something
    ransacCount = 1000;
end

%     disp(strcat('image ',num2str(p)));
% image1 = importdata(strcat(['image',num2str(p),'-color.jpg']));
if ~exist('image', 'var')
image1 = importdata(filename);    
else
    image1 = image;
end
[b, bNorm] = normalizeImage(image1(:,:,1));
[g, gNorm] = normalizeImage(image1(:,:,2));
[r, rNorm] = normalizeImage(image1(:,:,3));
%run the harris corner finder on the image to locate the corners in the
%red, green, and bblue channels.
[rPoints, gPoints, bPoints] = harris(r,g,b);

thresh = 1;
rShift = ransac(rPoints, bPoints, ransacCount, thresh);
gShift = ransac(gPoints, bPoints, ransacCount, thresh);
%The shift amount returned is generally the opposite as i want to shift red
%and green towards blue but here i get the shift from blue to red or green.
%So i multiply by -1
rShifted = circshift(r, -1*rShift);
gShifted = circshift(g, -1*gShift);

%One time running they didn't return the same size, so this fixes that.
[rShifted, gShifted, bShifted] = makeAllSameSize(rShifted, gShifted, b);
shiftedImage(:,:,1) = bShifted;
shiftedImage(:,:,2) = gShifted;
shiftedImage(:,:,3) = rShifted;
figure(5)
imshow(shiftedImage);
title(strcat(['Harris/RANSAC Alignment, rShift of ', num2str(-rShift(1)), ...
    ' by ', num2str(-rShift(2)), ' gshift of ',...
    num2str(-gShift(1)), ' by ', num2str(-gShift(2))]));

if(saveFile)
    [~,imagename] = fileparts(filename);
    imwrite(shiftedImage, strcat([imagename,'-corner.jpg']));
    %     disp(strcat('image ',num2str(p), 'complete'));
end

    function [r,g,b] = makeAllSameSize(r,g,b)
        rsiz = size(r);
        gsiz = size(g);
        bsiz = size(b);
        xsizes = [rsiz(1);gsiz(1);bsiz(1)];
        ysizes = [rsiz(2);gsiz(2);bsiz(2)];
        
        minSize = [min(xsizes), min(ysizes)];
        r = r(1:minSize(1), 1:minSize(2));
        g = g(1:minSize(1), 1:minSize(2));
        b = b(1:minSize(1), 1:minSize(2));
        
    end



    function [point1, point2] =  chooseRandomPoints(points1, points2)
        
        point1 = points1(floor(rand()*size(points1,1))+1,1:2);
        point2 = points2(floor(rand()*size(points2,1))+1,1:2);
        
    end

    function bestShift = ransac(points1, points2, k, ransacThreshold)
        % r = Image3D(:,:,1);
        % g = Image3D(:,:,2);
        % b = Image3D(:,:,3);
        % The algorithm randomly picks a
        % feature in image 1 (lets say the B channel image) and assumes it aligns
        % with a random feature in image 2
        % (lets say, the G channel image). Calculate the pixel shift for
        % this alignment. Then apply the same pixel shift
        % to every feature in image 1, and search for a corresponding feature in
        % image 2 within a threshold (a small
        % window). If you find a feature within that window, you can count that as
        % an inlier; else you it is not. Run
        % this several times, and pick the best alignment (highest number of inliers)
        comparison = zeros(k,1);
        pixelShift = zeros(k,2);
        for i = 1:k
            [point1, point2] = chooseRandomPoints(points1, points2);
            pixelShift(i, :) = point1-point2;
            if(abs(pixelShift(i,1)) +abs(pixelShift(i,1)) > 29)
                continue;
            end
            
            shiftedPoints1 = points1 - pixelShift(i,:);
            %Take the difference, square it, take the sum across x and y points
            %after which you check to see if the resultant difference is within
            %some threshold. This will produce an array of 0s and 1s. Now sum up
            %the array and the result will be a count of how many things fell
            %within the distance of another feature.
            for p1 = 1:length(points1)
                shiftMagnitude = sqrt((shiftedPoints1(p1,1) - points2(:,1)).^2 + (shiftedPoints1(p1,2) - points2(:,2)).^2);
                comparison(i) = comparison(i)  + sum( shiftMagnitude < ransacThreshold);
            end
        end
        [~, maxPointIndex] = max(comparison);
        bestShift = pixelShift(maxPointIndex,1:2);
    end
    function [normImage, maxNorm] = normalizeImage(image)
        %normalizeImage: Normalizes the image to the maximum value
        %maximum value of a uint8
        maxNorm = 255;
        imageMax = max(image,[],'all');
        %if it's not a uint8, normalize to whatever the maximum value of the
        %image is.
        if(imageMax> maxNorm)
            maxNorm = imageMax;
        end
        normImage = double(image)/double(maxNorm);
    end
end