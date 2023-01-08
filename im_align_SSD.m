%%All contents herein were created by Robert Lott for Robotics Algorithms
%%Assignment 3 due on November 20th 2021

function im_align_SSD(image, filename,saveFile)
%IM_ALIGN1 Align image using Sum of Square Differences
if ~exist('saveFile','var')
    saveFile = false;
end
if ~exist('image', 'var')
image1 = importdata(filename);    
else
    image1 = image;
end
shiftMax = 15;

[b, bNorm] = normalizeImage(image1(:,:,1));
[g, gNorm] = normalizeImage(image1(:,:,2));
[r, rNorm] = normalizeImage(image1(:,:,3));

[b1,gmin,rmin, alignment] = ssdAlignment(r,g,b,shiftMax);

rgbImage(:,:,1) = uint8(b1*bNorm);
rgbImage(:,:,2) = uint8(gmin*gNorm);
rgbImage(:,:,3) = uint8(rmin*rNorm);
figure(2)
imshow(rgbImage);
title(strcat(['ssd Alignment, rShift of ', num2str(alignment(1,1)), ...
    ' by ', num2str(alignment(1,2)), ' gshift of ',...
    num2str(alignment(2,1)), ' by ', num2str(alignment(2,2))]));
if(saveFile)
    [~,imagename] = fileparts(filename);
    imwrite(rgbImage, strcat([imagename,'-ssd.jpg']));
end

    function score = sumsquaredifference(channel_1, channel_2)
        %SUMSQUAREDIFFERENCE - take the sum of the differences of the images
        %squared and return the value
        copy1 = (channel_1(30:end-30,80:120));
        copy2 = (channel_2(30:end-30, 80:120));
        score = sum(sum((copy1-copy2).^2));
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
    function cropped = crop(image, cropAmount)
        %crop the image along all edges in the same way
        cropped = image(cropAmount:end-cropAmount, cropAmount:end-cropAmount);
    end
    function[b,gmin,rmin, alignment] =  ssdAlignment(r,g,b, shiftMax)
        %SSDALIGNMENT - Locate the alignment required to shift the images to
        %match the blue channel by shifting across a range defined by shiftMax
        %and taking the finding the smallest sum of square differences.
        %start out by cropping r
        b = crop(b, shiftMax);
        %create a range of values to check against
        ranges = [-shiftMax,shiftMax];
        %create zero vectors for scores to simplify.
        greenScore = zeros(shiftMax);
        redScore =  zeros(shiftMax);
        siz = size(greenScore);
        x = 1;
        y = 1;
                rangevalues =[ranges(1) : ranges(2)];

        %iterate over x and y along the ranges until the images are all shifted
        for i = rangevalues
            for j = rangevalues
                %Creates a n,m cell matrix of shifted images
                greenHolder(x,y) = {crop(circshift(g, [i,j]), shiftMax)};
                redHolder(x,y) = {crop(circshift(r, [i,j]), shiftMax)};
                %evalutes the n,m matrix of shifted images against red(having that as our baseline)
                %using sum of square differences. This isn't very good yet. going
                %to try something else.
                greenScore(x,y) = sumsquaredifference(cell2mat(greenHolder(x,y)), b);
                redScore(x,y) =  sumsquaredifference(cell2mat(redHolder(x,y)), b);
                y = y+1;
            end
            y = 1;
            x = x+1;
        end
        %finds the location of the maximum max and min scores
        [minGreen,minIndexGreen] = min(greenScore,[],'all','linear');
        [minRed,minIndexRed] = min(redScore,[],'all','linear');
        blueSiz = size(redScore);
        greenSiz= size(greenScore);
        
        [rminX, rminY]=ind2sub(blueSiz, minIndexRed);
        [gminX, gminY]=ind2sub(greenSiz, minIndexGreen);
        alignment = [rangevalues(rminX),rangevalues(rminY);
            rangevalues(gminX),rangevalues(gminY)];

        %then uses the index func to locate the related max/min
        gmin = cell2mat(greenHolder(gminX, gminY));
        rmin = cell2mat(redHolder(rminX, rminY));
    end
end