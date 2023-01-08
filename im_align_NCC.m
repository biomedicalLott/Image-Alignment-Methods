%%All contents herein were created by Robert Lott for Robotics Algorithms
%%Assignment 3 due on November 20th 2021

function im_align_NCC(image, filename, saveFile)
%IM_ALIGN2 Align image using Normalized Cross Correlation
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

[b2, gmax, rmax, alignment] = nccAlignment(r,g,b,shiftMax);
rgbImage(:,:,1) = uint8(b2*bNorm);
rgbImage(:,:,2) = uint8(gmax*gNorm);
rgbImage(:,:,3) = uint8(rmax*rNorm);
figure(3)
imshow(rgbImage);
title(strcat(['ncc Alignment, rShift of ', num2str(alignment(1,1)), ...
    ' by ', num2str(alignment(1,2)), ' gshift of ',...
    num2str(alignment(2,1)), ' by ', num2str(alignment(2,2))]));
if saveFile
    [~,imagename] = fileparts(filename);
    imwrite(rgbImage, strcat([imagename,'-ncc.jpg']));
end
    function score = crosscorrelation(image1,image2)
        %CROSSCORRELATION - Take the dot prouct over the norm of the image to
        %find the highest possible value. This particular cross-correlation
        %found great success by filtering channels beforehand.
        
        %Step 1 eliminate any values that may muddy the image, increase
        %contrast
        image1(image1 <= 0.8) = 0;
        image2(image2 <= 0.5) = 0;
        %then take the normalization of the images
        dp = dot(image1,image2);
        normVal = sum(sqrt((image1.^2)+(image2.^2)));
        result = (dp/normVal);
        %and output the maximum score
        score = max(result);
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
    function[b,gmax,rmax, alignment] =  nccAlignment(r,g,b, shiftMax)
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
                %evalutes the n,m matrix of shifted images against red(having
                %that as our baseline) using cross correlation.
                greenScore(x,y) = crosscorrelation(b,cell2mat(greenHolder(x,y)));
                redScore(x,y) =  crosscorrelation(b,cell2mat(redHolder(x,y)));
                redShift(x,y) = {[i,j]};
                greenShift(x,y) = {[i,j]};
                y = y+1;
            end
            y = 1;
            x = x+1;
        end
        %finds the location of the maximum max and min scores
        [maxGreen,maxIndexGreen] = max(greenScore,[],'all','linear');
        [maxRed,maxIndexRed] = max(redScore,[],'all','linear');
        redSiz = size(redScore);
        greenSiz= size(greenScore);
        
        
        [rmaxX, rmaxY]=ind2sub(redSiz, maxIndexRed);
        [gmaxX, gmaxY]=ind2sub(greenSiz, maxIndexGreen);
        alignment = [rangevalues(rmaxX),rangevalues(rmaxY);
            rangevalues(gmaxX),rangevalues(gmaxY)];
        
        %then uses the index func to locate the related max/min
        gmax = cell2mat(greenHolder(gmaxX, gmaxY));
        rmax = cell2mat(redHolder(rmaxX,rmaxY));
        
        
    end
end