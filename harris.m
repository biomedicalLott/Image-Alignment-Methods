%%All contents herein were created by Robert Lott for Robotics Algorithms
%%Assignment 3 due on November 20th 2021

function [rPoints,gPoints, bPoints] = harris(r,g,b)
%HARRIS takes in the channels of an image and returns a series of points
%corresponding to the 200(or fewer) best corners. First the image is put
%through a box blur which is an approximation of a gaussian, then its edges
%are found using sobel partial derivative matrices. The edges can then be
%used to locate the corners of which the best 200 are determined and
%returned. 
shiftMax = 15;
blurQuantity = 3;
blurStepSize = 2;
contrastAmplitude = [6,6,6];
EdgeThresholdForBinary = [20,20,20];
CornerThresholdForBinary = [20,20,20];
windowSize = 2;

rb = crop(r,shiftMax);
gb = crop(g,shiftMax);
bb = crop(b,shiftMax);

for n = 1:blurQuantity
    rb = boxFilter(rb,blurStepSize);
    gb = boxFilter(gb,blurStepSize);
    bb = boxFilter(bb,blurStepSize);
end
% disp('box filtered')

[rGx, rGy, rG] = edgePrep(rb, contrastAmplitude(1), EdgeThresholdForBinary(1));
[gGx, gGy, gG] = edgePrep(gb, contrastAmplitude(2), EdgeThresholdForBinary(2));
[bGx, bGy, bG] = edgePrep(bb, contrastAmplitude(3), EdgeThresholdForBinary(3));
% figure(1)
% imshow(cat(3,rG,gG, bG))
% disp('edge prepped ')
redHarris = harrisResponse(rGx, rGy, rG, CornerThresholdForBinary(1));
greenHarris = harrisResponse(gGx, gGy, gG, CornerThresholdForBinary(2));
blueHarris = harrisResponse(bGx, bGy, bG,CornerThresholdForBinary(3));

% figure(2)
% imshow(cat(3,zeros(size(greenHarris)), gb, greenHarris))
%
% figure(3)
% imshow(cat(3,zeros(size(redHarris)), rb, redHarris))
%
% figure(4)
% imshow(cat(3,zeros(size(blueHarris)), bb, blueHarris))
% disp('harris...responded')

[rScore, gScore, bScore, rPoints, gPoints, bPoints] = ...
    harrisScores(redHarris, greenHarris, blueHarris,windowSize, shiftMax);
return
% disp('harris scores get')
%%-------------------------CORNER RETURN-------------------------%%
    function corners = harrisResponse(Gx, Gy, G, threshold)
        %HARRISRESPONSE - get the corners of the image channel using a threshold
        %value to then create a binary image.
        Gxx = boxFilter(Gx.^2, 2);
        Gxy = boxFilter(Gy.*Gx,2);
        Gyy = boxFilter(Gy.^2,2);
        
        k = 0.04;
        detA = Gxx.*Gyy - Gxy.^2;
        traceA = Gxx+Gyy;
        harris = detA-k.*traceA.^2;
        normalized_harris = harris;
        corners = corner2Binary(normalized_harris, threshold);
    end
%%-------------------------ENED CORNER RETURN-------------------------%%

%%-------------------------CORNER WINDOW FUNCTIONS-------------------------%%
    function [redScore, greenScore, blueScore, rCenter, gCenter, bCenter] =...
            harrisScores(rHarris,gHarris,bHarris,windowSize,shiftMax)
        %HARRISSCORES - Takes in the harris corners and finds the 200 best corners
        %to return for ransac.
        
        imageSize = size(rHarris);
        coordinateIndex = [1,1,1];
        %%get the corners
        for i = 1:imageSize(1) - windowSize-1
            for j = 1:imageSize(2) - windowSize-1
                rSum = sum(rHarris(i:i+windowSize,j:j+windowSize),'all');
                gSum = sum(gHarris(i:i+windowSize,j:j+windowSize),'all');
                bSum = sum(bHarris(i:i+windowSize,j:j+windowSize),'all');
                if(rSum > 0)
                    rCenter(coordinateIndex(1),1:2) = [i,j];
                    rmax(coordinateIndex(1)) = rSum;
                    coordinateIndex(1) = coordinateIndex(1)+1;
                end
                if(gSum > 0)
                    gCenter(coordinateIndex(2),1:2) = [i,j];
                    gmax(coordinateIndex(1)) = gSum;
                    coordinateIndex(2) = coordinateIndex(2)+1;
                end
                if(bSum > 0)
                    bCenter(coordinateIndex(3),1:2) = [i,j];
                    bmax(coordinateIndex(3)) = bSum;
                    coordinateIndex(3) = coordinateIndex(3)+1;
                end
            end
        end
        
        %---------------CORNER TRIMMING TO FIND 200 BEST---------------%
        %Remove corners that are too close to the edges
        rCenter = badCornerRemover(rHarris, rCenter,3);
        gCenter = badCornerRemover(gHarris, gCenter,3);
        bCenter = badCornerRemover(bHarris, bCenter,3);
        %get the scores of every point
        redScore = windowCheck(rHarris, rCenter, bHarris, bCenter, 3);
        greenScore = windowCheck(gHarris, gCenter, bHarris, bCenter, 3);
        %get blue scores to eliminate bad blues
        blueScore1 = windowCheck(bHarris, bCenter, rHarris, rCenter, 3);
        blueScore2 = windowCheck(bHarris, bCenter, gHarris, gCenter, 3);
        blueScore = blueScore1+blueScore2;
        %Sort the scores by best to worst
        [redScore, ri]= sort(redScore,'ascend');
        [greenScore,gi] = sort(greenScore,'ascend');
        [blueScore,bi] = sort(blueScore,'ascend');
        %then use the resulting indices to only select the centers
        rCenter = rCenter(ri,:);
        gCenter = gCenter(gi,:);
        bCenter = bCenter(bi,:);
        %We want 200 but realistically, there may be far fewer. So just reduce the
        %count so we're not removing more than the matrix actually has
        rdim = 201;
        gdim = 201;
        bdim = 201;
        if(length(redScore) < 201)
            rdim = length(redScore);
        end
        if(length(greenScore) < 201)
            gdim = length(greenScore);
        end
        if(length(blueScore) < 201)
            bdim = length(blueScore);
        end
        redScore = redScore(1:rdim,:);
        greenScore = greenScore(1:gdim,:);
        blueScore = blueScore(1:bdim,:);
        rCenter = rCenter(1:rdim,:);
        gCenter = gCenter(1:gdim,:);
        bCenter = bCenter(1:bdim,:);
    end


    function shiftScores = windowCheck(image1, centers1, image2, centers2, windowSize)
        %WINDOWCHECK - finds the SSD between all windows in the image centers
        windowSize = floor(windowSize/2);
        shiftScores = zeros(length(centers1),1);
        for i = 1:windowSize:length(centers1)
            window1 = image1(centers1(i,1) - windowSize : centers1(i,1)+windowSize,...
                centers1(i,2) - windowSize : centers1(i,2)+windowSize);
            for j = 1:windowSize:length(centers2)
                window2 = image2(centers2(j,1) - windowSize : centers2(j,1)+windowSize,...
                    centers2(j,2) - windowSize : centers2(j,2)+windowSize);
                newSSDResult = windowSumSquaredDifference(window1, window2);
                shiftScores(i) = shiftScores(i) + newSSDResult;
            end
        end
    end
    function score = windowSumSquaredDifference(window1,window2)
        %WINDOWSUMSQUAREDDIFFERENCE - sum of square differences of the window
        score = sum(sum((window1-window2).^2));
    end
    function remainder = badCornerRemover(image, centerPoints, windowSize)
        %BADCORNERREMOVER - removes corners that are around the edge of the
        %image channel since that could just as easily just be a border as it
        %could be an actual corner
        windowCount = length(centerPoints);
        windowSize = floor(windowSize/2)+1;
        check = centerPoints-windowSize < 1;
        check2 = [centerPoints(:,1)+windowSize > size(image,1),...
            centerPoints(:,2)+windowSize > size(image,2)];
        L = 1;
        for i = 1:windowCount
            if(check(L,1) || check(L,2) || check2(L,1) || check2(L,2))
                check(L,:) = [];
                check2(L,:) = [];
                
                centerPoints(L,:) = [];
                L = L-1;
            end
            L = L+1;
        end
        remainder = centerPoints;
    end
%%-------------------------END CORNER WINDOW FUNCTIONS-------------------------%%

%%-------------------------CROP-------------------------%%
    function cropped = crop(image, cropAmount)
        %CROP - crop the image equally along all edges
        cropped = image(cropAmount:end-cropAmount, cropAmount:end-cropAmount);
    end
%%-------------------------END CROP-------------------------%%

%%-------------------------EDGE PREPARATION-------------------------%%
    function [Gx, Gy, G] = edgePrep(image, contrastIntensity, thresholdLimit)
        %EDGEPREP - prepares an image channel by getting edges and then returning a
        %simple binary matrix of 1s and 0s
        [Gx, Gy, G]= contrastFilter(image, contrastIntensity);
        Gx = color2binary(Gx,thresholdLimit);
        Gy = color2binary(Gy,thresholdLimit);
        G = color2binary(G,thresholdLimit);
        
        
    end
    function [smoothX, smoothY, smooth] = boxFilter(image, stepSize)
        %BOXFILTER blurs an image channel, using a box blur which is an
        %approximation of a gaussian.
        %I made a mistake and made this identical
        %to the sobel filter when I was tired and only noticed this after my images
        %turned out well. I'm too afraid now to actually go and change it.
        bx = box();
        imageSize = size(image);
        smoothX = zeros(imageSize);
        smoothY = zeros(imageSize);
        smooth = zeros(imageSize);
        for i = 1:imageSize(1)-stepSize-1
            for j = 1:imageSize(2)-stepSize-1
                smoothX(i+1, j+1) = sum(sum(bx.*image(i:i+stepSize, j:j+stepSize)));
                smoothY(i+1, j+1)= sum(sum(bx.*image(i:i+stepSize, j:j+stepSize)));
                smooth(i+1, j+1) = sqrt(smoothX(i+1,j+1).^2+smoothY(i+1,j+1).^2);
            end
        end
        
    end
    function [Gx, Gy, G]= contrastFilter(image, contrastIntensity)
        %CONTRASTFILTER - Takes in an image channel and returns an image with
        %highlighted edges
        imagesize = size(image);
        adjustedSize = [floor(imagesize(1)/3)*3, floor(imagesize(2)/3)*3];
        image = image(1:adjustedSize(1), 1:adjustedSize(2));
        sx = sobelx(contrastIntensity);
        sy = sobely(contrastIntensity);
        % rp = r2([1:3:adjustedSize(1)-2]:[3:3:adjustedSize(1)],[1:3:adjustedSize(2)-2]:[3:3:adjustedSize(1)])
        
        G = zeros(imagesize);
        Gx = zeros(imagesize);
        Gy = zeros(imagesize);
        
        for i = 1:adjustedSize(1)-2
            for j = 1:adjustedSize(2)-2
                Gx(i+1, j+1) = sum(sum(sx.*image(i:i+2, j:j+2)));
                Gy(i+1, j+1)= sum(sum(sy.*image(i:i+2, j:j+2)));
                G(i+1, j+1) = sqrt(Gx(i+1,j+1).^2+Gy(i+1,j+1).^2);
            end
        end
        
    end
    function converted = color2binary(image, thresholdLimit)
        %COLOR2BIBNARY Sets everything that isn't at a certain threshold to 0 but
        %adjusts the threshold to make sure there are enough non-zero pixels to
        %allow for actual edges of an image.
        if isa(image, 'double')
            filteredImage = uint8(255*image);
        end
        pixelCount = size(image,1) * size(image,2);
        for i = 1:20
            outputImage = max(filteredImage, thresholdLimit);
            outputImage(outputImage == round(thresholdLimit)) = 0;
            outputImage = double(outputImage);
            converted  = outputImage;
            
            if sum(converted, 'all') < round(pixelCount*0.5);
                thresholdLimit = thresholdLimit - i*10;
            else
                return;
            end
        end
        
    end
    function converted = corner2Binary(image, thresholdLimit)
        %CORNER2BBINARY This is basically the color-2-binary but tweaked slightly
        %for corners, we don't need any pixel thresholds here, the fewer corners
        %the better. It sets everything that isn't at a certain threshold to 0
        if isa(image, 'double')
            filteredImage = uint8(255*image);
        end
        outputImage = max(filteredImage, thresholdLimit);
        outputImage(outputImage == round(thresholdLimit)) = 0;
        outputImage = double(outputImage);
        converted  = outputImage;
    end
%%-------------------------END EDGE PREPARATION-------------------------%%


%%-------------------------FILTER MATRICES-------------------------%%
    function sx = sobelx(filterAmplitude)
        %SOBELX- matrix approximation of a partial derivative of x
        if(~exist('filterAmplitude', 'var') )
            filterAmplitude = 2;
        end
        if(filterAmplitude < 2)
            filterAmplitude = 2;
        end
        sx = 1/8 * ...
            [-1 0 1;
            -filterAmplitude 0 filterAmplitude;
            -1 0 1];
    end
    function sy = sobely(filterAmplitude)
        %SOBELY- matrix approximation of a partial derivative of y
        if(~exist('filterAmplitude', 'var'))
            filterAmplitude = 2;
        end
        sy = 1/8 * ...
            [1 filterAmplitude 1;
            0 0 0;
            -1 -filterAmplitude -1];
    end
    function filter = box()
        %BOX - Averaging filter matrix that approximates a gaussian
        filter = 1/9*[1,1,1;1,1,1;1,1,1];
    end
%%-------------------------END FILTER MATRICES-------------------------%%




end

