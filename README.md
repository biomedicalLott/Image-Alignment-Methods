# Image-Alignment-Methods
My exploration of image alignment methods for Robotics Algorithms
In this assignment, I implemented some basic image processing algorithms to detect features to
frame alignment using Matlab. 

Sergei Mikhailovich Prokudin-Gorskii (1863-1944) was a photographer who, between the years 1909-1915,
traveled the Russian empire and took thousands of photos of everything he saw. He used an early color
technology that involved recording three exposures of every scene onto a glass plate using a red, green,
and blue filter. Back then, there was no way to print such photos, and they had to be displayed using a
special projector. Prokudin-Gorskii left Russia in 1918, following the Russian revolution. His glass plate
negatives survived and were purchased by the Library of Congress in 1948. Today, a digitized version of the

Prokudin-Gorskii collection is available on- line at http://www.loc.gov/exhibits/empire/gorskii.html.
The goal of this assignment is to learn to work with images in Matlab/Octave by taking the digitized
Prokudin-Gorskii glass plate images and automatically producing a color image with as few visual artifacts
as possible. In order to do this, I extracted three color channel images, placed them on top of
one another, and aligned them so they would form a single RGB color image. 

Your program should take a glass plate image as input and produce a single color image as output. The
program should divide the image into three equal parts and align the second and the third parts (G and R)
to the first (B). 

Each image is a concatenation of three separate glass plate images, one each for each color channel (R, G and B).

The first alignment will be done using an L2 norm or Sum of Squared Differences. 

The second alignment is a Normalized Cross Correlation,

The final alignment uses RANSAC.

<h2>Overview</h2>
The images given to us were placed in a series of 3 channels in black and white placed along
the same plane of the image. We were tasked with extracting each channel to form a complete
red-green-blue(RGB) image and then testing different alignment methods. The methods chosen
were Sum of Square Differences(SSD), Normalized Cross Correlation(NCC), and Harris Corner
Extraction with Ransac(Harris).

<h2>Preparation</h2>
I began by removing the edges of the image clump, noting that they had thick white and black
edges. The filter would remove columns/rows until the intensity values of the image fell within a
range of 80 to 200. Then, the image was divided into thirds first by taking the image size over 3
and then searching for the blackest spot in that range
<p><img src ="https://i.imgur.com/WIDxUGP.png" alt = "Edge between images"/></p>

As you can see, this should make it the edge of the image. To speed up
the processing I used a single column/row near the center of the
image and checked for the values. Then, I cropped the images to fit
together using the smallest values for rows and columns. The result was this
unaligned image.

<p><img src = "https://i.imgur.com/iCNrzht.png" alt = "No Alignment"/></p>
<h2>Alignment 1 - SSD</h2>
To begin I normalized the image values to be between 0 and 1 and work as doubles so that the range
wasn’t restricted by uint8. Next, I ran each channel across a range of +/- 15
for shifting in the x and y axis. On each shift, the channel would be cropped and placed through
the SSD function alongside a corresponding region in the blue channel. The returning values
were stored and afterwards the minimum SSD value was found and the shit of the minimum
returned to implement the shift for both red and green channels.
<p><img src = "https://i.imgur.com/6mYDnMh.png" alt = "SSD ALignment, rshift of 4 by -10 gshift of -3 by 1"/></p>
<h2>Alignment 2 - NCC</h2>
To begin I once again normalized the image values to be between 0 and 1 followed by
running them through the same +/- 15 range as the SSD alignment. During the process i placed
similar windows into NCC where I applied a filter of otherChannel(otherChannel<= 0.8) = 0;
blueChannel(blueChannel<= 0.5) = 0; Followed by taking the dot product divided norm
of the matrix. The returning values were again stored and the maximum NCC value was used
to find and implement the shift for both red and green channels.
<p><img src = "https://i.imgur.com/06mLp0g.png" alt = "NCC ALignment, rshift of 6 by -10 gshift of -2 by 1"/></p>
<h2>Alignment 3 Part 1</h2> 
<h3>Harris</h3> 
The harris corner alignment was the pickiest of the bunch. Follow normalization, I
cropped and applied a box blur to the image which is an approximation of a gaussian
blur. With any luck this should increase the sensitivity of harris such that it locates better
edges and corners. Following this I applied sobel filters to every channel to
extract the edges. The sobel filter is an approximation of a partial derivative which acts 
as an excellent edge detector. I iterated the filter across the whole image and following this I
applied a binary mask. The binary mask works to emphasize edges and eliminate everything in 
between as it makes everything either a 0 or a 1 using a threshold of my choice that is 
programmatically decreased if the results are far too few points to actually use.
<p><img src = "https://i.imgur.com/bfd83pD.png" alt = "Thresholding the Edge Detection"/></p>

Following this, I placed channel edges through a harris response in which they were again
box blurred. From this, I was able to determine the corners from a matrix created from the
multiplied sobelized matrices. I went on to choose the best points by summing windows of
points to find the largest values and took the centers of those windows as my centers. 
I eliminated any corners too close to the edge and eliminated any corners with bad scores. 
The arrays of maximums were sorted and the best 200 were retained.
<p><img src = "https://i.imgur.com/1t4oT4s.png" alt = "Showing the detected best corners"/></p>
<h2>Alignment 3 Part 2</h2> 
<h3>Ransac</h3>
The points gathered from Harris were placed into RANSAC. Ransac chose a random corner
from the red/green channel and the blue channel. The point was then subtracted to find the shift
and the shift was applied across every one of the red/green points. Following this, I looped
through each of the now shifted points and compared them to points in the blue channel, finding
the magnitude of the distance between them. The values at the end of the loop were checked to
ensure they were below a threshold of 1 (meaning they were on top of one another).
This ransac was made to run 1000 times and in the end the shift with the maximum number
of inliers was output to align the image.
<p><img src = "https://i.imgur.com/njBIMiC.png" alt = "Harris/RANSAC alignment, rshift of 5 by -11 gshift of -3 by 1"/></p>


