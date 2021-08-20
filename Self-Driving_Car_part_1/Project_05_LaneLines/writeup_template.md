# **Finding Lane Lines on the Road** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file. But feel free to use some other method and submit a pdf if you prefer.

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/process.png "Grayscale"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. 
1. I converted the images to grayscale 
2. Found edges using the Canny transform and Gaussian Noise kernel.
3. Restrict zone in which lanes are expected.
4. Filter edges using Hough Transform.
5. Put this edges to the base image

In order to draw a single line on the left and right lanes, I modified the draw_lines() function by extending each segment
to bottom line and middle of x.

If you'd like to include images to show how every step work on test images: 

![alt text][image1]


### 2. Identify potential shortcomings with your current pipeline

Current pipeline is simple.  
Some lines do not correspond to lanes.


### 3. Suggest possible improvements to your pipeline

It is possible found more robust parameters and ignore lines far from lanes.
