# Library implementation details

Here some brief details as how the library is implemented;

## ROI Calculation

The BlazeFace model, which is based on the SSD (Single Shot MultiBox Detector) framework, makes extensive use of anchoring. The model outputs 896 boxes, which represent potential locations where faces might be detected. By evaluating classifier scores, we can determine which box has the highest likelihood of containing a face.

Faces are detected in one or multiple boxes that fall within a grid composed of multiple layers. In this implementation, two grids of sizes 16x16 and 8x8 are used. When overlapped, they look something like this:

![output](https://github.com/CLFML/Face_Detector.Cpp/assets/95024850/3d4de0ed-6f02-43dd-b3a3-9f840f596e4a)

The green grid represents the 8x8 grid.

The blue grid represents the 16x16 grid.

But you might ask, 16x16 + 8x8 = 320, which is not equal to 896, right? Yes, you're correct! To account for this, we add layers. Layers allow us to add more grid boxes at the same location, which helps increase detection accuracy by providing multiple scales and aspect ratios of anchors at each grid cell. By adding 2 layers to the 16x16 grid we get more accuracy for smaller faces and by adding 6 layers to the 8x8 grid we get more accuracy for larger faces. Which fortunately for us is the exact 896 boxes which comes out of this model ;)

To make things easier this implementation generates these anchors before inference and saves them into an array that directly maps to the model output, besides this it gives the boxes a size of 1x1 which helps with scaling it to our input image.