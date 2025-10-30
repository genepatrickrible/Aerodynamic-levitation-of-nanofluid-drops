Overview:
Supplementary Code for our paper entitled "Aerodynamic levitation of nanofluid drops". This code can be used for future works in drop analysis.
It is designed to create a binarized video, heatmap video, characteristic shape, and frequency spectra of a levitating or falling drop.
The code is designed to use an MP4 file for the drop video and a chix file with the same name as the MP4 file for the framerate.
The code can be changed with little difficulty to manually input a framerate rather than have it search for a CHIX file which is an output file format of the Photron Nova S6 software.


Instructions:
The code is broken into two MATLAB scripts designed to be ran in order:
(1) ImageAnalysisHeatmap.m
(2) frequencySpectra.m
Each file has instructions commented at the top. We encourage thoroughly reading all instructions before running the code.
There are some dependancies in the "Functions" folder and the cache between (1) and (2) is stored in "Cache".


Common Issues:
- using "\" or "/" when referencing file paths may cause conflicts depending on the operating system. 
This issue is often resolved by changing one for the other in all locations in the files.
- If "frequencySpectra.m" is not fully processing the video, it is highly likely that the video was not binarized properly in "ImageAnalysisHeatmap.m". 
We recommend making adjustments to the editable lines in "ImageAnalysisHeatmap.m" or the "process" function contained at the bottom of the file.
Additionally, the simplest fix is to re-film the video and make sure that there is sufficient contrast between the drop and the environment.
