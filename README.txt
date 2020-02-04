MatchingDriver: The main driver for our matching program. Running the main function in this file will produce the output.
ICPDriver: The main driver for our ICP program. Running the main function in this file will produce the output.
Triangle: A data structure to hold the information about the points and edges defined by the
MatrixHelper: Concatenates and stacks matrices; also converts Lists to Matrix
BodyData, MeshData, SampleReadingsData, and Output3Data: Stores the information from each of the corresponding data files

To run our program, either go to ICPDriver or MatchingDriver in Eclipse and run; 
or compile using javac with the jama dependency and then run the main class in MatchingDriver or ICPDriver.

Running the ICP or matching driver will produce the output files in the output directory,
as well as print the error between our output and the debug outputs to console.

To test our findClosestPoints method, run the TestFCP main class, which will run
a quick unit test of the method with a known triangle and vectors.
