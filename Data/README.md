Data
====

This is the master directory for storing all of the data that is important for the program. There are three main folders in this directory. ModelData is the only directory that the user is supposed to use, the other two are for internal uses and their contents are automatically generated. 

#### /ModelData

To use this program you will need to have some sort of data 3D Model, such as a WaveFront .obj file or a .3ds file. Those files must go inside this directory. 
* Put all of your model, material, and texture files into one folder and call it something meaningful like MyBedroom.
* Put the MyBedroom folder inside of this directory, and when using the PerspectiveGenerator program you will be asked to submit the name of the Model that the perspectives should be generated from. That is when you will submit the model name, in this case 'MyBedroom'.


#### /FeatureData

For each model, a directory is created for all features computed. When `PreLocalization/DatabaseGen/src/CreateDB.cpp` is run, folders containing `.yml` files and `.jpg` files are created.



#### /RenderedImages

When the user generates images from a model inside of the */ModelData* directory using the PerspectiveGenerator program, a new directory under the same name is created here to store those generated images. 
Do not change the names of the images stored in here, as they contain the perspective information for that image.