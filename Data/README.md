Data
====

There are three main folders in this directory. Each folder has a subdirectory for each model. To begin, place an object file into its own directory within the `ModelData` folder. Do not modify generated filenames or files.


#### FeatureData

For each model, a directory is created for all features computed. When `PreLocalization/DatabaseGen/src/CreateDB.cpp` is run, folders containing `.yml` files and `.jpg` files are created.


#### ModelData

Place `.obj`, `.stl`, and `.mtl` files in their own folder for each map. The name of the folder is the name of the model and should be passed to functions.


#### RenderedImages

When images are rendered from model, the images are placed here under their folder name.