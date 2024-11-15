### Environment : Windows 10 x64 and Visual Studio 2017 
## setting
### you need：
* CGAL 4.13.1 (https://github.com/CGAL/cgal/releases/tag/releases%2FCGAL-4.13.1/) You need to CMake
* boost 1.71.0 (https://www.boost.org/users/history/version_1_71_0.html)
* OpenCV 4.0.1 (https://opencv.org/releases/)
* nanoflann(https://github.com/jlblancoc/nanoflann)
* nlohmann(https://github.com/nlohmann/json)
* OpenGL(glfw & glad)(refer to the [LearOpenGL](https://learnopengl.com/Getting-started/Creating-a-window) or the [youtube channel](https://www.youtube.com/watch?v=XpBGwZNyUh0&list=PLPaoO-vpZnumdcb4tZc4x5Q-v7CkrQ6M-))
* GLM(https://glm.g-truc.net/0.9.9/index.html)
* stb_image(https://github.com/nothings/stb/blob/master/stb_image.h)


### fold setting：

```
your folder
  ├─boost
  ├─opencv
  ├─CGAL
  ├─CGAL
  ├─include
  │   ├─glad
  │   ├─glfw
  │   ├─glm
  │   └─nlohmann
  │ 
  └─{project name}
    ├─{project name}.sln (visual studio)
    ├─x64
    └─source code(same as project name)
        ├─preprocess
        ├─shader
        └─DataSet
            ├─dataset1
            ├─dataset2
            └─ ...
                ├─contour
                ├─image
                ├─json
                ├─output
                └─shapeCloud
    
```

### VS setup
include
<img src=".\readmeImages\include.png" alt="include" style="zoom:75%;" />
linker
<img src=".\readmeImages\link.png" alt="link" style="zoom:75%;" />
                                                                                                                                                                                                      



## System overview

<img src=".\readmeImages\systemoverview.png" alt="systemoverview" style="zoom:67%;" />



## About Project

* ### preprocessing 
    * #### **canvas shape(outer contour)**
        * execute *GetContour.cpp*
        * input：image file
        * outpur：json file
        * <img src=".\readmeImages\canvas.png" alt="canvas" style="zoom: 33%;" />
    * #### **objects**
        * execute *GetObjectData.cpp*
        * input：image file
    * output：json file
        * <img src=".\readmeImages\object.png" alt="object" style="zoom:33%;" />
    
* ### main program
    * execute Main.cpp
    * input： json files of *canvas shape* & *objects*
    * operation(keyboard button)：
    * image shape cloud
        * just execute the program.
    * dynamic shape cloud
        * need to set different contours
        * <img src=".\readmeImages\dynamic.png" alt="dynamic" style="zoom:48%;" />

    

