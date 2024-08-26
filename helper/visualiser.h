#ifndef VISUALISER_H
#define VISUALISER_H
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkProperty.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace sfm::vis {
    class visualiser {
    public:
        void start(std::vector<Point3d> &obj, std::vector<Vec3b> &colors, std::vector<Point3d> &camera_positions, std::vector<Vec3f> &
                   camera_orientations);
    };
}

#endif //VISUALISER_H
