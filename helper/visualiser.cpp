#include "visualiser.h"

/**
 * \brief
 * Visualizes 3D object points, their colors, and camera positions with orientations using VTK.
 *
 * \param obj Input vector<Point3d> 3D object points
 * \param colors Input vector<Vec3b> color for each 3D point. If empty, points will be rendered in white
 * \param camera_positions Input vector<Point3d> positions of the cameras
 * \param camera_orientations Input vector<Vec3f> orientation of each camera in radians
 */
void sfm::vis::visualiser::start(std::vector<Point3d> &obj, std::vector<Vec3b> &colors,
                                 std::vector<Point3d> &camera_positions,
                                 std::vector<Vec3f> &camera_orientations) {
    // init
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto colorArray = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colorArray->SetNumberOfComponents(3);
    colorArray->SetName("Colors");

    for (size_t i = 0; i < obj.size(); ++i) {
        const auto &point = obj[i];
        points->InsertNextPoint(point.x, point.y, point.z);
        if (colors.empty()) {
            colorArray->InsertNextTuple3(255, 255, 255); // DEF COLOR
            continue;
        }
        colorArray->InsertNextTuple3(colors[i][2], colors[i][1], colors[i][0]); // BGR to RGB
    }

    // add points
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colorArray);

    // point object type
    auto vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexFilter->SetInputData(polyData);
    vertexFilter->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(vertexFilter->GetOutputPort());
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(3);

    // renderer window interactor
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // add the points
    renderer->AddActor(actor);

    // add cams
    if (!camera_positions.empty())
        for (size_t i = 0; i < camera_positions.size(); ++i) {
            auto cubeSource = vtkSmartPointer<vtkCubeSource>::New();
            cubeSource->SetXLength(0.25);
            cubeSource->SetYLength(0.25);
            cubeSource->SetZLength(0.25);

            auto transform = vtkSmartPointer<vtkTransform>::New();
            transform->Translate(camera_positions[i].x, camera_positions[i].y, camera_positions[i].z);

            transform->RotateX(camera_orientations[i][0] * 180.0 / M_PI);
            transform->RotateY(camera_orientations[i][1] * 180.0 / M_PI);
            transform->RotateZ(camera_orientations[i][2] * 180.0 / M_PI);

            auto transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
            transformFilter->SetInputConnection(cubeSource->GetOutputPort());
            transformFilter->SetTransform(transform);
            transformFilter->Update();

            auto cameraMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            cameraMapper->SetInputConnection(transformFilter->GetOutputPort());

            auto cameraActor = vtkSmartPointer<vtkActor>::New();
            cameraActor->SetMapper(cameraMapper);
            cameraActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // Set camera color (red)

            renderer->AddActor(cameraActor);
        }

    // background color and render
    renderer->SetBackground(1, 1, 1); // Set background color
    renderWindow->Render();
    renderWindowInteractor->Start();
}
