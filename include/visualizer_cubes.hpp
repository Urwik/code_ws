/**
 * @file visualizer_cubes.hpp
 * @brief This file contains the implementation of the addCubesToVisualizer function.
 * usage: addCubesToVisualizer(viewer, cloud, cube_size);
 */


#ifndef VISUALIZER_CUBES_HPP
#define VISUALIZER_CUBES_HPP
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkUnsignedCharArray.h>
#include <vtkRenderWindow.h>

// using pcl::PointXYZRGB = pcl::PointXYZRGB;
// using pcl::PointXYZRGB = pcl::PointXYZ;

void addCubesToVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double cube_size, const bool voxel_grid = false) {
    
    vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vtk_colors->SetName("Colors");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (voxel_grid)
    {
        std::cout << "Applying voxel grid filter. Actual size: " << cloud->size();
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(2.0*cube_size, 2.0*cube_size, 2.0*cube_size);
        sor.filter(*cloud_filtered);
        std::cout << " --->  Reduced size: " << cloud_filtered->size() << std::endl;
    }
    else
    {
        cloud_filtered = cloud;
    }

    // unsigned char color[3] = {255, 0, 0};  // Red color

    for (const auto& pt : cloud_filtered->points) {
        vtk_points->InsertNextPoint(pt.x, pt.y, pt.z);
        unsigned char color[3] = {pt.r, pt.g, pt.b};
        vtk_colors->InsertNextTypedTuple(color);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtk_points);
    polyData->GetPointData()->SetScalars(vtk_colors);

    // Create cube source
    vtkSmartPointer<vtkCubeSource> cube_source = vtkSmartPointer<vtkCubeSource>::New();
    cube_source->SetXLength(cube_size);
    cube_source->SetYLength(cube_size);
    cube_source->SetZLength(cube_size);
    
    // Create a slightly larger cube for the wireframe (for visibility)
    vtkSmartPointer<vtkCubeSource> larger_cube_source = vtkSmartPointer<vtkCubeSource>::New();
    larger_cube_source->SetXLength(cube_size + 0.0001);  // 10% larger
    larger_cube_source->SetYLength(cube_size + 0.0001);
    larger_cube_source->SetZLength(cube_size + 0.0001);


    // Replicate cubes using Glyph3D
    vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
    glyph->SetSourceConnection(cube_source->GetOutputPort());
    glyph->SetInputData(polyData);
    glyph->SetColorModeToColorByScalar();
    glyph->SetScaleMode(VTK_SCALE_BY_SCALAR);
    glyph->SetScaleFactor(1.0);  // Keep fixed size
    glyph->SetScaleModeToDataScalingOff();  // Fix cube size
    glyph->Update();

    // Mapper for cubes
    vtkSmartPointer<vtkPolyDataMapper> cube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    cube_mapper->SetInputConnection(glyph->GetOutputPort());

    // Actor for cubes
    vtkSmartPointer<vtkActor> cube_actor = vtkSmartPointer<vtkActor>::New();
    cube_actor->SetMapper(cube_mapper);
    cube_actor->GetProperty()->SetEdgeColor(0.0, 0.0, 0.0);  // Black edges
    cube_actor->GetProperty()->SetEdgeVisibility(true);

    // Add the actor using VTK 7.1 compatible function
    viewer->getRendererCollection()->GetFirstRenderer()->AddActor(cube_actor);



    // // Replicate wireframe cubes using the larger cube
    // vtkSmartPointer<vtkGlyph3D> wireframe_glyph = vtkSmartPointer<vtkGlyph3D>::New();
    // wireframe_glyph->SetSourceConnection(larger_cube_source->GetOutputPort());
    // wireframe_glyph->SetInputData(polyData);
    // // wireframe_glyph->SetColorModeToColorByScalar();
    // wireframe_glyph->SetColorModeToColorByScale();
    // wireframe_glyph->SetScaleFactor(1.0);  // Keep fixed size
    // wireframe_glyph->SetScaleModeToDataScalingOff();  // Keep fixed size
    // wireframe_glyph->Update();

    // // Mapper for wireframe
    // vtkSmartPointer<vtkPolyDataMapper> wireframe_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    // wireframe_mapper->SetInputConnection(wireframe_glyph->GetOutputPort());

    // // Actor for wireframe cubes (larger cubes)
    // vtkSmartPointer<vtkActor> wireframe_actor = vtkSmartPointer<vtkActor>::New();
    // wireframe_actor->SetMapper(wireframe_mapper);
    // wireframe_actor->GetProperty()->SetRepresentationToWireframe();
    // wireframe_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);  // Black edges
    // wireframe_actor->GetProperty()->Setcolo
    // wireframe_actor->GetProperty()->SetLineWidth(2.0);  // Adjust line thickness
    // // wireframe_actor->GetProperty()->SetDiffuse(0.0);
    // // wireframe_actor->GetProperty()->SetAmbient(0.0);
    // // wireframe_actor->GetProperty()->SetSpecular(0.0);
    // // wireframe_actor->GetProperty()->SetOpacity(1.0);
    // wireframe_actor->GetProperty()->SetRenderLinesAsTubes(true); // Smooth edges
    // wireframe_actor->GetProperty()->SetBackfaceCulling(false);
    // viewer->getRendererCollection()->GetFirstRenderer()->AddActor(wireframe_actor);


    // // OPTION 2
    // vtkSmartPointer<vtkPolyDataMapper> wireframe_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    // wireframe_mapper->SetInputConnection(glyph->GetOutputPort());

    // vtkSmartPointer<vtkActor> wireframe_actor = vtkSmartPointer<vtkActor>::New();
    // wireframe_actor->SetMapper(wireframe_mapper);
    // wireframe_actor->GetProperty()->SetRepresentationToWireframe();
    // wireframe_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);  // Force black
    // wireframe_actor->GetProperty()->SetLighting(false);
    // wireframe_actor->GetProperty()->SetDiffuse(0.0);
    // wireframe_actor->GetProperty()->SetAmbient(1.0);
    // wireframe_actor->GetProperty()->SetSpecular(0.0);
    // wireframe_actor->GetProperty()->SetOpacity(1.0);

    // viewer->getRendererCollection()->GetFirstRenderer()->AddActor(wireframe_actor);

    // // OPTION 1
    // vtkSmartPointer<vtkActor> wireframe_actor = vtkSmartPointer<vtkActor>::New();
    // wireframe_actor->SetMapper(cube_mapper);
    // wireframe_actor->GetProperty()->SetRepresentationToWireframe();
    // wireframe_actor->GetProperty()->SetLineWidth(2.0);  // Adjust line thickness
    
    // wireframe_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);  // Black edges
    // wireframe_actor->GetProperty()->SetLighting(false);
    // wireframe_actor->GetProperty()->SetDiffuse(0.0);
    // wireframe_actor->GetProperty()->SetAmbient(1.0);
    // wireframe_actor->GetProperty()->SetSpecular(0.0);
    // wireframe_actor->GetProperty()->SetOpacity(1.0);  // Ensure it's fully visible
    // // wireframe_actor->GetProperty()->SetRenderLinesAsTubes(true); // Smooth edges
    // wireframe_actor->SetForceOpaque(true);
    
    // viewer->getRendererCollection()->GetFirstRenderer()->AddActor(wireframe_actor);
}

#endif


