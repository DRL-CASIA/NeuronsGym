#ifndef MY_ICP_H
#define MY_ICP_H
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/cloud_iterator.h>

#include <pcl/features/normal_3d.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

using namespace pcl::visualization;
PCLVisualizer::Ptr vis;
bool first_init = true;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void findCorrespondences(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt,
                         pcl::Correspondences &all_correspondences, double dist)
{
  pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal> est;

  pcl::PointCloud<pcl::PointNormal>::Ptr src_cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt_cloud_normals (new pcl::PointCloud<pcl::PointNormal>);


  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setInputCloud (src);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.03);
  pcl::PointCloud<pcl::Normal>::Ptr src_normals (new pcl::PointCloud<pcl::Normal>), tgt_normals (new pcl::PointCloud<pcl::Normal>);

  ne.setInputCloud (src);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.05);
  ne.compute (*src_normals);

  ne.setInputCloud (tgt);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.05);
  ne.compute (*tgt_normals);

  pcl::concatenateFields (*src, *src_normals, *src_cloud_normals);
  pcl::concatenateFields (*tgt, *tgt_normals, *tgt_cloud_normals);
  est.setInputSource(src_cloud_normals);
  est.setInputTarget(tgt_cloud_normals);
  pcl::PCLPointCloud2::Ptr input_transformed_blob, output_transformed_blob;
  input_transformed_blob.reset (new pcl::PCLPointCloud2);
  output_transformed_blob.reset (new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2 (*src, *input_transformed_blob);
  pcl::toPCLPointCloud2(*tgt, *output_transformed_blob);
//  est.setSourceNormals(input_transformed_blob);
//  est.setTargetNormals(output_transformed_blob);
  est.determineCorrespondences(all_correspondences, dist);
}

void findTransformation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt,
                        const pcl::Correspondences &all_correspondences,
                        Eigen::Matrix4d &transform)
{
  pcl::ConstCloudIterator<pcl::PointXYZ> cloud_src(*src, all_correspondences, true);
  pcl::ConstCloudIterator<pcl::PointXYZ> cloud_tgt(*tgt, all_correspondences, false);

  const int npts = static_cast<int>(cloud_src.size());
  Eigen::Matrix<float, 3, Eigen::Dynamic> eigen_cloud_src (3, npts);
  Eigen::Matrix<float, 3, Eigen::Dynamic> eigen_cloud_tgt (3, npts);
  for (int i=0; i< npts; i++) {
    eigen_cloud_src(0, i) = cloud_src->x;
    eigen_cloud_src(1, i) = cloud_src->y;
    eigen_cloud_src(2, i) = cloud_src->z;
    ++cloud_src;

    eigen_cloud_tgt(0, i) = cloud_tgt->x;
    eigen_cloud_tgt(1, i) = cloud_tgt->y;
    eigen_cloud_tgt(2, i) = cloud_tgt->z;
    ++cloud_tgt;
  }

  cloud_src.reset (); cloud_tgt.reset ();

  transform.setIdentity ();
  Eigen::Matrix<double, 4, 1> centroid_src, centroid_tgt;
  pcl::compute3DCentroid(cloud_src, centroid_src);
  pcl::compute3DCentroid(cloud_tgt, centroid_tgt);
  cloud_src.reset (); cloud_tgt.reset ();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_tgt_demean;
  pcl::demeanPointCloud (cloud_src, centroid_src, cloud_src_demean);
  pcl::demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);

  Eigen::Matrix<double, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner (3, 3);
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 3, 3> u = svd.matrixU ();
  Eigen::Matrix<double, 3, 3> v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }
  Eigen::Matrix<double, 3, 3> R = v * u.transpose ();

  transform.topLeftCorner (3, 3) = R;
  const Eigen::Matrix<double, 3, 1> Rc (R * centroid_src.head (3));
  transform.block (0, 3, 3, 1) = centroid_tgt.head (3) - Rc;
}


void view (const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
          const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt,
          const pcl::CorrespondencesPtr &correspondences)
{
  if (!vis) return;
  PointCloudColorHandlerCustom<pcl::PointXYZ> green (tgt, 0, 255, 0);
  if (!vis->updatePointCloud<pcl::PointXYZ> (src, "source"))
  {
    vis->addPointCloud<pcl::PointXYZ> (src, "source");
//    vis->resetCameraViewpoint ("source");
  }

  if (!vis->updatePointCloud<pcl::PointXYZ> (tgt, green, "target")) vis->addPointCloud<pcl::PointXYZ> (tgt, green, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.7, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
  pcl::console::TicToc tt;
  tt.tic ();
  if (!vis->updateCorrespondences<pcl::PointXYZ> (src, tgt, *correspondences, 1))
    vis->addCorrespondences<pcl::PointXYZ> (src, tgt, *correspondences, 1, "correspondences");
  tt.toc_print ();
  vis->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, "correspondences");
  //vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
  vis->setCameraPosition(3.93924, 1.59246, 16.4494, 3.93924, 1.59246, -1,   0, 1, 0);
//  vis->saveScreenshot("/home/drl/ros_codes/qt_ws/src/lidar_ICP/icp_images/vis.png");
//  vis->spin ();

//  std::vector<pcl::visualization::Camera> cam;
//  vis->getCameras(cam);
//  cout << "Cam: " << endl
//               << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] <<", "    << cam[0].pos[3] << ")" << endl
//               << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl
//               << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
}

void random_perturbate(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
                       pcl::PointCloud<pcl::PointXYZ> &tgt)
{
  double delta_yaw = fRand(-0.05, 0.05);
  double delta_x = fRand(-0.02, 0.02);
  double delta_y = fRand(-0.02, 0.02);
  Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
  transform_(0, 0) = cos(delta_yaw);
  transform_(0, 1) = -sin(delta_yaw);
  transform_(1, 0) = sin(delta_yaw);
  transform_(1, 1) = cos(delta_yaw);
  transform_(0, 3) = delta_x;
  transform_(1, 3) = delta_y;
  pcl::transformPointCloud(*src, tgt, transform_);
}

void naive_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt,
               Eigen::Matrix4d &transform,
               int &num_correspondences)
{
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>), new_src(new pcl::PointCloud<pcl::PointXYZ>);
//  random_perturbate(src, *new_src);
  *output = *src;
  Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity ());
  int iterations = 0;
  pcl::registration::DefaultConvergenceCriteria<double> converged (iterations, transform, *all_correspondences);

  converged.setMaximumIterations(5);
  converged.setRelativeMSE(1e-3);

  double init_dist_threshold = 2.5;
  do
  {
    final_transform =  transform * final_transform;
    pcl::transformPointCloud(*src, *output, final_transform.cast<float>());

    findCorrespondences (output, tgt, *all_correspondences, init_dist_threshold);
//    PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());
    findTransformation (output, tgt, *all_correspondences, transform);

    ++iterations;
//    view (output, tgt, all_correspondences);
//    std::cout<<converged.getRelativeMSE()<<std::endl;
    if(init_dist_threshold > 0.1)
    {
      init_dist_threshold = init_dist_threshold * 0.8;
    }
  }
  while (!converged);

  num_correspondences = all_correspondences->size ();

  transform = final_transform;

}


void show_icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt,
               Eigen::Matrix4d &transform,
               int file_id)
{
  // for visualizition
  if(first_init)
  {
    vis.reset (new PCLVisualizer ("Registration example"));
    first_init = false;
  }
  else {
    vis->removeAllPointClouds();
    vis->removeCorrespondences("correspondences");
  }

  vis->setCameraPosition(3.93924, 1.59246, 16.4494, 3.93924, 1.59246, -1,   0, 1, 0);
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>), new_src(new pcl::PointCloud<pcl::PointXYZ>);
//  random_perturbate(src, *new_src);
  *output = *src;
  Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity ());
  int iterations = 0;
  pcl::registration::DefaultConvergenceCriteria<double> converged (iterations, transform, *all_correspondences);

  converged.setMaximumIterations(40);
  converged.setRelativeMSE(1e-3);

  double init_dist_threshold = 2.5;
  do
  {
    findCorrespondences (output, tgt, *all_correspondences, init_dist_threshold);
    PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());
    findTransformation (output, tgt, *all_correspondences, transform);
    final_transform = transform * final_transform;
    pcl::transformPointCloud(*new_src, *output, final_transform.cast<float>());
    ++iterations;
//    view (output, tgt, all_correspondences);
//    std::cout<<converged.getRelativeMSE()<<std::endl;
    if(init_dist_threshold > 0.2)
    {
      init_dist_threshold = init_dist_threshold * 0.8;
    }
  }
  while (!converged);
  view (output, tgt, all_correspondences);
  std::cout<<iterations<< " "<< init_dist_threshold <<std::endl;

  char file_name[100];
  std::sprintf(file_name, "/home/drl/ros_codes/qt_ws/src/lidar_ICP/icp_images/align_image_%d.png", file_id);
  vis->saveScreenshot(file_name);

  transform = final_transform;

}


#endif // MY_ICP_H
