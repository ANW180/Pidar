#include "meshrender.h"
#define EIGEN_DONT_VECTORIZE 1

meshRender::meshRender()
{

}

PolygonMesh meshRender::CreateMesh(pcl_data data)
{
        PointCloud<PointXYZ>::Ptr cloud;
        cloud = convertPointsToPTR(data.points);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        MovingLeastSquares<PointXYZ, PointXYZ> mls;
        mls.setInputCloud (cloud);
        mls.setComputeNormals (true);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.9);
//        mls.setSearchRadius (0.001);
//        mls.setPolynomialFit (true);
//        mls.setPolynomialOrder (1);
//        mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
//        mls.setUpsamplingRadius (0.005);
//        mls.setUpsamplingStepSize (0.003);

        PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
        mls.process (*cloud_smoothed);

        //cloud_smoothed = cloud;

        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;

        ne.setNumberOfThreads (8);
        ne.setInputCloud (cloud_smoothed);
        ne.setRadiusSearch (0.001);

        Eigen::Vector4f centroid;
        compute3DCentroid (*cloud_smoothed, centroid);
        ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.compute(*cloud_normals);

        for (size_t i = 0; i < cloud_normals->size (); ++i)
        {

            cloud_normals->points[i].normal_x *= -1;
            cloud_normals->points[i].normal_y *= -1;
            cloud_normals->points[i].normal_z *= -1;
        }

        PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
        concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

        Poisson<PointNormal> poisson;
        poisson.setDepth (9);
        poisson.setInputCloud (cloud_smoothed_normals);

        PolygonMesh mesh;
        poisson.reconstruct (mesh);
        mesh;
        return mesh;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr meshRender::convertPointsToPTR(std::vector<pcl_point> points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr
            (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ point;

    for(unsigned int i = 0; i < points.size(); i++)
    {

            //TODO: Convert these to cartesian
            double r = points[i].r;
            double theta = points[i].theta;
            double phi = DEG2RAD(points[i].phi);
            point.x = r * sin(theta) * cos(phi);
            point.y = r * sin(theta) * sin(phi);
            point.z = r * cos(theta);

        point_cloud_ptr->points.push_back (point);

    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    return point_cloud_ptr;
}
