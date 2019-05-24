#include <iostream>
#include <Eigen/Dense>
#include <ctime>
#include <ratio>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <string>
using std::string;
#include "sr_robot_lib/pwl_interp_2d_scattered.hpp"
#include "sr_robot_lib/shepard_interp_2d.hpp"

  #define NB_CALIBRATION_POINTS  (25)
  #define NB_SURROUNDING_POINTS  (10)
  #define NB_TOTAL_POINTS (NB_CALIBRATION_POINTS + NB_SURROUNDING_POINTS)

  int element_neighbor[3*2*NB_TOTAL_POINTS];
  int element_num;
  int element_order = 3;
  int ni = 1;
  int node_num = NB_TOTAL_POINTS;
  //raw J1, raw J2
  double node_xy[2*NB_TOTAL_POINTS] = {
    2738, 2301,
    2693, 2154,
    2680, 1978,
    2677, 1840,
    2664, 1707,
    2334, 2287,
    2242, 2095,
    2230, 1953,
    2223, 1807,
    2206, 1685,
    1839, 2243,
    1772, 2112,
    1764, 1928,
    1755, 1762,
    1683, 1650,
    1387, 2219,
    1375, 2056,
    1370, 1884,
    1337, 1741,
    1329, 1630,
    1141, 2206,
    1132, 2055,
    1114, 1877,
    1103, 1730,
    1092, 1615
    };
  int triangle[3*2*NB_TOTAL_POINTS];
  //Sample coordinates
  double xyi[2*1];
  double zd_thj1[NB_TOTAL_POINTS] = {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.3927,
    0.3927,
    0.3927,
    0.3927,
    0.3927,
    0.7854,
    0.7854,
    0.7854,
    0.7854,
    0.7854,
    1.1781,
    1.1781,
    1.1781,
    1.1781,
    1.1781,
    1.5708,
    1.5708,
    1.5708,
    1.5708,
    1.5708
  };
  double zd_thj2[NB_TOTAL_POINTS] = {
    0.6981,
    0.34906,
    0.0,
    -0.34906,
    -0.6981,
    0.6981,
    0.34906,
    0.0,
    -0.34906,
    -0.6981,
    0.6981,
    0.34906,
    0.0,
    -0.34906,
    -0.6981,
    0.6981,
    0.34906,
    0.0,
    -0.34906,
    -0.6981,
    0.6981,
    0.34906,
    0.0,
    -0.34906,
    -0.6981
  };
  double *zi;

  double xd[NB_TOTAL_POINTS] = {
    2738,
    2693,
    2680,
    2677,
    2664,
    2334,
    2242,
    2230,
    2223,
    2206,
    1839,
    1772,
    1764,
    1755,
    1683,
    1387,
    1375,
    1370,
    1337,
    1329,
    1141,
    1132,
    1114,
    1103,
    1092
    };

   double yd[NB_TOTAL_POINTS] = {
    2301,
    2154,
    1978,
    1840,
    1707,
    2287,
    2095,
    1953,
    1807,
    1685,
    2243,
    2112,
    1928,
    1762,
    1650,
    2219,
    2056,
    1884,
    1741,
    1630,
    2206,
    2055,
    1877,
    1730,
    1615
    }; 

void print_double_array(double *data, int length, int stride, int start_at)
{
  std::stringstream ss;
  ss << "[";
  for(int i=start_at; i<length; i=i+stride)
  {
    ss << data[i];
    if ((i / stride) * stride < (length - stride))
    {
      ss << ",";
    }
  }
  ss << "]";
  std::cout << ss.str() << std::endl;
}

// https://gist.github.com/ialhashim/0a2554076a6cf32831ca
template<class Vector3>
std::pair<Vector3, Vector3> best_plane_from_points(const std::vector<Vector3> & c)
{
	// copy coordinates to  matrix in Eigen format
	size_t num_atoms = c.size();
	Eigen::Matrix< typename Vector3:: Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
	for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

	// calculate centroid
	Vector3 centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

	// subtract centroid
	coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

	// we only need the left-singular matrix here
	//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
	auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	Vector3 plane_normal = svd.matrixU().template rightCols<1>();
	return std::make_pair(centroid, plane_normal);
}

double evaluate_plane_point(double x, double y, std::pair<Eigen::Vector3d, Eigen::Vector3d> plane)
{
  // http://www.easy-math.net/transforming-between-plane-forms/
  double z = (plane.second.dot(plane.first) - plane.second(0) * x - plane.second(1) * y) / plane.second(2);
  return z;
}

void add_surrounding_points(int nb_calibration_points, double node_xy[], double z_data[], int nb_surrounding_points)
{
  std::vector<Eigen::Vector3d> calibration_points;
  for (int i = 0; i < nb_calibration_points; i++)
  {
    Eigen::Vector3d point(node_xy[i * 2], node_xy[i * 2 + 1], z_data[i]);
    calibration_points.push_back(point);
  }
  // Fit a plane to the actual calibration points
  std::pair<Eigen::Vector3d, Eigen::Vector3d> plane;
  plane = best_plane_from_points(calibration_points);

  // Find the maximum distance of a calibration point to the centroid
  // (only in XY, as we are trying to generate new XY points)
  // To determine the radius where we want the new surrounding points (perhaps R= 2 * max_distance)
  double max_distance = 0.0;
  for (int i = 0; i < nb_calibration_points; i++)
  {
    max_distance = std::max(max_distance, (calibration_points[i].head(2) - plane.first.head(2)).norm());
  }

  double radius = 2 * max_distance;
  double angular_interval = 2 * M_PI / nb_surrounding_points;
  for (int i = 0; i < nb_surrounding_points; i++)
  {
    // https://stackoverflow.com/questions/5300938/calculating-the-position-of-points-in-a-circle/5300976
    // We generate points in a circle (in xy) around the actual calibration points
    // then evaluate their z by projecting them on the plane that we fitted to the calibration points
    double theta = angular_interval * i;
    double x = plane.first(0) + radius * cos(theta);
    double y = plane.first(1) + radius * sin(theta);
    double z = evaluate_plane_point(x, y, plane);
    node_xy[(NB_CALIBRATION_POINTS * 2) + i * 2] = x;
    node_xy[(NB_CALIBRATION_POINTS * 2) + i * 2 + 1] = y;
    z_data[NB_CALIBRATION_POINTS + i] = z;
  }
}


int main(int argc, char** argv)
{
  int nb_samples = 10000;

  add_surrounding_points(NB_CALIBRATION_POINTS, node_xy, zd_thj1, NB_SURROUNDING_POINTS);
  // Print data points
  print_double_array(node_xy,2*NB_TOTAL_POINTS, 2, 0);
  print_double_array(node_xy, 2*NB_TOTAL_POINTS, 2, 1);
  print_double_array(zd_thj1, NB_TOTAL_POINTS, 1, 0);
  print_double_array(zd_thj2, NB_TOTAL_POINTS, 1, 0);
  //
  //  Set up the Delaunay triangulation.
  //
    r8tris2 ( node_num, node_xy, element_num, triangle, element_neighbor );

    for ( int j = 0; j < element_num; j++ )
    {
      for ( int i = 0; i < 3; i++ )
      {
        if ( 0 < element_neighbor[i+j*3] )
        {
          element_neighbor[i+j*3] = element_neighbor[i+j*3] - 1;
        }
      }
    }

    triangulation_order3_print ( node_num, element_num, node_xy, triangle, element_neighbor );


  // filter_edge_triangles_by_min_angle(node_num, node_xy, element_num, triangle, element_neighbor, 0.17);

  // triangulation_order3_print ( node_num, element_num, node_xy, triangle, element_neighbor );

  std::stringstream log_triangles;
  log_triangles << "triangles = [";
  for(int i=0; i<element_num; i++)
  {
    log_triangles << "[";
    for(int j=0; j<3; j++)
    {
      log_triangles << triangle[i * 3 + j] << ",";
    }
    log_triangles << "],";
  }
  log_triangles << "]";
  std::cout << log_triangles.str() << std::endl;

  std::stringstream log_x, log_y, log_z;
  int raw_1;
  int raw_2;
  log_x << "x = [";
  log_y << "y = [";
  log_z << "z = [";
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution_x(1000,2800);
  std::uniform_int_distribution<int> distribution_y(1550,2350);
  // std::uniform_int_distribution<int> distribution_x(950,2850);
  // std::uniform_int_distribution<int> distribution_y(1450,2450);
  // std::uniform_int_distribution<int> distribution_x(0,4000);
  // std::uniform_int_distribution<int> distribution_y(1000,4000);

  // Delaunay triangulation interpolation
  using namespace std::chrono;
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  for(int i=0; i<nb_samples; i++)
  {
    raw_1 = distribution_x(generator);
    raw_2 = distribution_y(generator);
    xyi[0] = static_cast<double> (raw_1);
    xyi[1] = static_cast<double> (raw_2);
    log_x << xyi[0] << ",";
    log_y << xyi[1] << ",";
    zi = pwl_interp_2d_scattered_value (node_num, node_xy, zd_thj1, element_num,
                                              triangle, element_neighbor, ni, xyi);
    log_z << zi[0] << ",";
    // ROS_INFO("THJ1: %f THJ2: %f Interpolated THJ1: %f", this->xyi[0], this->xyi[1], tmp_cal_value);
    delete [] zi;
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();

  duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

  std::cout << "It took " << (time_span.count() * 1000000) << " us.";
  std::cout << std::endl;

  log_x << "]";
  log_y << "]";
  log_z << "]";
  std::cout << log_x.str() << std::endl << std::endl;
  std::cout << log_y.str() << std::endl << std::endl;
  std::cout << log_z.str() << std::endl << std::endl;




  // // Shepard interpolation
  // log_x.str(std::string());
  // log_y.str(std::string());
  // log_z.str(std::string());
  // log_x << "x = [";
  // log_y << "y = [";
  // log_z << "z = [";
  // t1 = high_resolution_clock::now();
  // // the power used in the distance weighting
  // double p = 10;

  // for(int i=0; i<nb_samples; i++)
  // {
  //   raw_1 = distribution_x(generator);
  //   raw_2 = distribution_y(generator);
  //   xyi[0] = static_cast<double> (raw_1);
  //   xyi[1] = static_cast<double> (raw_2);
  //   log_x << xyi[0] << ",";
  //   log_y << xyi[1] << ",";

  //   zi = shepard_interp_2d ( node_num, xd, yd, zd_thj1, p, ni, &(xyi[0]), &(xyi[1]) );
  //   log_z << zi[0] << ",";
  //   // ROS_INFO("THJ1: %f THJ2: %f Interpolated THJ1: %f", this->xyi[0], this->xyi[1], tmp_cal_value);
  //   delete [] zi;
  // }

  // t2 = high_resolution_clock::now();

  // time_span = duration_cast<duration<double>>(t2 - t1);

  // std::cout << "Shepard took " << (time_span.count() * 1000000) << " us.";
  // std::cout << std::endl;

  // log_x << "]";
  // log_y << "]";
  // log_z << "]";
  // std::cout << log_x.str() << std::endl << std::endl;
  // std::cout << log_y.str() << std::endl << std::endl;
  // std::cout << log_z.str() << std::endl << std::endl;

  return 0;
}