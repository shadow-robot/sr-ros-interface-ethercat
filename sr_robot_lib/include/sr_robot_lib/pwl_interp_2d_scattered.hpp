#include <Eigen/Dense>
#include <cmath>

int diaedg ( double x0, double y0, double x1, double y1, double x2, double y2,
  double x3, double y3 );
void i4mat_transpose_print ( int m, int n, int a[], string title );
void i4mat_transpose_print_some ( int m, int n, int a[], int ilo, int jlo,
  int ihi, int jhi, string title );
void i4vec_heap_d ( int n, int a[] );
int i4vec_min ( int n, int a[] );
void i4vec_sort_heap_a ( int n, int a[] );
int i4vec_sorted_unique ( int n, int a[] );
int lrline ( double xu, double yu, double xv1, double yv1, double xv2,
  double yv2, double dv );
int perm_check2 ( int n, int p[], int base );
void perm_inverse ( int n, int p[] );
void pwl_interp_2d_scattered_value ( int nd, double xyd[], double zd[],
  int t_num, int t[], int t_neighbor[], int ni, double xyi[], double zi[] );
int r8tris2 ( int node_num, double node_xy[], int &triangle_num,
  int triangle_node[], int triangle_neighbor[] );
int swapec ( int i, int &top, int &btri, int &bedg, int node_num,
  double node_xy[], int triangle_num, int triangle_node[],
  int triangle_neighbor[], int stack[] );
void triangulation_order3_print ( int node_num, int triangle_num,
  double node_xy[], int triangle_node[], int triangle_neighbor[] );
void triangulation_search_delaunay ( int node_num, double node_xy[],
  int triangle_order, int triangle_num, int triangle_node[],
  int triangle_neighbor[], double p[2], int &triangle_index, 
  double &alpha, double &beta, double &gamma, int &edge,
  int &step_num );
void vbedg ( double x, double y, int node_num, double node_xy[],
  int triangle_num, int triangle_node[], int triangle_neighbor[],
  int &ltri, int &ledg, int &rtri, int &redg );

void re_index_triangles ( int deleted_triangle_index, int &triangle_num,
  int triangle_node[], int triangle_neighbor[] );
void remove_exterior_triangle ( int triangle_index, int &triangle_num,
  int triangle_node[], int triangle_neighbor[] );
double get_smallest_angle ( int triangle_index, double node_xy[], int &triangle_num,
  int triangle_node[]);
void filter_edge_triangles_by_min_angle ( int node_num, double node_xy[], int &triangle_num,
  int triangle_node[], int triangle_neighbor[], double min_angle );

template<class Vector3>
std::pair<Vector3, Vector3> best_plane_from_points(const std::vector<Vector3> & c);
double evaluate_plane_point(double x, double y, std::pair<Eigen::Vector3d, Eigen::Vector3d> plane);
void add_surrounding_points(int nb_calibration_points, double node_xy[], double z_data[], int nb_surrounding_points);
