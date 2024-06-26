#include <iostream>
// #include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Eigen>
#include "backward.hpp"

using namespace Eigen;

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

///Search and prune neighbors for JPS 3D
struct JPS3DNeib {
	// for each (dx,dy,dz) these contain:
	//    ns: neighbors that are always added
	//    f1: forced neighbors to check
	//    f2: neighbors to add if f1 is forced
	int ns[27][3][26];
	int f1[27][3][12];
	int f2[27][3][12];
	// nsz contains the number of neighbors for the four different types of moves:
	// no move (norm 0):        26 neighbors always added
	//                          0 forced neighbors to check (never happens)
	//                          0 neighbors to add if forced (never happens)
	// straight (norm 1):       1 neighbor always added
	//                          8 forced neighbors to check
	//                          8 neighbors to add if forced
	// diagonal (norm sqrt(2)): 3 neighbors always added
	//                          8 forced neighbors to check
	//                          12 neighbors to add if forced
	// diagonal (norm sqrt(3)): 7 neighbors always added
	//                          6 forced neighbors to check
	//                          12 neighbors to add if forced
	static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
	JPS3DNeib();
	private:
	void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
	void FNeib( int dx, int dy, int dz, int norm1, int dev,
	    int& fx, int& fy, int& fz,
	    int& nx, int& ny, int& nz);
};

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Vector3d coord; 
    Vector3i dir;   // direction of expanding
    Vector3i index;
	
    bool is_path;
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Vector3i _index, Vector3d _coord){  
		id = 0;
		is_path = false;
		index = _index;
		coord = _coord;
		dir   = Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};

class gridPathFinder
{
	public:
		double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
		double getManhHeu(GridNodePtr node1, GridNodePtr node2);
		double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
		double getHeu(GridNodePtr node1, GridNodePtr node2);

		std::vector<GridNodePtr> retrievePath(GridNodePtr current);

		double resolution, inv_resolution;
		double tie_breaker = 1.0 + 1.0 / 10000;

		std::vector<GridNodePtr> expandedNodes;
		std::vector<GridNodePtr> gridPath;
		std::vector<GridNodePtr> endPtrList;
		
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;
	
		Vector3i goalIdx;

		//uint8_t * data;

		GridNodePtr *** GridNodeMap;
		std::multimap<double, GridNodePtr> openSet;
		JPS3DNeib * jn3d;

		bool jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx);
		
		void getJpsSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets, int num_iter);
		void getSucc   (GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
		bool hasForced(const Vector3i & idx, const Vector3i & dir);

		bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Vector3i & index) const;

		Vector3d gridIndex2coord(const Vector3i & index) const;
		Vector3i coord2gridIndex(const Vector3d & pt) const;

	public:
		std::vector<Vector3d> debugNodes;
		gridPathFinder( ){				
    		jn3d = new JPS3DNeib();
		};

		~gridPathFinder(){
			delete jn3d;
		};

		void initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		void setObs(const double coord_x, const double coord_y, const double coord_z);
		void setObs(const int idx_x, const int idx_y, const int idx_z);

		void graphSearch(Vector3d start_pt, Vector3d end_pt, bool use_jps = false);
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		Vector3d coordRounding(const Vector3d & coord) const;
		std::vector<Vector3d> getPath();
		std::vector<Vector3d> getVisitedNodes();
		std::vector<Vector3d> getCloseNodes();

		uint8_t * data;
};