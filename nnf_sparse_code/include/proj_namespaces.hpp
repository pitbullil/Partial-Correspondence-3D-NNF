#pragma once
#ifndef PROJ_NAMESPACES_H
#define PROJ_NAMESPACES_H
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <string>
#include <algorithm>
#include <queue>
#include <cstdint>
#include <tuple>
using namespace std;

#define MAXXXX 0xffffffff
namespace dis3{

	enum nnfDest   {T, Q, TKEY, QKEY, R, RKEY, RNNF, TFEATURE, QFEATURE, NNF, NNF1, NNF5,
					NNF1COLOR, NNF5COLOR, NNFCOLOR, SIMILARITY, SIMILARITYCOLOR, RESULT,
					TSURF, QSURF, KAPPA, DIST, CLOSEST, RMESH, RBOUNDARY, TBOUNDARY,
					TSIMILARITY, TPIECE};
	enum patchMode {GEODESIC,GEONN,SPHERE,Geocut};
	enum FeatureRadiusMode {DIRECT,FRAC,FRACQ,FRACT};
	enum SurfReconMethod {POISSON,FTRIANGLE,NONE,LOAD};
	enum SimilarityMeasure {DIS,WDIS,DDIS,WBBS,BBS,NDIS,HDIS,HDISP,WDISP,MSWDIS,TRIDIS, TRIDISP};
	enum DistanceFunction {CHI2,L2F,L1F};
	enum KeypointTypes {ISS,HARRIS,SIFT,NARF,GRID,BOUND,ALL};
	enum MeshType {DUAL,NORMAL,SUPER};
	enum GeodesicType {DIJK,EXTENDED,FAST};

	const float RADIUS_SEARCH_MULTIPLIER = 3;
	const float FPFH_SEARCH_MULTIPLIER = 1.6;
	typedef pcl::PointXYZ XYZ;
	typedef pcl::PointCloud<XYZ> XYZCloud;
	typedef pcl::PointXYZL XYZL;
	typedef pcl::PointCloud<XYZL> XYZLCloud;
	typedef pcl::PointXYZI XYZI;
	typedef pcl::PointCloud<XYZI> XYZICloud;
	typedef pcl::Normal N;
	typedef pcl::PointCloud<N> NormalCloud;
	typedef pcl::PFHSignature125 PFH;	
	typedef pcl::PointCloud<PFH> PFHCloud;
	typedef pcl::FPFHSignature33 FPFH;
	typedef pcl::PointCloud<FPFH> FPFHCloud;
	typedef pcl::SHOT352 SHOT;
	typedef pcl::PointCloud<SHOT> SHOTCloud;
	typedef pcl::PointCloud<pcl::PointXYZRGBNormal> XYZRGBNCloud;
	typedef pcl::PointCloud<pcl::PointXYZRGB> XYZRGBCloud;
	typedef pcl::PointCloud<pcl::PointNormal> XYZNCloud;

	struct barycentric_polygon_s {
		int index=-1;
		float coor[3] = { 0.0 };
	};

	struct cloud_statistics_s {
		float MeanResolution, Range, MaxResolution;
	};

	struct surface_recon_params_s {
		MeshType mt = SUPER;
		SurfReconMethod Method = LOAD;
		std::string method = "LOAD";
		float searchRadius, Mu, upsamplingR, upsamplingStep, cloudResolution;
		int NN = 100, depth = 9, clean_multiplier = 1;
		bool base = true;
		bool clean = true;
		bool save = true;
		bool grid = false;
		bool normals = true;
		std::string out_path, in_path;
	};

	struct matches_stat_s {
		float exact=0, lessone=0, lesstwo=0;
	};

	struct distance_stat {
		float mu=0;
		float sigma=0;
		float exact = 0;
		float lessE = 0;
		float less2E = 0;
		float total_inrange = 0;
	};

	struct keypoint_params_s {
		double modelResolution=0;
		float salient_radius=0;
		float non_max_radius=0;
		float gamma_21 = 0.975;
		float gamma_32 = 0.975;
		int min_neighbors = 10;
		KeypointTypes Method=GRID;
		int threads;
		bool border = false;
		bool save = false;
		bool autodefault = true;
		std::string path;
	};

	struct feature_params_s {
		float searchRadius=0;
		std::string path;
	};
	
	struct vert_dis_s {//a struct which holds a float score and the originating index. used mainly to sort by the float score and give acces to the original value
		int ind;
		float dis = 0;
		bool operator > (const vert_dis_s& str) const
		{
			return (dis > str.dis);
		} 
		vert_dis_s(int i, float R = 0): ind(i),dis(R){}
	};

	typedef pair<float, float> ms_dis_s;
	typedef tuple<float, float,float> tris_dis_s;

	typedef pair<int,float> ind_dis_s;
	typedef pair<int, float> ind_r_s;

	template <class T> class pairComparemin {
	public:
		bool operator()(const T &a, const T & b) {
			return (a.second > b.second);
		};
	};

	template <class T> class pairComparemax {
	public:
		bool operator()(const T &a, const T & b) {
			return (a.second < b.second);
		};
	};


	template <typename T>
	using MINQ = priority_queue<T, vector<T>, pairComparemin<T>>;//a minimum heap

	template <typename T>
	using MAXQ = priority_queue<T, vector<T>, pairComparemax<T>>;//a maximum heap

	struct cloud_param_s {
		feature_params_s f;
		keypoint_params_s k;
		surface_recon_params_s s;
		cloud_statistics_s stat;
		std::string path, in;
		float diam, area,max_edge,mean_edge;
		float RG;//DEBUG FEATURES
		
	};

	struct template_match_parms_s {
		float R_E, R_G, frac,div_frac=0;
		patchMode pMode=GEODESIC;
		uint32_t focal_ind = MAXXXX;
		uint32_t num_obj = 1;
		float R_sample = 0.05;
		float sample_percent = 0;

		float nnf_reject_threshold = 1,r_thresh=100;
		SimilarityMeasure similarity=DIS;
		SurfReconMethod surface_method = LOAD;
		MeshType meshtype = SUPER;
		KeypointTypes keytype = ALL;
		std::string Rdir, feature = "FPFH", keypoint="ALL", radius="0", surface="", sim="DIS", patch="GEODESIC",mesh="SUPER",nnf_dir,result_dir,fractype,MDS,geotype="fast",nn="";
		bool normal_by_nn=false;
		bool mesh_normals = false;
		bool normal_features = false;
		bool load_d_t = false;
		bool load_d_q = false;
		bool store_geodesic_T = true;
		bool store_geodesic_Q = true;
		bool cut_in_template = false;
		uint32_t threads = 6;
		bool save_similarity_clouds = false;
		bool save_feature_patches = false;
		cloud_param_s T, S;
		nnfDest nnfType = NNF1;
		bool do_ransac;
		//DEBUG Parameters
		bool debug;
		uint32_t focal_gt, focal_gt_sym;
		FeatureRadiusMode RMode =  DIRECT;
		GeodesicType GeoMethod = FAST;
	};



}

#endif