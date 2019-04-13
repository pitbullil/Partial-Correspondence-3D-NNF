#ifndef CLOUD_NNF_MULTI_H
#define CLOUD_NNF_MULTI_H

#include <proj_namespaces.hpp>
#include <math.h>
#include <pcl/common/io.h>
#include <cloud_surface_visl.hpp>
#include <cloud_features_visl.hpp>
#include <cloud_registration_visl.hpp>
#include <cloud_key_points_visl.hpp>
#include <cloud_visualization_visl.hpp>
#include <cloud_io_visl.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/features/from_meshes.h>
#include <meshgraph.h>
#include <string.h>
#include <tuple>
#include <misc_utils.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/pca.h>
#include <pcl_fast_marching.h>
#include <QuickSort.hpp>

using namespace dis3;

class FeatureCloud {
public:
	std::string name;
	vector<uint32_t> DIS_v;
	std::string ReturnType() {
		return name;
	};

	FastMeshOMP FastT, FastQ;
	vector<vector<float>> DT, DQ;
	std::vector<uint32_t> feature_point_inds,feature_point_map;
	vector<vector<float>> TDistances;
	vector<float> Gmax, Mmax, Lmax;//maximal similarity score for normalization and aid in visualization
	//point clouds
	XYZCloud::Ptr Qc, Qkeypoints, Tc, Tkeypoints, QGridCloud, TGridCloud,result, resultkeypoints,Rboundary,TBoundary;
	//Normals
	NormalCloud::Ptr QN, TN;
	int candidates = 5;
	bool store_geodesic_T = true;
	bool store_geodesic_Q = true;

	int num_samples,model_vertices,part_vertices;
	float RTmax,R_thresh, R_thresh_ms, R_thresh_ls,density = 0.05;
	vector<MINQ<ind_dis_s>> Feature_Scores_Q;
	vector<vector<float>> r_j_t;
	vector<vector<uint32_t>> Kappa_t;
	vector<vector<uint32_t>> Kappa_j_t;
	vector<vector<uint32_t>> Patch_Buddies_t;
	vector<vector<float>> mindistances_t;//distance difference of the minimal corresponding point
	vector<uint32_t> thread_start;
	vector<uint32_t> thread_end;

	vector<vector<ind_dis_s>> Result;
	//holders for the polygons
	vector<pcl::Vertices> Qpolygons;
	vector<pcl::Vertices> Tpolygons;
	vector<vector<uint32_t>> closest_temp_t;
	MAXQ<ind_r_s> TRadiuses;//max heap used to sort the feature point maximal radiuses for optimization purposes
	vector<uint32_t> map_piece_to_full;
	vector<ind_r_s> F_R;//sorted by radiuses, ind here is the index in the feature point vector feature_point_inds - not in the cloud
	//distance from centers arrays
	vector<float> TQuasiGeoDistances;
	vector<float> SQuasiGeoDistances;
	vector<float> TRVec;
	//meshes
	pcl::PolygonMeshPtr Tmesh, Qmesh,Rmesh;
	vector<uint32_t> feasible_template_inds;
	uint32_t T_max_ind;
	uint32_t TQ_center;
	//meshes in graph format
	graphMesh T_G, Q_G,R_G;
	vector<graphMesh> Q_G_t;
	//information about the Template center
	focal_s T_f,R_f;
	vector<vector<float>> T_distances;
	vector<vector<float>> R_distances;
	vector<vector<vert_dis_s>> R_boundary_candidates;
	//vector<ind_dis_s> sim_result;
	vector<vector<ind_r_s>> scene_P_D_t; vector<ind_r_s> last_scene_P_D;

	vector<int> T_BoundaryInds;
	vector<bool> T_Boundary;
	vector<int> R_BoundaryInds;

	pcl::PointCloud<HistogramInt<5>>::Ptr DIS_cummulative,Kappacum;
	//Nearest Neighbor field holders
	pcl::PointCloud<NNFX<5>>::Ptr nnf5, resultnnf5;
	pcl::PointCloud<NNFX<1>>::Ptr nnf1, resultnnf1;
	vector<pcl::PointCloud<NNFX<1>>::Ptr> nnf_f;
	NNFX<5> resultinds;
	//correspondences for registration/visualization
	pcl::CorrespondencesPtr Corrs,filtered_closest;

	//kdtrees for eucludean distances
	pcl::KdTreeFLANN<XYZ> kdtree,QGkdtree,TGkdtree;

	//Nearest neighbor fields
	XYZLCloud::Ptr nnf, resultnnf,DDIS_refined_NNF, Reg_points;

	//statistics clouds
	XYZICloud::Ptr similarityCloud, resultlabel, kappac, distc, distcfull;
	vector<XYZICloud::Ptr> FsimilarityCloud;
	vector<XYZICloud::Ptr> MSFsimilarityCloud;
	vector<XYZICloud::Ptr> LSFsimilarityCloud;

	vector<float> TFsimilarity;
	vector<float> TMSFsimilarity;
	vector<float> TLSFsimilarity;

	vector<vector<float>> Fsimilarity;
	vector<vector<float>> MSFsimilarity;
	vector<vector<float>> LSFsimilarity;
	vector<uint32_t> closest_matches;
	vector<float> min_distances_res;


	 PointIndices::Ptr result_indices;

	vector<XYZRGBCloud::Ptr> FsimilarityCloudColor;

	vector<PolygonMeshPtr> Feature_Patches;
	//all the matching parameters are stored here
	template_match_parms_s P;
	float grid_max_resolution;
	uint32_t res_int;
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract;


	//Kappa holds how many points in Q match the same point in T
	vector<uint32_t> Kappa_res;
	vector<uint32_t> bad_inds;
	vector<uint32_t> good_inds;

	int gopt_hl_cands = 4;//how many candidates we will look for in greedy opt

	int gopt_ml_cands = 2;//how many candidates we will look for in greedy opt

	int gopt_ll_cands = 3;//how many candidates we will look for in greedy opt

	float non_max_opt_thresh = 0.02;
	//default constructor
	FeatureCloud() {
		Qc = XYZCloud::Ptr(new XYZCloud);
		XYZCloud::Ptr Tcloud(new XYZCloud); Tc = Tcloud;
		XYZCloud::Ptr QKcloud(new XYZCloud); Qkeypoints = QKcloud;
		XYZCloud::Ptr TKcloud(new XYZCloud); Tkeypoints = TKcloud;
		XYZCloud::Ptr resultt(new XYZCloud); result = resultt;
		XYZCloud::Ptr resultk(new XYZCloud); resultkeypoints = resultk;
		NormalCloud::Ptr Qnormal(new NormalCloud); QN = Qnormal;
		NormalCloud::Ptr Tnormal(new NormalCloud); TN = Tnormal;
		XYZLCloud::Ptr temp(new XYZLCloud); nnf = temp;
		XYZLCloud::Ptr tempn(new XYZLCloud); resultnnf = tempn;
		XYZCloud::Ptr Qgcloud(new XYZCloud); QGridCloud = Qgcloud;
		XYZCloud::Ptr Tgcloud(new XYZCloud); TGridCloud = Tgcloud;
		XYZICloud::Ptr temp2(new XYZICloud); similarityCloud = temp2;
		resultlabel = XYZICloud::Ptr(new XYZICloud);
		distc = XYZICloud::Ptr(new XYZICloud);
		kappac = XYZICloud::Ptr(new XYZICloud);;
		DDIS_refined_NNF = XYZLCloud::Ptr(new XYZLCloud);
		Qmesh = pcl::PolygonMeshPtr(new pcl::PolygonMesh());
		Tmesh = pcl::PolygonMeshPtr(new pcl::PolygonMesh());
		Rmesh = pcl::PolygonMeshPtr(new pcl::PolygonMesh());
		distcfull = XYZICloud::Ptr(new XYZICloud);
		nnf1 = pcl::PointCloud<NNFX<1>>::Ptr(new pcl::PointCloud<NNFX<1>>());
		nnf5 = pcl::PointCloud<NNFX<5>>::Ptr(new pcl::PointCloud<NNFX<5>>());
		resultnnf1 = pcl::PointCloud<NNFX<1>>::Ptr(new pcl::PointCloud<NNFX<1>>());
		resultnnf5 = pcl::PointCloud<NNFX<5>>::Ptr(new pcl::PointCloud<NNFX<5>>());
		DIS_cummulative = pcl::PointCloud<HistogramInt<5>>::Ptr(new pcl::PointCloud<HistogramInt<5>>());
		Kappacum = pcl::PointCloud<HistogramInt<5>>::Ptr(new pcl::PointCloud<HistogramInt<5>>());
		Reg_points = XYZLCloud::Ptr(new XYZLCloud);
		Rboundary = XYZCloud::Ptr(new XYZCloud);
		TBoundary = XYZCloud::Ptr(new XYZCloud);

	}

	/////////////////////////////////////////Input Methods////////////////////////////////////
	//Load a grid mesh from the file in path 
	void LoadGridMesh(string path, nnfDest d);

	void LoadSurface(string path, nnfDest d);
	
	//sets the matching parameters
	void setparams(template_match_parms_s p) { P = p; }

	//returns the matching parameters
	template_match_parms_s gettparams() { return P; };

	//reads a cloud from path according to dest
	bool readCloud(string path, nnfDest d);

	//General Keypoint calculation method
	void ComputeKeyPoints(nnfDest d);
	void meshstats(nnfDest d);
	virtual void loadF(nnfDest d, string path) {};
	//Loads template center from file
	void LoadTFocal(string path);
	void LoadFeatureInds(string path);

	//Loads nearest neighbor fields from set file
	void loadNNFField();

	//Loads center from file
	void LoadFocal(string file);

	//Surface reconstruction method 
	void Reconstruct_Surface(nnfDest d);

	/////////////////////////////////////////Feature Methods////////////////////////////////////
	//normal computation - force indicates if we run over loaded data
	void computeNormal(nnfDest d,bool force);

	//Computes the nearest neighbor fields
	virtual void computeNNF() {};

	//Computes the features
	virtual void computeF(nnfDest d) {};

	//calculates feature stats
	virtual void feature_moments() = 0;

	//demean feature stats
	virtual void demean_features(nnfDest d) = 0;


	void save_distance_matrix(string path, nnfDest d);

	//Saves template center to file
	void saveTFocal(string path);

	/////////////////////////////////////////Patch collection and Stats Methods////////////////////////////////////

	void extractCandidatestats(vector<ind_r_s>& scene_P_D, uint32_t M, vector<float>& tDistances, vector<float>& mindistances, vector<uint32_t>& Kappa, vector<uint32_t>& Kappa_j, vector<uint32_t>& Patch_Buddies, vector<float>& r_j, vector<uint32_t>& closest, HistogramInt<5>& DIS, int feature);
	//returns the distance of every point on the template
	vector<float> getTemplateDistances(patchMode Mode, int i, float& R);
	//Uses euclidean regime
	vector<float> Template_Euclid_Distance(int i, float& R);
	//Uses surface for distances
	vector<float> Template_Geo_Distance(int i, float& R);
	//Projects points to their closest point on the surface
	vector<float> Template_Geo_NNDistance(int i, float& R);

	void save_template_similarity();
	vector<vector<ind_dis_s>> map_to_piece(vector<vector<ind_dis_s>> corrs, vector<uint32_t> finds);

	void extractTemplateCandidate(int i, vector<ind_r_s>& scene_P_D,
		pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract, float R, uint32_t &M, int thread = 0, bool full = false);
	//Extract window using the euclidean regime
	void extractTemplateCandidateEuc(int i, vector<ind_r_s>& scene_P_D,
		pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract, float R, uint32_t &M, bool full);
	//Extract window using point lying near a surface - used in case we sampled the surface for grid but want 
	//the entire set of points for similarity purposes
	void extractTemplateCandidateGeoNN(int i, vector<ind_r_s>& scene_P_D,
		ExtractIndices<PointXYZ>::Ptr extract, float R, int thread, bool full);
	//Extract the window using a surface mesh
	void extractTemplateCandidateGeo(int i, vector<ind_r_s>& scene_P_D, float R, uint32_t &M,int thread = 0, bool full = false);

	//Finds the focal- minimizer of the maximal distance on it's distance tree
	void calcTFocal();
	void FeatureCloud::set_sample_point();

	//extract result statistics anf clouds - NNF of result, colors scene 
	virtual void extractResult(bool res = true)=0;
	
	/////////////////////////////////////////Similarity Methods////////////////////////////////////
	//Main template matching function - calculates similarity score for the whole scene
	virtual vector<vector<ind_dis_s>> calculateSimilarity(bool Center=false)=0;
	//virtual vector<ind_dis_s> matchTemplate() = 0;

	virtual void pointSimilarity(uint32_t i, vector<vector<float>>& TDistances,vector<ind_r_s>& F_R,int t) = 0;
	ind_dis_s lower_scale_sim_max(uint32_t src, uint32_t result,int i, SimilarityMeasure sim = MSWDIS, bool TCenter = false);
	
	virtual float Similarity(int M, vector<uint32_t>& Kappa, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
		std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs, std::vector<float>& Distances) {
		return 0;
	};
	virtual ms_dis_s MSSimilarity(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
		std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs, std::vector<float>& Distances) {
		return ms_dis_s(0, 0);
	};
	float DDISfunc(int M, std::vector<uint32_t>& Kappa_j, std::vector<float>& r_j);
	float WDISfunc(int M, vector<uint32_t>& Kappa, std::vector<float>& mindistances);
	tris_dis_s MSWDISfunc(int M, vector<uint32_t>& Kappa, std::vector<float>& mindistances, vector<float>& Distances);
	tris_dis_s TRISDISfunc(int M, vector<uint32_t>& Kappa, std::vector<float>& mindistances, vector<float>& Distances);
	tris_dis_s TRIDISPfunc(int M, vector<uint32_t>& Kappa, std::vector<float>& mindistances, vector<float>& Distances);
	float WDISPfunc(int M, vector<uint32_t>& Kappa, std::vector<float>& mindistances, std::vector<float>& Distances);
	float HDISfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances);
	float HDISPfunc(int M, vector<uint32_t>& Kappa, vector<float>& r_j);

	void sim_to_nnf(vector<vector<ind_dis_s>> corrs, vector<uint32_t> finds);

	//A method to show Nearest neighbor matches and distance heat map of the matches between last extracted result and template
	void showNNF(uint32_t ID);
	void visualizeDDIS(uint32_t src, uint32_t target);
	virtual void clear_results(int i)=0;
	/**\brief extracts a template candidate centered around keypoint i and extracts similarity input informaiton
	   \param i center point index
	   \param scene_indices a container which will be filled with the candidate indices
	   \param sceneDistances a container which will be filled with the candidate points' distance from the focal
	   \param extract a filter used to extract indices into a seperate cloud for load handling
	   \param full tells if to take only keypoints or full cloud points
	   */
	void extract_all_feature_patches(string path);
	pcl::PolygonMesh::Ptr extract_feature_patch(int i, vector<ind_r_s>& scene_P_D, nnfDest Dest);
	void load_correspondences(string path,bool throw_header=false);
	void load_result(string path);
	void show_correspondences(int num);
	void show_similarity_side(int num);
	void load_feature_similarity(int num);
	void load_feature_patch(int num);
	void show_correspondences_side(int num);
	//Calculate similarity score for a single search window centered around point i in the scene
	vector<ind_r_s> extract_patch_match(uint32_t t_i, uint32_t q_i);
	void over_sample();
	virtual void zero_template_on_Q_features() = 0;

	void findResultBoundary();
	/**\brief calculates stats for the purpose of similarity calculation
	   \param scene_indices contains the candidate points indices
	   \param M the number of points in the candidate
	   \param sDistances the distance of each point from the candidate focal
	   \param tDistances the distances of each point on the template from it's focal
	   \param mindistances the minimal distance of each point in the template to one of the scene points matching it
	   \param Kappa measures how many point point to a certain template point
	   \param Kappa_j saves for each point in the window how many points point to the same template point
	   \param Patch_Buddies saves nearest neighbor for a given scene point in the subarray
	   \param r_j saves the difference between distance of point in the scene from the center and distance of it's matching template point from the template focal
	   \param DISs the DIS score
	   \param closest - saves the point in the scene with minimum r_j to the template point
	*/

		//calculates a geodesic distance matrix between the mesh vertices
	void createresultDmatrix();
	//calculates a patch similarity score to the template

	//creates a list of the indexes of the boundaries on the mesh
	void setdensesample();
	void extractTemplateBoundaryInds();
	void init_sim_structs();
	void init_global_thread_structs();

	//////////////////////////////////postprocessing methods//////////////////////////////////////
	void distance_based_filter(float mean_thresh,float d_thresh);
	vector<vector<ind_dis_s>> greedy_opt(float mean_thresh, float d_thresh);
	vector<int> extract_band_indices(float distance, float Ewidth);

	vector<vector<ind_dis_s>> Densify_correspondences();

	void Densify_template_match(int src);

	void reject_outliers();

	int Triangulate_match(int src, int detectors);

	void createRDistanceTree();
	
	uint32_t similarityMax();

	void LoadMesh(nnfDest d);

	//////////////////////////////////output methods//////////////////////////////////////

	void save_similarity(string path);

	virtual void saveCloud(string path, nnfDest d) {
		switch (d) {
		case T:savePointCloudToFile(Tc, path); break;
		case Q:savePointCloudToFile(Qc, path); break;
		case TKEY:savePointCloudToFile(Tkeypoints, path); break;
		case QKEY:savePointCloudToFile(Qkeypoints, path); break;
		case NNF: {
			switch (P.nnfType) {
			case NNF: pcl::io::savePCDFile(path, *nnf);  break;
			case NNF1:pcl::io::savePCDFile(path, *nnf1); break;
			case NNF5:pcl::io::savePCDFile(path, *nnf5); break;
			}
			break;
		}
		case RBOUNDARY:savePointCloudToFile(Rboundary, path); break;
		case NNFCOLOR: {
			XYZRGBCloud::Ptr nnf_color;
			switch (P.nnfType) {
			case NNF: nnf_color = visualizeCloud<XYZL>(nnf);  break;
			case NNF1:nnf_color = visualizeNNF<NNFX<1>>(*nnf1, Qkeypoints); break;
			case NNF5:nnf_color = visualizeNNF<NNFX<5>>(*nnf5, Qkeypoints); break;
			}
			savePointCloudToFile(nnf_color, path); break; }
		case SIMILARITY:pcl::io::savePCDFile(path, *similarityCloud); break;
		case TSIMILARITY: {
			XYZICloud::Ptr nsimilarity(FsimilarityCloud.at(0));
			float max_sim = *max_element(TFsimilarity.begin(), TFsimilarity.end());
			for (int j = 0; j < nsimilarity->size(); j++) {
				nsimilarity->at(j).intensity = TFsimilarity.at(j) / max_sim * 255;
			}
			XYZRGBCloud::Ptr sim_color = visualizeCloud<XYZI>(nsimilarity);
			//io::savePCDFile(path + "_similarity.pcd", *FsimilarityCloud.at(i));
			toPCLPointCloud2(*sim_color, Qmesh->cloud);
			io::savePLYFile(path + "_similarity.ply", *Qmesh);
			break;
		}
		case SIMILARITYCOLOR:
			for (int i=0;i<num_samples;i++){
				XYZICloud::Ptr nsimilarity(FsimilarityCloud.at(i));
				for (int j = 0; j < nsimilarity->size(); j++) {
					nsimilarity->at(j).intensity = Fsimilarity.at(i).at(j) / Gmax.at(feature_point_inds.at(i)) * 255;
				}
				XYZRGBCloud::Ptr sim_color = visualizeCloud<XYZI>(nsimilarity);
				io::savePCDFile(path + "F_" + to_string(i) + "_similarity.pcd", *FsimilarityCloud.at(i));
				toPCLPointCloud2(*sim_color, Qmesh->cloud);
				io::savePLYFile(path + "F_" + to_string(i) + "_similarity.ply", *Qmesh);

				if (P.similarity == TRIDIS) {
					XYZICloud::Ptr Msimilarity(FsimilarityCloud.at(i));
					XYZICloud::Ptr Ssimilarity(FsimilarityCloud.at(i));
					for (int j = 0; j < Msimilarity->size(); j++) {
						Msimilarity->at(j).intensity = MSFsimilarity.at(i).at(j) / Mmax.at(feature_point_inds.at(i)) * 255;
						Ssimilarity->at(j).intensity = LSFsimilarity.at(i).at(j) / Lmax.at(feature_point_inds.at(i)) * 255;
					}
					XYZRGBCloud::Ptr Msim_color = visualizeCloud<XYZI>(Msimilarity);
					toPCLPointCloud2(*Msim_color, Qmesh->cloud);
					io::savePLYFile(path + "_MF_" + to_string(i) + "_similarity.ply", *Qmesh);

					io::savePCDFile(path + "_MF_" + to_string(i) + "_similarity.pcd", *Msimilarity);
					XYZRGBCloud::Ptr Ssim_color = visualizeCloud<XYZI>(Ssimilarity);
					io::savePCDFile(path + "_SF_" + to_string(i) + "_similarity.pcd", *Ssimilarity);
					toPCLPointCloud2(*Ssim_color, Qmesh->cloud);
					io::savePLYFile(path + "_SF_" + to_string(i) + "_similarity.ply", *Qmesh);
				}
			}
			break;
		case R: {
			XYZRGBCloud::Ptr r_color = visualizeCloud<XYZI>(resultlabel, 1);
			savePointCloudToFile(r_color, path); break; }
		case RESULT:savePointCloudToFile(result, path); break;
		case RKEY:savePointCloudToFile(resultkeypoints, path); break;
		case RNNF:	
			switch (P.nnfType) {

			case NNF: pcl::io::savePCDFile(path, *resultnnf);  break;
			case NNF1:pcl::io::savePCDFile(path, *resultnnf1); break;
			case NNF5:pcl::io::savePCDFile(path, *resultnnf5); break;
			}
			break;
		case KAPPA: {
			savePointCloudToFile(kappac, path);
			XYZRGBCloud::Ptr k_color = visualizeCloud<XYZI>(kappac, 1);
			savePointCloudToFile(k_color, path+"color");
			break; }
		case DIST: {
			{
				savePointCloudToFile(distcfull, path);
				XYZRGBCloud::Ptr d_color = visualizeCloud<XYZI>(distc, 1);
				savePointCloudToFile(d_color, path + "color");
				break; 
			}
		}
		case CLOSEST: {
			{
				savePointCloudToFile(DDIS_refined_NNF, path);
				XYZRGBCloud::Ptr c_color = visualizeCloud<XYZL>(DDIS_refined_NNF);
				savePointCloudToFile(c_color, path + "color");
				break;
			}
		}

		case RMESH: {
			for (int i = 0; i < num_samples; i++) {
				extractTemplate(Result.at(feature_point_inds.at(i)).at(0).first, TRVec.at(i));
				pcl::io::savePLYFile(path+".ply", *Rmesh);
			}
			break;
		}
		case TPIECE: {
			extractTemplate(TQ_center, T_f.RG);
			pcl::io::savePLYFile(path + ".ply", *Rmesh); break;
		}
		case TSURF:break;
		case QSURF:break;
		};
	};

	cloud_statistics_s CalculateStatistics(nnfDest d, string out);

	virtual ~FeatureCloud() = default;
	//////////////////////////////////registration methods//////////////////////////////////////

	virtual void registrationmds() = 0;;
	void icp_mds();
	virtual pcl::CorrespondencesPtr matchKeyPointsFeaturesmds(float threshold)=0;
	//preform registration to the result
	virtual void registration() = 0;

	void icp();
	virtual void extractTemplate(int ind, float R) =0;
	void show_registered();
	//result picture
	void snapshot();

	//DEBUG FEATURES/METHODS
	vector<int> vertGTmap;
	vector<int> vertGTsymmap;
	/** \brief This method returns the vertices mapping from a barycentric map(closest vertex)
	\param gt_map a barycentric coordinate/polygon map
	\return vertex to vertex map according to maximal barycentric coordinate
	*/
	vector<int> Refined_GT_map(vector<barycentric_polygon_s> gt_map);

	/** \brief This method returns the distance differences from the centers between 
	           a template point and it's ground truth mapping
	\param gt_map a vertex mapping
	\return mean and var of geodesic distances
	*/
	distance_stat Distance_Difference(int center, vector<int> gt_map);

	/*returns a struct containing indices of top 5 matching window centers and their similarity score*/
	NNFX<5> extract_top();

	/*accumulates the number of exact matches on closest - 5th closest NN*/
	pcl::PointCloud<pcl::Histogram<5>>::Ptr exact_matches;
	
	/** \brief This method returns the distances from the center of other points
	    \param center the reference point
	    \param top a struct containing the query points
							*/
	vector<float> top_distances(int center, NNFX<5> top, float maxdist = INFINITY);
	
	vector<int> GT_indices(vector<barycentric_polygon_s> gt_map);
	
	vector<int> patch_indices(int i);

	void find_sample_points();
	XYZCloud::Ptr cutgt(vector<int> gt_indices);

	pcl::PointCloud<NNFX<5>>::Ptr cutnnf(vector<int> gt_indices);

	float gt_scene_ratio(vector<int> scene_indices, vector<int> gt_indices);
	/** \brief counts the number of exact matches in NNF according to the ground truth
	     a mathc is considered exact if it falls on one of the 3 polygon vertices
		\param gt_map a vactor containing the ground truth polygons*/
	vector<matches_stat_s> exact_match_counter(vector<barycentric_polygon_s> gt_map);
	HistogramInt<5> FeatureCloud::exact_match_counter(vector<int> gt_map);
	/** \brief logs distances of matches from their ground truth
	\param gt_map a vector containing the ground truth polygons*/
	pcl::PointCloud<pcl::Histogram<5>>::Ptr match_distances(vector<int> gt_map, float maxdist = INFINITY);

	/** \brief counts DIS of only ground truth points
	*/
	HistogramInt<5> GTDIS(vector<barycentric_polygon_s> gt_bar);

	/** \brief this method returns a statistics of the minimal possible 
	     distances of the ground truth points from their matches using 1-5 nearest neighbors
	    \param distogram - a histogram of distances between GT and it's match
		\param gt_map a map of Template points to Scene points
	*/
	vector<distance_stat> distance_stats(pcl::PointCloud<pcl::Histogram<5>>::Ptr distogram, vector<int> gt_map);
};

template <class CloudT, class D> class Feature1 : public FeatureCloud {
public:
	pcl::KdTreeFLANN<CloudT, D> patchFTree;
	pcl::PointCloud<CloudT> QF,TF,RF;
	CloudT MeanF;
	CloudT VarF;
	void computeF(nnfDest d) {};
	void computeNNF() {};
	Feature1() {
	}
	uint32_t center_piece_ind;
	void Feature1<CloudT, D>::extractTemplateinds(int ind, float R);
	void zero_template_on_Q_features();
	void loadF(nnfDest d, string path);
	void pointSimilarity(uint32_t i, vector<vector<float>>& TDistances, vector<ind_r_s>& F_R,int t);
	vector<ind_dis_s> calculateSimilarityband(vector<int> candidates,int origin);
	tris_dis_s Similarity(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
		std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs,std::vector<float>& Distances);
	ms_dis_s MSSimilarity(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
		std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs, std::vector<float>& Distances);
	vector<vector<ind_dis_s>> calculateSimilarity(bool Center);
	float BBSfunc(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Patch_Buddies);
	float WBBSfunc(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Patch_Buddies, vector<float>& mindistances);
	void clear_results(int i);
	void saveCloud(string path, nnfDest d);
	pcl::CorrespondencesPtr matchKeyPointsFeatures(float threshold=1);
	void extractTemplate(int ind, float R);
	void extractResult(bool res = true);
	void registration();
	void registrationmds();
	pcl::CorrespondencesPtr matchKeyPointsFeaturesmds(float threshold=1);
	~Feature1() {};
};

template<class CloudT, class D> void Feature1<CloudT, D>::zero_template_on_Q_features() {
	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
	kdtree->setInputCloud(Qc);
	float R;
	vector<int> pointIdxSearch(1);
	vector<float> pointSquaredDistance(1);

	for (int i = 0; i < Tc->size(); i++) {
		PointXYZ point = Tc->at(i);
		kdtree->nearestKSearch(point, 1, pointIdxSearch, pointSquaredDistance);
		QF.at(pointIdxSearch.at(0)) = CloudT();
	}
}


template <class CloudT> class CHI2Feature : public Feature1<CloudT, flann::ChiSquareDistance<float>> {
public:
	void computeNNF();
	void feature_moments();
	void demean_features(nnfDest d);
};

template <class CloudT> class L2Feature : public Feature1<CloudT, flann::L2_Simple<float>> {
public:
	void computeNNF();
	void feature_moments();
	void demean_features(nnfDest d);
};

class FPFHFeature : public CHI2Feature<FPFH> {
public:
	FPFHFeature() {
	}
	void computeF(nnfDest d);
};

class SFPFHFeature : public CHI2Feature<SFPFH> {
public:
	SFPFHFeature() {
	}
	void computeF(nnfDest d);
};

class GFPFHFeature : public CHI2Feature<GFPFH> {
public:
	GFPFHFeature() {
	}
	void computeF(nnfDest d);
};

class GPFHFeature : public CHI2Feature<GPFH> {
public:
	GPFHFeature() {
	}
	void computeF(nnfDest d);
};

class SHOTFeature : public L2Feature<SHOT> {
public:
	SHOTFeature() {
	}
	void computeF(nnfDest d);
};

class ROPSFeature : public L2Feature<ROPS> {
public:
	ROPSFeature() {
	}
	void computeF(nnfDest d);
};

class PFHFeature : public CHI2Feature<PFH> {
public:
	PFHFeature() {
	}
	void computeF(nnfDest d);
};

class HKSFeature : public L2Feature<HKS> {
public:
	HKSFeature() {
	}
	void computeF(nnfDest d);
};

class SIHKSFeature : public L2Feature<SIHKS> {
public:
	SIHKSFeature() {
	}
	void computeF(nnfDest d);
};

class SCHKSFeature : public L2Feature<SCHKS> {
public:
	SCHKSFeature() {
	}
	void computeF(nnfDest d);
};

class LBEIGSFeature : public L2Feature<LBEIGS> {
public:
	LBEIGSFeature() {
	}
	void computeF(nnfDest d);
};

template<class CloudT, class D> vector<vector<ind_dis_s>> Feature1<CloudT,D>::calculateSimilarity(bool Center = false) {
	Q_G_t.resize(P.threads);
	for (int i = 0; i < P.threads; i++)
		Q_G_t.at(i) = Q_G;
	TRVec.assign(num_samples, INFINITY);
	R_thresh = min(P.r_thresh * P.S.diam / 100, Center ? (T_f.RG * float(1.05)): (RTmax * float(1.05))); //threshold for sub piece radius
	if (P.similarity == MSWDIS) { R_thresh_ms = R_thresh / 2; R_thresh_ls = R_thresh / 2; };//If using 2 scales set lower scales Radius
	if (P.similarity == TRIDIS) {
		R_thresh_ms = 2 * R_thresh / 3;
		R_thresh_ls = R_thresh / 3;
	}//If using 3 scales set lower scales Radii

	cout << "1";
	Result.resize(Tc->size());//initializes a match vector for every point on the part
	DIS_cummulative->resize(model_vertices);
	Feature_Scores_Q.resize(num_samples);//resizing the Correspondences according to the desired number of feature points
	//vector<vector<float>> TDistances;//holds the distance of every feature point on the template to any other point on the mesh 
	float R;

	for (uint32_t j = 0; j < num_samples; j++) {
		TDistances.at(j) = getTemplateDistances(P.pMode, feature_point_inds.at(j), R);
		if (P.r_thresh < 100) { R = R_thresh; };
		TRadiuses.push(ind_r_s(j, R*1.10)); TRVec.at(j) = R;
	}
	F_R.clear();
	for (uint32_t j = 0; j < num_samples; j++) {//creating a list of feature points sorted by their radius from the farthest point
		F_R.push_back(TRadiuses.top()); TRadiuses.pop();
	}
	QGkdtree.setInputCloud(QGridCloud);//used for Euclidean distance deformations
	kdtree.setInputCloud(Qkeypoints);
	cout << "2";
	float max = 0;
	extract= pcl::ExtractIndices<pcl::PointXYZ>::Ptr(new ExtractIndices<pcl::PointXYZ>());
	cout << "3";
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);//used for the euclidean case
	cout << "4";
#pragma omp parallel for
	for (int t = 0; t < P.threads; t++) {//calculating similarity for each search window

		for (int i = thread_start.at(t); i < thread_end.at(t); i++) {//calculating similarity for each search window
			pointSimilarity(feasible_template_inds.at(i), TDistances, F_R, t);//sim holds the index of the point in Q and it's score
		}
	}
	cout << "5";
	//#pragma omp parallel for
	for (int i = 0; i < num_samples; i++) {//writing top results
		Result.at(feature_point_inds.at(i)).resize(candidates);


			Result.at(feature_point_inds.at(i)).at(0).first = distance(Fsimilarity.at(i).begin(), max_element(Fsimilarity.at(i).begin(), Fsimilarity.at(i).end()));
			Gmax.at(feature_point_inds.at(i)) = Fsimilarity.at(i).at(Result.at(feature_point_inds.at(i)).at(0).first);

			switch (P.similarity){
				case TRIDIS: {
					Result.at(feature_point_inds.at(i)).at(0) = lower_scale_sim_max(feature_point_inds.at(i), Result.at(feature_point_inds.at(i)).at(0).first, i, MSWDIS);//finding lower scale similarity
						Mmax.at(feature_point_inds.at(i)) = Result.at(feature_point_inds.at(i)).at(0).second;
					Result.at(feature_point_inds.at(i)).at(0) = lower_scale_sim_max(feature_point_inds.at(i), Result.at(feature_point_inds.at(i)).at(0).first, i, TRIDIS);//finding lower scale similarity
						Lmax.at(feature_point_inds.at(i)) = Result.at(feature_point_inds.at(i)).at(0).second; break;
				}
				case MSWDIS: {
				Result.at(feature_point_inds.at(i)).at(0) = lower_scale_sim_max(feature_point_inds.at(i), Result.at(feature_point_inds.at(i)).at(0).first,i, MSWDIS);//finding lower scale similarity
					Mmax.at(feature_point_inds.at(i)) = Result.at(feature_point_inds.at(i)).at(0).second; break;
				}
			}
		//}
		feature_point_map.at(i) = Result.at(feature_point_inds.at(i)).at(0).first;
	}
 	cout << "6";

	return Result;
}

template<class CloudT, class D> void Feature1<CloudT, D>::pointSimilarity(uint32_t i, vector<vector<float>>& TDistances, vector<ind_r_s>& F_R,int t) {
	scene_P_D_t.at(t).assign(model_vertices, ind_r_s(MAXUINT32, INFINITY));
	float R = F_R.at(0).second;//setting the current point maximal radius
	uint32_t M;//

	extractTemplateCandidate(i, scene_P_D_t.at(t), extract,R,M,t);//to be done for 1st feature point and trimmed down with each smaller radius point

	for (std::vector<ind_dis_s> ::const_iterator f_r = F_R.begin(); f_r < F_R.end(); ++f_r) {//j iterates over feature points
		float R = f_r->second;//setting the current point maximal radius
		int ind = f_r->first;
		//Kappacum->assign(part_vertices,0);
		closest_temp_t.at(t).assign(part_vertices, MAXUINT32);//holds the correspondences with minimal distance difference
		r_j_t.at(t).assign(model_vertices, INFINITY);
		Kappa_t.at(t).assign(part_vertices,0);
		Kappa_j_t.at(t).assign(model_vertices, 0);
		Patch_Buddies_t.at(t).assign(model_vertices, MAXUINT32);
		mindistances_t.at(t).assign(part_vertices, INFINITY);//distance difference of the minimal corresponding point
		//cout << "candidate stats\n";

		extractCandidatestats(scene_P_D_t.at(t), M, TDistances.at(ind), mindistances_t.at(t), Kappa_t.at(t),Kappa_j_t.at(t), Patch_Buddies_t.at(t), r_j_t.at(t), closest_temp_t.at(t), DIS_cummulative->at(i), ind);
		tris_dis_s sim = Similarity(M, scene_P_D_t.at(t), Kappa_t.at(t), Kappa_j_t.at(t), mindistances_t.at(t), Patch_Buddies_t.at(t), r_j_t.at(t), DIS_cummulative->at(i).counter[0], TDistances.at(ind));
		//sim_result.push_back(ind_dis_s(i,get<0>(sim)));

		switch (P.similarity) {
		case (TRIDIS): {
			MSFsimilarity.at(ind).at(i) = get<1>(sim);
			LSFsimilarity.at(ind).at(i) = get<2>(sim);//getting 3rd scale similarity score
			break;
		}

		case (MSWDIS): {
			MSFsimilarity.at(ind).at(i) = get<1>(sim);//getting 2nd scale similarity score
		}			   
		}
		Fsimilarity.at(ind).at(i) = get<0>(sim);
	};
}

template<class CloudT, class D> tris_dis_s Feature1<CloudT,D>::Similarity(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
	std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs, std::vector<float>& Distances) {
	switch (P.similarity) {
	case WDIS: return tris_dis_s(WDISfunc(M,Kappa,mindistances),0,0);
	case WDISP: return tris_dis_s(WDISPfunc(M, Kappa, mindistances, Distances), 0, 0);
	case DIS: return tris_dis_s(DISs, 0, 0);;
	case DDIS: return tris_dis_s(DDISfunc(M, Kappa_j, r_j), 0, 0);
	case BBS: return tris_dis_s(BBSfunc(M, scene_P_D,Kappa, Patch_Buddies), 0, 0);
	case WBBS: return tris_dis_s(WBBSfunc(M, scene_P_D, Kappa, Patch_Buddies, mindistances), 0, 0);
	case HDIS: return tris_dis_s(HDISfunc(M, Kappa, mindistances), 0, 0);
	case HDISP: return tris_dis_s(HDISPfunc(M, Kappa, mindistances), 0, 0);
	case MSWDIS: return MSWDISfunc(M, Kappa, mindistances, Distances);
	case TRIDISP: return TRIDISPfunc(M, Kappa, mindistances, Distances);
	case TRIDIS: return TRISDISfunc(M, Kappa, mindistances, Distances);

	default: return tris_dis_s(0,0,0);
	}
}

template<class CloudT, class D> ms_dis_s Feature1<CloudT, D>::MSSimilarity(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
	std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs, std::vector<float>& Distances) {
	switch (P.similarity) {

	default: return ms_dis_s(0,0);
	}
}

template<class CloudT, class D> void Feature1<CloudT, D>::loadF(nnfDest d, string path) {
	cout << "loading " << path << endl;
	switch (d){
	case Q: pcl::io::loadPCDFile<CloudT>(path, QF); cout << "descriptors:" << QF.size() <<endl; break;
	case T: pcl::io::loadPCDFile<CloudT>(path, TF); cout << "descriptors:" << TF.size() << endl; break;
	}
}

template<class CloudT, class D> float Feature1<CloudT,D>::BBSfunc(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa,std::vector<uint32_t>& Patch_Buddies) {
	float result = 0; std::vector<int> neigh_indices; std::vector<float> neigh_sqr_dists;
	PointCloud<CloudT>::Ptr patchCloud(new PointCloud<CloudT>);
	//cout << "1";
	for (size_t j = 0; j < M; ++j) {
		patchCloud->push_back(QF.at(scene_P_D.at(j).first));
	}
	pcl::KdTreeFLANN<CloudT, D> patchKDTree;
	patchKDTree.setInputCloud(patchCloud);
	for (size_t j = 0; j < TF.size(); ++j) {
		if (Kappa.at(j) != 0) {
			int found_neighs = patchKDTree.nearestKSearch(TF.at(j), 1, neigh_indices, neigh_sqr_dists);
			if (Patch_Buddies[neigh_indices[0]] == j) { result++; }
		}
	}
	return result;
}

template<class CloudT, class D> float Feature1<CloudT, D>::WBBSfunc(int M, vector<ind_r_s>& scene_P_D, vector<uint32_t>& Kappa, std::vector<uint32_t>& Patch_Buddies, vector<float>& mindistances) {
	float result = 0; std::vector<int> neigh_indices; std::vector<float> neigh_sqr_dists;
	PointCloud<CloudT>::Ptr patchCloud(new PointCloud<CloudT>);
	for (size_t j = 0; j < M; ++j) {
		patchCloud->push_back(QF.at(scene_P_D.at(j).first));
	}
	pcl::KdTreeFLANN<CloudT, D> patchKDTree;
	patchKDTree.setInputCloud(patchCloud);
	for (size_t j = 0; j < TF.size(); ++j) {
		if (Kappa.at(j) != 0) {
			int found_neighs = patchKDTree.nearestKSearch(TF.at(j), 1, neigh_indices, neigh_sqr_dists);
			if (Patch_Buddies[neigh_indices[0]] == j) { result+=1/(1+mindistances.at(j)); }
		}
	}
	return result;
};


template<class CloudT> void CHI2Feature<CloudT>::demean_features(nnfDest d) {
    pcl:PointCloud<CloudT>* F;
	switch (d) {
	case Q:   F = &QF; break;
	case T:   F = &TF; break;
	}
	
	for (int i = 0; i < F->size(); i++) {
		CloudT * Feat = &F->at(i);
		for (int j = 0; j < MeanF.descriptorSize(); j++) {
			Feat->histogram[j] -= MeanF.histogram[j];
		}
	}
}

template<class CloudT> void CHI2Feature<CloudT>::feature_moments() {
	for (int i = 0; i < TF.size(); i++) {
		CloudT feature = TF.at(i);
		for (int j = 0; j < MeanF.descriptorSize(); j++) {
			MeanF.histogram[j] += feature.histogram[j]/TF.size();
		}
	}

	for (int i = 0; i < TF.size(); i++) {
		CloudT feature = TF.at(i);
		for (int j = 0; j < MeanF.descriptorSize(); j++) {
			VarF.histogram[j] += pow((feature.histogram[j] - MeanF.histogram[j]), 2) / TF.size();
		}
	}
};

template<class CloudT> void L2Feature<CloudT>::demean_features(nnfDest d) {
pcl:PointCloud<CloudT>* F;
	switch (d) {
	case Q:   F = &QF; break;
	case T:   F = &TF; break;
	}

	for (int i = 0; i < F->size(); i++) {
		CloudT * Feat = &F->at(i);
		for (int j = 0; j < MeanF.descriptorSize(); j++) {
			Feat->descriptor[j] -= MeanF.descriptor[j];
		}
	}
}
template<class CloudT> void L2Feature<CloudT>::feature_moments() {
	for (int i = 0; i < TF.size(); i++) {
		CloudT feature = TF.at(i);
		for (int j = 0; j < MeanF.descriptorSize(); j++) {
			MeanF.descriptor[j] += feature.descriptor[j] / TF.size();
		}
	}

	for (int i = 0; i < TF.size(); i++) {
		CloudT feature = TF.at(i);
		for (int j = 0; j < MeanF.descriptorSize(); j++) {
			VarF.descriptor[j] += pow((feature.descriptor[j]- MeanF.descriptor[j]),2)/ TF.size();
		}
	}
};

template<class CloudT> void CHI2Feature<CloudT>::computeNNF() {
	if (P.normal_features) {
		feature_moments();
		demean_features(T);
		demean_features(Q);
	}

		if (P.nnfType == NNF) {
			copyPointCloud(*Qkeypoints, *nnf);
			computeNNFChiSquare(TF, QF, nnf, P.nnf_reject_threshold);
		}
		else if (P.nnfType == NNF1) {
			nnf1->resize(Qkeypoints->size());
			computeNNFChiSquare(TF, QF, *nnf1, P.nnf_reject_threshold);
		}
		else if (P.nnfType == NNF5) {
			nnf5->resize(Qkeypoints->size());
			computeNNFChiSquare(TF, QF, *nnf5, P.nnf_reject_threshold);
		}
	
}

template<class CloudT> void L2Feature<CloudT>::computeNNF() {
	if (P.normal_features) {
		feature_moments();
		demean_features(T);
		demean_features(Q);
	}
	copyPointCloud(*Qkeypoints,*nnf);
	if (P.nnfType == NNF) {
		copyPointCloud(*Qkeypoints, *nnf);
		computeNNFL2(TF, QF, nnf, P.nnf_reject_threshold);
	}
	else if (P.nnfType == NNF1) {
		nnf1->resize(Qkeypoints->size());
		computeNNFL2(TF, QF, *nnf1, P.nnf_reject_threshold);
	}
	else if (P.nnfType == NNF5) {
		nnf5->resize(Qkeypoints->size());
		computeNNFL2(TF, QF, *nnf5, P.nnf_reject_threshold);
	}
}

template<class CloudT, class D> void Feature1<CloudT, D>::saveCloud(string path, nnfDest d) {
	FeatureCloud::saveCloud(path, d);
	switch (d) {
	case TFEATURE:pcl::io::savePCDFile(path, TF); break;
	case QFEATURE:pcl::io::savePCDFile(path, QF); break;
	}
}


template <class CloudT, class D> void Feature1<CloudT, D>::registration() {
	CorrespondencesPtr rawCorrespondences = matchKeyPointsFeatures(); 
	CorrespondencesPtr Corrs(new pcl::Correspondences());
	Eigen::Matrix4f transform, inverseTransform, ransacTransform, ransacInverseTransform, icpInverseTransform;
	if (P.do_ransac) {
		ransac(rawCorrespondences,  resultkeypoints, Tkeypoints, *Corrs, rawCorrespondences->size() * 100, 15 * max(P.T.stat.MeanResolution, P.S.stat.MeanResolution));
	}
	else {
		Corrs = rawCorrespondences;
	}
	calcTransform(resultkeypoints, Tkeypoints, Corrs, transform);
	inverseTransform = transform.inverse();
	transformPointCloud(*Tc, *Tc, inverseTransform);
	transformPointCloud(*Tkeypoints, *Tkeypoints, inverseTransform);
	transformPointCloud(*distc, *distc, inverseTransform);
	transformPointCloud(*kappac, *kappac, inverseTransform);
	transformPointCloud(*DDIS_refined_NNF, *DDIS_refined_NNF, inverseTransform);
	pcl::PointXYZ center = Qc->at(resultinds.labels[0]);
	pcl::PointXYZ Tcenter = Tc->at(T_f.ind);
	Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
	translate(0, 3) = center.x - Tcenter.x;
	translate(1, 3) = center.y - Tcenter.y;
	translate(2, 3) = center.z - Tcenter.z;
	transformPointCloud(*Tc, *Tc, translate);
	transformPointCloud(*Tkeypoints, *Tkeypoints, translate);
	transformPointCloud(*distc, *distc, translate);
	transformPointCloud(*kappac, *kappac, translate);
	transformPointCloud(*DDIS_refined_NNF, *DDIS_refined_NNF, translate);
}

template <class CloudT, class D> void Feature1<CloudT, D>::registrationmds() {
	CorrespondencesPtr rawCorrespondences = matchKeyPointsFeatures();
	CorrespondencesPtr Corrs(new pcl::Correspondences());
	Eigen::Matrix4f transform, inverseTransform, ransacTransform, ransacInverseTransform, icpInverseTransform;
	if (P.do_ransac) {
		ransac(rawCorrespondences, Qc, Tc, *Corrs, rawCorrespondences->size() * 100, 15 * max(P.T.stat.MeanResolution, P.S.stat.MeanResolution));
	}
	else {
		Corrs = rawCorrespondences;
	}
	calcTransform(Qc, Tc, Corrs, transform);
	inverseTransform = transform.inverse();
	transformPointCloud(*Tc, *Tc, inverseTransform);
}

template<class CloudT, class D> pcl::CorrespondencesPtr Feature1<CloudT, D>::matchKeyPointsFeatures(float threshold) {
	pcl::CorrespondencesPtr correspondences(new Correspondences());
	bool only_closest = true;
	for (int i = 0; i < resultnnf5->size(); i++) {
		pcl::Correspondence corr1;
		corr1.index_query = i;
		corr1.index_match = resultnnf5->at(i).labels[0];
		corr1.distance = distcfull->at(i).intensity;
		if (!(only_closest && (corr1.distance>threshold)) && (resultnnf5->at(i).labels[0] != MAXUINT32)) {
			correspondences->push_back(corr1);
		}
	}

	return correspondences;
};

template<class CloudT, class D> pcl::CorrespondencesPtr Feature1<CloudT, D>::matchKeyPointsFeaturesmds(float threshold) {
	pcl::CorrespondencesPtr correspondences(new Correspondences());
	bool only_closest = false;
	for (int i = 0; i < resultnnf5->size(); i++) {
		pcl::Correspondence corr1;
		corr1.index_query = i;
		corr1.index_match = resultnnf5->at(i).labels[0];
		corr1.distance = distcfull->at(i).intensity;
		if (!(only_closest && (closest_matches.at(corr1.index_match) == MAXUINT32) && (corr1.distance<threshold)) && (resultnnf5->at(i).labels[0] != MAXUINT32)) {
			correspondences->push_back(corr1);
		}
	}
	return correspondences;
};

template<class CloudT, class D> void Feature1<CloudT, D>::extractTemplate(int ind, float R) {
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
	vector<ind_r_s> scenePD(model_vertices, ind_r_s(MAXUINT32, INFINITY));
	map_piece_to_full.assign(model_vertices,  INFINITY);
	uint32_t M;
	extractTemplateCandidate(ind, scenePD, extract, R,M);
	Rmesh = cut_mesh_from_points(Qmesh, vector<ind_r_s>(scenePD.begin(), scenePD.begin() + M));
	for (int i = 0; i < M; i++) map_piece_to_full.at(scenePD.at(i).first) = i;
}

template<class CloudT, class D> void Feature1<CloudT, D>::extractTemplateinds(int ind, float R) {
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
	vector<ind_r_s> scenePD(model_vertices, ind_r_s(MAXUINT32, INFINITY));
	uint32_t M;
	extractTemplateCandidate(ind, scenePD, extract, R, M);
	Rmesh = cut_mesh_from_points(Qmesh, vector<ind_r_s>(scenePD.begin(), scenePD.begin() + M));
}


template<class CloudT, class D> void Feature1<CloudT, D>::extractResult(bool res) {
	float & max = resultinds.distance[0];
	max = 0;
	for (int i = 0; i < similarityCloud->size(); i++) {
		if (similarityCloud->at(i).intensity > max) {
			max = similarityCloud->at(i).intensity; resultinds.labels[0] = i;
		}
	}
	if (res) (saveres(P.result_dir+"\\ind.txt", resultinds.labels[0]));
	extractTemplate(resultinds.labels[0],0);
};

template<class CloudT, class D> void Feature1<CloudT, D>::clear_results(int i) {
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
	vector<ind_r_s> scenePD(model_vertices, ind_r_s(MAXUINT32, INFINITY));
	uint32_t M;
	extractTemplateCandidate(TQ_center, scenePD, extract, TRVec.at(i), M);
	feasible_template_inds.clear();
	for (int j = 0; j < M; j++) {
		feasible_template_inds.push_back(scenePD.at(j).first);
		TFsimilarity.at(scenePD.at(j).first) = 0;
		if ((P.similarity == TRIDIS) || (P.similarity == MSWDIS)) {
			TMSFsimilarity.at(scenePD.at(j).first) = 0;
		}
		if ((P.similarity == TRIDIS)) {
			TLSFsimilarity.at(scenePD.at(j).first) = 0;
		}

	}
	uint32_t thread_load = M / P.threads;
	thread_start.clear();
	thread_end.clear();
	for (int t = 0; t < P.threads; t++) {
		thread_start.push_back(t*thread_load);
		thread_end.push_back((t + 1)*thread_load);
	}
	thread_end.at(P.threads - 1) = M;

}

inline float FeatureCloud::WDISfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances) {
	float WDIS_score = 0;
	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i) != 0)
		{
			WDIS_score += (1 / ((mindistances.at(i) / (P.div_frac / 100 * P.S.diam)) + 1));
		}
	}
	return WDIS_score;
};

inline tris_dis_s FeatureCloud::TRISDISfunc(int M, vector<uint32_t>& Kappa, vector<float>& mindistances, vector<float>& Distances) {
	float WDIS_score = 0;
	float MWDIS_score = 0;
	float LWDIS_score = 0;

	for (int i = 0; i < part_vertices; i++) {
		if (Kappa.at(i) != 0)
		{
			float addition = (1 / ((mindistances.at(i) / (P.div_frac / 100 * P.S.diam)) + 1));
			WDIS_score += addition;
			if (Distances.at(i) < R_thresh_ms) {
				MWDIS_score += addition;
			}
			if (Distances.at(i) < R_thresh_ls) {
				LWDIS_score += addition;
			}
		}
	}
	return tris_dis_s(WDIS_score, MWDIS_score, LWDIS_score);
};

#endif