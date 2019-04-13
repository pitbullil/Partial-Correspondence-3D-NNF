#ifndef CLOUD_NNF_H
#define CLOUD_NNF_H

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
#include <misc_utils.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/pca.h>
#include <omp.h>

using namespace dis3;

class FeatureCloud {
public:
	std::string name;
	vector<uint32_t> DIS_v;
	std::string ReturnType() {
		return name;
	};

	//point clouds
	XYZCloud::Ptr Qc, Qkeypoints, Tc, Tkeypoints, QGridCloud, TGridCloud,result, resultkeypoints,Rboundary,TBoundary;
	//Normals
	NormalCloud::Ptr QN, TN;

	//holders for the polygons
	vector<pcl::Vertices> Qpolygons;
	vector<pcl::Vertices> Tpolygons;

	//distance from centers arrays
	vector<float> TQuasiGeoDistances;
	vector<float> SQuasiGeoDistances;

	//meshes
	pcl::PolygonMeshPtr Tmesh, Qmesh,Rmesh;

	//meshes in graph format
	graphMesh T_G, Q_G,R_G;

	//information about the Template center
	focal_s T_f,R_f;
	vector<vector<double>> T_distances;
	vector<vector<double>> R_distances;
	vector<vector<vert_dis_s>> R_boundary_candidates;

	vector<int> T_BoundaryInds;
	vector<int> R_BoundaryInds;

	pcl::PointCloud<HistogramInt<5>>::Ptr DIS_cummulative,Kappacum;
	//Nearest Neighbor field holders
	pcl::PointCloud<NNFX<5>>::Ptr nnf5, resultnnf5;
	pcl::PointCloud<NNFX<1>>::Ptr nnf1, resultnnf1;
	NNFX<5> resultinds;
	//correspondences for registration/visualization
	pcl::CorrespondencesPtr Corrs,filtered_closest;

	//kdtrees for eucludean distances
	pcl::KdTreeFLANN<XYZ> kdtree,QGkdtree,TGkdtree;

	//Nearest neighbor fields
	XYZLCloud::Ptr nnf, resultnnf,DDIS_refined_NNF, Reg_points;

	//statistics clouds
	XYZICloud::Ptr similarityCloud, resultlabel,kappac,distc, distcfull;

	//all the matching parameters are stored here
	template_match_parms_s P;
	float grid_max_resolution;
	uint32_t res_int;
	pcl::PointIndices::Ptr result_indices;

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

	//Kappa holds how many points in Q match the same point in T
	vector<uint32_t> Kappa_res;
	vector<uint32_t> closest_matches;
	vector<float> min_distances_res;

	//General Keypoint calculation method
	void ComputeKeyPoints(nnfDest d);
	void meshstats(nnfDest d);
	//Load a grid mesh from the file in path 
	void LoadGridMesh(string path, nnfDest d);

	void LoadSurface(string path, nnfDest d);

	//Surface reconstruction method 
	void Reconstruct_Surface(nnfDest d);
	void Template_refinement();
	//sets the matching parameters
	void setparams(template_match_parms_s p) { P = p; }

	//returns the matching parameters
	template_match_parms_s gettparams() { return P; };

	//reads a cloud from path according to dest
	bool readCloud(string path, nnfDest d);

	//normal computation - force indicates if we run over loaded data
	void computeNormal(nnfDest d,bool force);

	//Computes the nearest neighbor fields
	virtual void computeNNF() {};

	//Computes the features
	virtual void computeF(nnfDest d) {};

	void save_distance_matrix(string path, nnfDest d);
	//Loads template center from file
	void LoadTFocal(string path);

	//Saves template center to file
	void saveTFocal(string path);

	//Finds the focal- minimizer of the maximal distance on it's distance tree
	void calcTFocal();

	//Loads nearest neighbor fields from set file
	void loadNNFField();

	//Loads center from file
	void LoadFocal(string file);
	
	vector<int> extract_band_indices(float distance,float Ewidth);
	//calculates feature stats
	virtual void feature_moments() = 0;

	//demean feature stats
	virtual void demean_features(nnfDest d) = 0;

	//extract result statistics anf clouds - NNF of result, colors scene 
	virtual void extractResult(bool res = true)=0;

	//Main template matching function - calculates similarity score for the whole scene
	virtual XYZICloud::Ptr calculateSimilarity() { XYZICloud::Ptr result(new XYZICloud);  return result; };
	virtual vector<vert_dis_s> calculateSimilarityband(vector<int> candidates, int origin)=0;
	//A method to show Nearest neighbor matches and distance heat map of the matches between last extracted result and template
	void showNNF(uint32_t ID);

	/**\brief extracts a template candidate centered around keypoint i and extracts similarity input informaiton
	   \param i center point index
	   \param scene_indices a container which will be filled with the candidate indices
	   \param sceneDistances a container which will be filled with the candidate points' distance from the focal
	   \param extract a filter used to extract indices into a seperate cloud for load handling
	   \param full tells if to take only keypoints or full cloud points
	   */
	void extractTemplateCandidate(int i, pcl::PointIndices::Ptr scene_indices, std::vector<float>& sceneDistances,
		                          pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract, float R, bool full = false);
	//Extract window using the euclidean regime
	void extractTemplateCandidateEuc(int i, pcl::PointIndices::Ptr scene_indices, std::vector<float>& sceneDistances,
		pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract, float R, bool full = false);
	//Extract window using point lying near a surface - used in case we sampled the surface for grid but want 
	//the entire set of points for similarity purposes
	void extractTemplateCandidateGeoNN(int i, pcl::PointIndices::Ptr scene_indices, std::vector<float>& sceneDistances,
		pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract, float R, bool full = false);
	//Extract the window using a surface mesh
	void extractTemplateCandidateGeo(int i, pcl::PointIndices::Ptr scene_indices, std::vector<float>& sceneDistances, float R, bool full = false);
	
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
	void extractCandidatestats(pcl::PointIndices::Ptr scene_indices, uint32_t M, vector<float>& sDistances, vector<float>& tDistances, vector<float>& mindistances,
		pcl::PointCloud<HistogramInt<5>>::Ptr Kappac,std::vector<uint32_t>& Kappa_j, std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j,vector<uint32_t>& closest, HistogramInt<5>& DIS);
	
	//returns the distance of every point on the template
	vector<float> getTemplateDistances(patchMode Mode,int i);
	//Uses euclidean regime
	vector<float> Template_Euclid_Distance(int i);
	//Uses surface for distances
	vector<float> Template_Geo_Distance(int i);
	//Projects points to their closest point on the surface
	vector<float> Template_Geo_NNDistance(int i);
	//calculates a geodesic distance matrix between the mesh vertices
	void createresultDmatrix();
	//calculates a patch similarity score to the template
	virtual float Similarity(int M, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
		std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs) {
		return 0;
	};
	//creates a list of the indexes of the boundaries on the mesh
	void extractTemplateBoundaryInds();
	void createRDistanceTree();
	void LoadMesh(nnfDest d) {
		switch (d) {
		case T: pcl::io::loadPLYFile(P.T.in, *Tmesh); break;
		case Q: pcl::io::loadPLYFile(P.S.in, *Qmesh); break;
		}
	};

	float DDISfunc(int M, std::vector<uint32_t>& Kappa_j, std::vector<float>& r_j);

	float WDISfunc(int M, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac, std::vector<float>& mindistances);
	virtual void registrationmds() = 0;;
	void icp_mds();
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
		case SIMILARITYCOLOR: {
							  XYZRGBCloud::Ptr sim_color = visualizeCloud<XYZI>(similarityCloud);
							  savePointCloudToFile(sim_color, path); break; }
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

		case RMESH: pcl::io::savePLYFile(path, *Rmesh);
		case TSURF:break;
		case QSURF:break;
		};
	};

	cloud_statistics_s CalculateStatistics(nnfDest d, string out);

	virtual ~FeatureCloud() = default;
	virtual pcl::CorrespondencesPtr matchKeyPointsFeaturesmds(float threshold)=0;
	//preform registration to the result
	virtual void registration() = 0;

	void icp();
	virtual void extractTemplate(int ind) =0;
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
	vector<int> FeatureCloud::Refined_GT_map(vector<barycentric_polygon_s> gt_map);

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
	//Calculate similarity score for a single search window centered around point i in the scene
	virtual float calculateSimilarity(uint32_t i) = 0;

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
	float calculateSimilarity(uint32_t i);
	vector<vert_dis_s> calculateSimilarityband(vector<int> candidates,int origin);
	float Similarity(int M, pcl::PointIndices::Ptr scene_indices, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
		std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs);
	XYZICloud::Ptr calculateSimilarity();
	float BBSfunc(int M, pcl::PointIndices::Ptr scene_indices, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac, std::vector<uint32_t>& Patch_Buddies);
	float WBBSfunc(int M, pcl::PointIndices::Ptr scene_indices, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac, std::vector<uint32_t>& Patch_Buddies, vector<float>& mindistances);

	void saveCloud(string path, nnfDest d);
	pcl::CorrespondencesPtr matchKeyPointsFeatures(float threshold=1);
	void extractTemplate(int ind);
	void extractResult(bool res = true);
	void registration();
	void registrationmds();
	pcl::CorrespondencesPtr matchKeyPointsFeaturesmds(float threshold=1);
	~Feature1() {};
};

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


template<class CloudT, class D> float Feature1<CloudT, D>::calculateSimilarity(uint32_t i) {
	cout << "1";
	DIS_cummulative->resize(Qkeypoints->size());
	Kappacum->resize(Tkeypoints->size());
	vector<float> TDistances = getTemplateDistances(P.pMode,T_f.ind); QGkdtree.setInputCloud(QGridCloud);
	kdtree.setInputCloud(Qkeypoints);
	cout << "2";
	float max = 0;
	copyPointCloud(*Qkeypoints, *similarityCloud);
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	cout << "3";
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	vector<uint32_t> M(similarityCloud->size());
	cout << "4";
	vector<uint32_t> closest_temp(Tkeypoints->size(), MAXUINT32);
	SQuasiGeoDistances.clear();
	pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
	float TempSimilarity = 0;
	vector<float> sceneDistances;
	extractTemplateCandidate(i, scene_indices, sceneDistances, extract, T_f.RG);
	M[i] = scene_indices->indices.size();
	std::vector<float> r_j(M[i]);
	std::vector<uint32_t> Kappa(Tkeypoints->size());
	std::vector<uint32_t> Kappa_j(M[i]);
	std::vector<uint32_t> Patch_Buddies(M[i]);
	vector<float> mindistances(Tkeypoints->size(), INFINITY);
	extractCandidatestats(scene_indices, M[i], sceneDistances, TDistances, mindistances, Kappacum, Kappa_j, Patch_Buddies, r_j, closest_temp,DIS_cummulative->at(i));
	max = Similarity(M[i], scene_indices, Kappacum, Kappa_j, mindistances, Patch_Buddies, r_j, DIS_cummulative->at(i).counter[0]);
	cout << "9";
	//similarityCloud->at(i).intensity = similarityCloud->at(i).intensity;
	
	copyPointCloud(*Tkeypoints, *DDIS_refined_NNF);	copyPointCloud(*Tkeypoints, *distc); copyPointCloud(*Tkeypoints, *kappac);
	for (int j = 0; j < Tkeypoints->size(); j++) {
		DDIS_refined_NNF->at(j).label = closest_matches.at(j);
		distc->at(j).intensity = min_distances_res.at(j) * 255/T_f.path.chain.size();
	}
	return max;
}

template<class CloudT, class D> vector<vert_dis_s> Feature1<CloudT, D>::calculateSimilarityband(vector<int> candidates, int origin) {
	cout << "1";
	vector<vert_dis_s> result;
	float R = T_G.getRadius(origin);
	DIS_cummulative->resize(resultkeypoints->size());
	vector<float> TDistances = getTemplateDistances(P.pMode, origin); QGkdtree.setInputCloud(QGridCloud);
	kdtree.setInputCloud(Qkeypoints);
	cout << "2";
	float max = 0;
	DISQ resultsQueue;
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	cout << "3";
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	vector<uint32_t> M(candidates.size());
	cout << "4";
	for (int i = 0; i < candidates.size(); i++) {//calculating similarity for each search window
		pcl::PointCloud<HistogramInt<5>>::Ptr Kappa = pcl::PointCloud<HistogramInt<5>>::Ptr(new pcl::PointCloud<HistogramInt<5>>());
		vert_dis_s v;
		v.ind = candidates.at(i);
		Kappa->resize(Tkeypoints->size());
		vector<uint32_t> closest_temp(Tkeypoints->size(), MAXUINT32);
		SQuasiGeoDistances.clear();
		pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
		vector<float> sceneDistances;
		extractTemplateCandidate(v.ind, scene_indices, sceneDistances, extract,R);
		M[i] = scene_indices->indices.size();
		std::vector<float> r_j(M[i]);
		std::vector<uint32_t> Kappa_j(M[i]);
		std::vector<uint32_t> Patch_Buddies(M[i]);
		vector<float> mindistances(Tkeypoints->size(), INFINITY);
		extractCandidatestats(scene_indices, M[i], sceneDistances, TDistances, mindistances, Kappa, Kappa_j, Patch_Buddies, r_j, closest_temp, DIS_cummulative->at(v.ind));
		v.dis = Similarity(M[i], scene_indices, Kappa, Kappa_j, mindistances, Patch_Buddies, r_j, DIS_cummulative->at(v.ind).counter[0]);
		resultsQueue.push(v);
	}
	cout << "9";
	for (int i = 0; i < 5; i++) {
		result.push_back(resultsQueue.top());
		resultsQueue.pop();
	}

	return result;
}

template<class CloudT, class D> XYZICloud::Ptr Feature1<CloudT,D>::calculateSimilarity() {
	cout << "1";
	DIS_cummulative->resize(Qkeypoints->size());
	vector<float> TDistances = getTemplateDistances(P.pMode,T_f.ind);
	QGkdtree.setInputCloud(QGridCloud);
	kdtree.setInputCloud(Qkeypoints);
	cout << "2";
	float max = 0;
	copyPointCloud(*Qkeypoints, *similarityCloud);
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	cout << "3";
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	vector<uint32_t> M(similarityCloud->size());
	cout << "4";
	for (int i = 0; i < similarityCloud->size(); i++) {//calculating similarity for each search window
		pcl::PointCloud<HistogramInt<5>>::Ptr Kappa= pcl::PointCloud<HistogramInt<5>>::Ptr(new pcl::PointCloud<HistogramInt<5>>());
		Kappa->resize(Tkeypoints->size());
		vector<uint32_t> closest_temp(Tkeypoints->size(), MAXUINT32);
		SQuasiGeoDistances.clear();
		pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
		vector<float> sceneDistances;
		extractTemplateCandidate(i, scene_indices, sceneDistances, extract, T_f.RG);
		M[i] = scene_indices->indices.size();
		std::vector<float> r_j(M[i]);
		std::vector<uint32_t> Kappa_j(M[i]);
		std::vector<uint32_t> Patch_Buddies(M[i]);
		vector<float> mindistances(Tkeypoints->size(), INFINITY);
		extractCandidatestats(scene_indices, M[i], sceneDistances, TDistances, mindistances, Kappa, Kappa_j,  Patch_Buddies, r_j, closest_temp,DIS_cummulative->at(i));
		similarityCloud->at(i).intensity = Similarity(M[i], scene_indices, Kappa, Kappa_j, mindistances,Patch_Buddies, r_j, DIS_cummulative->at(i).counter[0]);
		if (similarityCloud->at(i).intensity > max) {
			max = similarityCloud->at(i).intensity; closest_matches = closest_temp; pcl::copyPointCloud(*Kappa,*Kappacum); min_distances_res = mindistances;
		};
	
	}
	cout << "9";
	for (int i = 0; i < similarityCloud->size(); i++) {
		similarityCloud->at(i).intensity = 255* similarityCloud->at(i).intensity / max;
	}
	copyPointCloud(*Tkeypoints, *DDIS_refined_NNF);	copyPointCloud(*Tkeypoints, *distc); copyPointCloud(*Tkeypoints, *kappac);
	for (int i = 0; i < Tkeypoints->size(); i++) {
		DDIS_refined_NNF->at(i).label = closest_matches.at(i);
		distc->at(i).intensity = min_distances_res.at(i);
		if (min_distances_res.at(i)<P.T.stat.MeanResolution){
			Reg_points->push_back(DDIS_refined_NNF->at(i));
		}
	}
	return similarityCloud;
}

template<class CloudT, class D> float Feature1<CloudT,D>::Similarity(int M, pcl::PointIndices::Ptr scene_indices, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac, std::vector<uint32_t>& Kappa_j, vector<float>& mindistances,
	std::vector<uint32_t>& Patch_Buddies, std::vector<float>& r_j, int& DISs) {
	switch (P.similarity) {
	case WDIS: return WDISfunc(M,Kappac,mindistances);
	case DIS: return DISs;
	case DDIS: return DDISfunc(M, Kappa_j, r_j);
	case BBS: return BBSfunc(M, scene_indices,Kappac, Patch_Buddies);
	case WBBS: return WBBSfunc(M, scene_indices, Kappac, Patch_Buddies, mindistances);

	default: return 0;
	}
}

template<class CloudT, class D> float Feature1<CloudT,D>::BBSfunc(int M, pcl::PointIndices::Ptr scene_indices, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac,std::vector<uint32_t>& Patch_Buddies) {
	float result = 0; std::vector<int> neigh_indices; std::vector<float> neigh_sqr_dists;
	PointCloud<CloudT>::Ptr patchCloud(new PointCloud<CloudT>);
	cout << "1";
	for (size_t j = 0; j < M; ++j) {
		patchCloud->push_back(QF.at(scene_indices->indices[j]));
	}
	pcl::KdTreeFLANN<CloudT, D> patchKDTree;
	patchKDTree.setInputCloud(patchCloud);
	for (size_t j = 0; j < TF.size(); ++j) {
		if (Kappac->at(j).counter[0] != 0) {
			int found_neighs = patchKDTree.nearestKSearch(TF.at(j), 1, neigh_indices, neigh_sqr_dists);
			if (Patch_Buddies[neigh_indices[0]] == j) { result++; }
		}
	}
	return result;
}

template<class CloudT, class D> float Feature1<CloudT, D>::WBBSfunc(int M, pcl::PointIndices::Ptr scene_indices, pcl::PointCloud<HistogramInt<5>>::Ptr Kappac, std::vector<uint32_t>& Patch_Buddies, vector<float>& mindistances) {
	float result = 0; std::vector<int> neigh_indices; std::vector<float> neigh_sqr_dists;
	PointCloud<CloudT>::Ptr patchCloud(new PointCloud<CloudT>);
	for (size_t j = 0; j < M; ++j) {
		patchCloud->push_back(QF.at(scene_indices->indices[j]));
	}
	pcl::KdTreeFLANN<CloudT, D> patchKDTree;
	patchKDTree.setInputCloud(patchCloud);
	for (size_t j = 0; j < TF.size(); ++j) {
		if (Kappac->at(j).counter[0] != 0) {
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
	} else if (P.nnfType == NNF1) {
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
	//float thresholdPrecentage = 1;
	//filterMatchesByFeaturesDist(QF, RF, rawCorrespondences, Correspondences, thresholdPrecentage);*/
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
	//float thresholdPrecentage = 1;
	//filterMatchesByFeaturesDist(QF, RF, rawCorrespondences, Correspondences, thresholdPrecentage);*/
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
	//pcl::CorrespondenceEstimation<CloudT, CloudT> estimator; 
	pcl::CorrespondencesPtr correspondences(new Correspondences());
	bool only_closest = true;
	for (int i = 0; i < resultnnf5->size(); i++) {
		pcl::Correspondence corr1;
		corr1.index_query = i;
		corr1.index_match = resultnnf5->at(i).labels[0];
		corr1.distance = distcfull->at(i).intensity;
		//if (!(only_closest && (closest_matches.at(corr1.index_match) == MAXUINT32) && (corr1.distance<threshold))&& (resultnnf5->at(i).labels[0] != MAXUINT32)){
		if (!(only_closest && (corr1.distance>threshold)) && (resultnnf5->at(i).labels[0] != MAXUINT32)) {
			correspondences->push_back(corr1);
		}
	}

	return correspondences;
};

template<class CloudT, class D> pcl::CorrespondencesPtr Feature1<CloudT, D>::matchKeyPointsFeaturesmds(float threshold) {
	//pcl::CorrespondenceEstimation<CloudT, CloudT> estimator; 
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

template<class CloudT, class D> void Feature1<CloudT, D>::extractTemplate(int ind) {
	pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
	vector<float> sceneDistances;
	extractTemplateCandidate(ind, scene_indices, sceneDistances, extract, T_f.RG);
	copyPointCloud(*Qc, *resultlabel);
	vector<float> Tdist = getTemplateDistances(P.pMode,T_f.ind);
	resultnnf->clear(); resultnnf1->clear(); resultnnf5->clear(); resultkeypoints->clear(); result->clear(); 
	for (int i = 0; i < scene_indices->indices.size(); i++) {
		PointXYZI point;
		point.x = Qkeypoints->at(scene_indices->indices.at(i)).x;
		point.y = Qkeypoints->at(scene_indices->indices.at(i)).y;
		point.z = Qkeypoints->at(scene_indices->indices.at(i)).z;
		uint32_t label;
		switch (P.nnfType) {
		case NNF:resultnnf->push_back(nnf->at(scene_indices->indices.at(i))); label = nnf->at(scene_indices->indices.at(i)).label;  break;
		case NNF1:resultnnf1->push_back(nnf1->at(scene_indices->indices.at(i))); label = nnf1->at(scene_indices->indices.at(i)).labels[0];  break;
		case NNF5:resultnnf5->push_back(nnf5->at(scene_indices->indices.at(i))); label = nnf5->at(scene_indices->indices.at(i)).labels[0]; break;
		}
		point.intensity = MAXUINT32;
		if (label < MAXUINT32) {
			point.intensity = abs(sceneDistances.at(i) - Tdist.at(label)) / (P.T.stat.MeanResolution);
		}
		distcfull->push_back(point);
		resultkeypoints->push_back(Qkeypoints->at(scene_indices->indices.at(i)));
		RF.push_back(QF.at(scene_indices->indices.at(i)));
	}
	extractTemplateCandidate(ind, scene_indices, sceneDistances, extract, T_f.RG, true);
	result_indices = scene_indices;
	for (int i = 0; i < scene_indices->indices.size(); i++) {
		result->push_back(Qc->at(scene_indices->indices.at(i)));
		resultlabel->at(scene_indices->indices.at(i)).intensity = 160;
	}
	Rmesh = cut_mesh_from_points(Qmesh,scene_indices);

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
	extractTemplate(resultinds.labels[0]);
	/*pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract(new ExtractIndices<pcl::PointXYZ>);
	cout << "3";
	extract->setNegative(false);
	extract->setInputCloud(Qkeypoints);
	pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
	vector<float> sceneDistances;
	extractTemplateCandidate(ind, scene_indices, sceneDistances, extract);
	cout << "4";
	copyPointCloud(*Qc, *resultlabel);
	vector<float> Tdist = getTemplateDistances(P.pMode);
	for (int i = 0; i < scene_indices->indices.size(); i++) {
		PointXYZI point; 
		point.x= Qkeypoints->at(scene_indices->indices.at(i)).x;
		point.y= Qkeypoints->at(scene_indices->indices.at(i)).y;
		point.z= Qkeypoints->at(scene_indices->indices.at(i)).z;
		uint32_t label;
		switch (P.nnfType) {
		case NNF:resultnnf->push_back(nnf->at(scene_indices->indices.at(i))); label = nnf->at(scene_indices->indices.at(i)).label;  break;
		case NNF1:resultnnf1->push_back(nnf1->at(scene_indices->indices.at(i))); label = nnf1->at(scene_indices->indices.at(i)).labels[0];  break;
		case NNF5:resultnnf5->push_back(nnf5->at(scene_indices->indices.at(i))); label = nnf5->at(scene_indices->indices.at(i)).labels[0]; break;
		}
		cout << "5";
		point.intensity = INFINITY;
		if (label < MAXUINT32) {
			point.intensity = abs(sceneDistances.at(i) - Tdist.at(label)) / (P.T.stat.MeanResolution);
		}
		distcfull->push_back(point);
		resultkeypoints->push_back(Qkeypoints->at(scene_indices->indices.at(i)));
		if (res) RF.push_back(QF.at(scene_indices->indices.at(i)));
	}
	extractTemplateCandidate(ind, scene_indices, sceneDistances, extract, true);
	cout << "6";
	for (int i = 0; i < scene_indices->indices.size(); i++) {
		result->push_back(Qc->at(scene_indices->indices.at(i)));
		resultlabel->at(scene_indices->indices.at(i)).intensity = 160;
	}*/
	
};
#endif