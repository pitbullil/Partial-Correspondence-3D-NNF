#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <omp.h>
#include <direct.h>
#include <cloud_io_visl.hpp>
#include <cloud_analysis_visl.hpp>
#include <cloud_key_points_visl.hpp>
#include <cloud_features_visl.hpp>
#include <cloud_registration_visl.hpp>
#include <cloud_visualization_visl.hpp>
#include <cloud_registeration.hpp>
#include <Quicksort.hpp>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/geometry.h>
#include <meshgraph.h>
#include <cloud_nnf_multi_visl_omp.hpp>
#include <misc_utils.hpp>



POINT_CLOUD_REGISTER_POINT_STRUCT(Histogram<153>,
(float[153], histogram, histogram)
);

using namespace std;
using namespace pcl;

/***************************  Main File ***********************************/

int main(int argc, char** argv)
{
	time_t base_time;
	time(&base_time);
	int x = omp_get_num_threads();
	printf_s("The number of threads is %d\n", omp_get_max_threads());
	bool normals = false;
	vector<TimeLog*> logs;
	time_t timer;
	//const float PI(3.14159265359);
	bool Autodefault = true;
	/***************************  Parsing parameters from command line: ***********************************/

	if (argc < 7) {
		(cerr << " Please Enter: <template_cloud 'T.ply'> <query cloud 'Q.ply'> <surface=Surface_Reconstruction method:POISSON/NONE/TRIANGLE> <keypoint=keypoints method:ISS/ALL/GRID/DUALGRID/HARRIS/SIFT> \n <feature=feature method: FPFH/PFH/SHOT/ROPS> <template=template method: GEODESIC/GEONN/SPHERE> <similarity=similarity method:DIS/DDIS/BBS/DISN> <mesh=DUAL/SUPER/NORMAL> <RQ=RadiusQ> <RT=RadiusT> <LOAD_DT> <LOAD_DQ>" << endl);
		return -1;
	}
	float feature_o_radius = dis3::RADIUS_SEARCH_MULTIPLIER;
	if (argc < 9) {
		cout << "no radiuses recieved ,asking for radiuses manually\n"; Autodefault = false;
	}
	else { feature_o_radius = atof(argv[8]); }
	float feature_t_radius = feature_o_radius;
	if (argc > 10) { feature_t_radius = atof(argv[9]); }

	template_match_parms_s p;
	parse_3dis_params(argv, argc, p);
	create_output_directories(p);
	cout << endl << "The template file is: " << p.T.in << endl;
	cout << endl << "The scene file is: " << p.S.in << endl;
	/***************************  Initializing Template Matcher: ***********************************/
	FeatureCloud *c;
	if (p.feature.compare(0, 4, "FPFH") == 0) { c = new FPFHFeature(); normals = true; }
	else if (p.feature.compare(0, 4, "SHOT") == 0) { c = new SHOTFeature(); }
	else if (p.feature.compare(0, 4, "ROPS") == 0) {
		c = new ROPSFeature(); if (p.surface.compare(0, 4, "NONE") == 0) { p.surface = "POISSON"; }
	}
	else if (p.feature.compare(0, 3, "PFH") == 0) { c = new PFHFeature(); normals = true;}
	else if (p.feature.compare(0, 3, "HKS") == 0) { c = new HKSFeature(); }
	else if (p.feature.compare(0, 5, "SIHKS") == 0) { c = new SIHKSFeature();}
	else if (p.feature.compare(0, 5, "SCHKS") == 0) { c = new SCHKSFeature();}
	else if (p.feature.compare(0, 6, "LBEIGS") == 0) { c = new LBEIGSFeature(); }
	else if (p.feature.compare(0, 5, "SFPFH") == 0) { c = new SFPFHFeature(); normals = true;}
	else if (p.feature.compare(0, 4, "GPFH") == 0) { c = new GPFHFeature(); normals = true;}
	else if (p.feature.compare(0, 5, "GFPFH") == 0) { c = new GFPFHFeature(); normals = true;}

	//p.nnf_reject_threshold = 1.5;

	c->setparams(p);

	/***************************  Read base clouds: ***********************************/
	printSection("Reading files from drive");
	c->readCloud(p.S.in, Q); cloud_statistics_s Q_s = c->CalculateStatistics(Q, p.S.path + "\\res.txt");
	c->readCloud(p.T.in, T); cloud_statistics_s T_s = c->CalculateStatistics(T, p.T.path + "\\res.txt");
	p = c->gettparams();
	time(&timer);
	TimeLog surface_time("Surface");
	TimeLog feature_time(p.feature);
	TimeLog nnf_time("nnf");
	TimeLog similarity_time("Similarity");
	TimeLog keypoints_time("Keypoints");
	TimeLog greedy_opt_time("Greedy Opt");

	surface_time.Reset();
	string out_dir;
	string test_string = c->P.T.path;

	if (c->P.T.path.compare(0, 4, "cuts") == 0 || c->P.T.path.compare(0, 5, "holes") == 0) {
		out_dir = ".\\" + c->P.sim + "_" + ftos(c->P.r_thresh, 1) + "_DF_" + ftos(c->P.div_frac, 1) + "_NNR_" + ftos(c->P.nnf_reject_threshold, 2) + "_FR_" + ftos(c->P.frac, 2);
	}
	else {
		test_string = c->P.T.path + "_" + c->P.S.path;
		out_dir = ".\\" + c->P.sim + "_" + ftos(c->P.r_thresh, 1) + "_DF_" + ftos(c->P.div_frac, 1) + "_NNR_" + ftos(c->P.nnf_reject_threshold, 2) + "_FR_" + ftos(c->P.frac, 2);

	}


	
	string out_dir_time = out_dir + "\\time\\";
	string raw_corrs = out_dir + "\\raw_sparse\\";
	string refined_corrs = out_dir + "\\refined_sparse\\";

	mkdir(out_dir.c_str());
	mkdir(raw_corrs.c_str());
	mkdir(refined_corrs.c_str());
	mkdir(out_dir_time.c_str());

	/***************************  Surface Reconstruction: ***********************************/
	p.S.s.cloudResolution = Q_s.MaxResolution; p.T.s.cloudResolution = T_s.MaxResolution;
	cout << c->gettparams().geotype << endl;
	c->Reconstruct_Surface(Q);c->Reconstruct_Surface(T);
	printSection("grid calculation");
	vector<barycentric_polygon_s> barmap = vertex_to_bar(c->Qmesh);
	cout <<"init polygons: "<< c->Qmesh->polygons.size() << endl;

	/***************************  Grid for Geodesics: ***********************************/
	p.T.s.base = false; p.T.s.save = false; p.T.s.depth = 7; p.T.s.grid = true; p.T.s.cloudResolution = T_s.MaxResolution;
	p.S.s = p.T.s;
	p.S.s.cloudResolution = Q_s.MaxResolution;
	c->setparams(p); c->Reconstruct_Surface(Q);c->Reconstruct_Surface(T);
	cout << "2nd polygons: " << c->Qmesh->polygons.size() << endl;
	p = c->gettparams();
	surface_time.SetTime(); logs.push_back(&surface_time);
	c->meshstats(Q); c->meshstats(T);
	cout << "polygons: " << c->Qmesh->polygons.size() << endl;
	c->find_sample_points();

	/*************************** Keypoint Calculations: ***********************************/
	keypoints_time.Reset();
	printSection("Keypoint Calculation");
	c->ComputeKeyPoints(Q); c->ComputeKeyPoints(T);
	c->saveCloud(p.S.k.path, QKEY);
	c->saveCloud(p.T.k.path, TKEY);
	keypoints_time.SetTime(); logs.push_back(&keypoints_time);

	/*************************** Feature Calculations: ***********************************/
	printSection("Feature Calculation");
	feature_time.Reset();

	c->computeNormal(Q, normals);
	c->computeF(Q);
	c->saveCloud(p.S.f.path, QFEATURE);
	c->computeNormal(T, normals); c->computeF(T);
	c->saveCloud(p.T.f.path, TFEATURE);
	feature_time.SetTime(); logs.push_back(&feature_time);

	/*************************** Nearest Neighbor Field Calculations: ***********************************/
	nnf_time.Reset();
	printSection("NNF Calculation");
	c->computeNNF();
	c->saveCloud(p.nnf_dir + "nnf.pcd", NNF);
	nnf_time.SetTime(); logs.push_back(&nnf_time);

	/*************************** Similarity Calculations: ***********************************/
	similarity_time.Reset();
	printSection("inds loading");

	printSection("Similarity Calculation");

	//Raw similarity calculation
	vector<vector<ind_dis_s>> results = c->calculateSimilarity();
	similarity_time.SetTime(); logs.push_back(&similarity_time);

	vector<barycentric_polygon_s> corrs = corrs_to_bar_corrs(results, barmap);

	save_bar_corrs(corrs, raw_corrs + "\\" + test_string +".corr");
	save_feature_corrs_to_csv(results, raw_corrs + "\\"+ test_string +".csv",c->feature_point_inds);
	if (c->P.save_similarity_clouds) c->saveCloud(p.result_dir, SIMILARITYCOLOR);

	if (c->P.save_similarity_clouds) c->save_similarity(raw_corrs + "\\" + test_string);
	if (c->P.save_feature_patches) c->extract_all_feature_patches(raw_corrs);

	//Refinement stage
	greedy_opt_time.Reset();
	cout << "refinement\n"; bool change = false;
	change = false;
	c->distance_based_outlier_detector(1.15, 0.01);
	results = c->outlier_refinement(0.01,change);
	change = false;
	c->gopt_use_m = true;
	c->distance_based_outlier_detector(1.15, 0.01);
	results=c->outlier_refinement(0.01, change);
	c->gopt_use_h = true;
	c->distance_based_outlier_detector(1.15, 0.01);
	results = c->outlier_refinement(0.01, change);
	corrs = corrs_to_bar_corrs(results, barmap);

	save_bar_corrs(corrs, refined_corrs + "\\" + test_string + ".corr");
	save_feature_corrs_to_csv(results, refined_corrs + "\\" + test_string + ".csv", c->feature_point_inds);

	greedy_opt_time.SetTime();
	logs.push_back(&greedy_opt_time);
	Time_LogtoCSV(logs, out_dir_time + "\\" + test_string + ".csv");

}
