#pragma once
#ifndef PCL_FASTMARCH_H
#define PCL_FASTMARCH_H

#include <omp.h>
#include <proj_namespaces.hpp>
#include <math.h>
#include <pcl/search/kdtree.h>
#include <string.h>
#include <tuple>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/pca.h>
#include <QuickSort.hpp>
#include <fastmarch.h>

using namespace dis3;

class FastMeshOMP {
private:
	vector<float> X, Y, Z;
	vector<int> T;
	vector<vector<uint32_t>> indices_t;
	vector<vector<float>> SourceVal_t;
	vector<vector<float>> Distances_t;
	vector<FMM*> fmm_t;
	int VNum;
	int TNum;
	uint8_t threads;
public:
	FastMeshOMP() {};
	FastMeshOMP(pcl::PolygonMesh::Ptr mesh, uint8_t threads):threads(threads) {
		XYZCloud::Ptr cloud(new XYZCloud());
		fromPCLPointCloud2(mesh->cloud, *cloud);
		VNum = cloud->size();
		TNum = mesh->polygons.size();
		X.resize(VNum); Y.resize(VNum); Z.resize(VNum);
		for (int i = 0; i < VNum; i++) {
			X.at(i) = cloud->at(i).x;
			Y.at(i) = cloud->at(i).y;
			Z.at(i) = cloud->at(i).z;
		}
		T.resize(TNum * 3);
		for (int i = 0; i < TNum; i++) {
			T.at(i) = mesh->polygons.at(i).vertices.at(0);
			T.at(i + TNum) = mesh->polygons.at(i).vertices.at(1);
			T.at(i + 2 * TNum) = mesh->polygons.at(i).vertices.at(2);
		}
		indices_t.resize(threads);
		Distances_t.resize(threads);
		SourceVal_t.resize(threads); 
		for (int t = 0; t < threads; t++) {
			indices_t.at(t).assign(VNum, MAXUINT32);
			Distances_t.at(t).assign(VNum, INFINITY);
			SourceVal_t.at(t).assign(VNum, INFINITY);
			fmm_t.push_back(new FMM(X.data(), Y.data(), Z.data(), T.data(), VNum, TNum));
		}
	};

	vector<vector<float>> createdistancematrixOMP(int threads);
	void GeodesicDisc(uint32_t src, vector<ind_r_s>& disc, uint32_t& M, float R_Disc=INFINITY, uint8_t thread=0);
};

float* createdistancematrix(pcl::PolygonMesh::Ptr mesh);

vector<vector<float>> createdistancematrixOMP(pcl::PolygonMesh::Ptr mesh, int threads); 
#endif