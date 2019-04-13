#pragma once
#ifndef CLOUD_IO_H
#define CLOUD_IO_H

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/pcl_exports.h>
#include <misc_utils.hpp>
#include <proj_namespaces.hpp>

using namespace std;
using namespace dis3;

void printSection(string outputString, string variableString="");
float load_score(string filename);
vector<int> load_gt_map(string filename);

vector<barycentric_polygon_s> load_gt_map_bar(string filename);

template<typename PointT>
int readCloudFile(string filename, pcl::PointCloud<PointT>& pcloud) {
	int sf = filename.size();
	cout << "reading cloud from:" << filename << endl;

	string endian = filename.substr(sf - 3, 3);
	if (endian == "ply") {
		return pcl::io::loadPLYFile<PointT>(filename, pcloud);
	}
	else if (endian == "pcd") {
		return pcl::io::loadPCDFile<PointT>(filename, pcloud);
	}
	else {
		PCL_ERROR("Couldn't read query Cloud");
		return -1;
	}
}

template<typename T>
bool readvector(string filename, vector<vector<T>>& vec) {
	ifstream vec_file(filename);
	int i = 0;
	int j = 0;
	string line;
	if (vec_file.is_open()) {
		while (getline(vec_file, line)) {
			std::istringstream stm(line);
			T value;
			while (stm >> value) vec[i].push_back(value);
			i++;
		}
		vec_file.close();
		return true;
	}
	return false;
}


template<typename T>
void savevector(string filename, vector<vector<T>>& vec) {
	ofstream vec_file;
	vec_file.open(filename);

	for (int i = 0; i < vec.size(); i++) {
		for (int j = 0; j < vec[i].size(); j++) {
			vec_file << vec[i][j] << " ";
		}
		vec_file << endl;
	}
	vec_file.close();
}

template<class CloudTPointer>
bool savePointCloudToFile(CloudTPointer cloud, string filenameString, string descriptionString = "") {
	if (cloud->size() > 0) {
		string outstr = filenameString.substr(0, filenameString.size()) + descriptionString + ".ply";
		cout << "writing cloud to:" << outstr<< endl;
		io::savePLYFileASCII(outstr, *cloud);
		return true;
	}
	else {
		return false;
	}
}

int loadres(string filename);
void saveres(string filename, int ind);

bool savePolygonMeshToFile(pcl::PolygonMeshPtr mesh, string filenameString);

Eigen::Matrix4f readTransform(string file_name);
void saveTransform(string file_name, Eigen::Matrix4f& transform);

float getUserValue(string varName, float defaultValue, bool Autodefault = false);
int getUserValue(string varName, int defaultValue, bool Autodefault = false);
bool getUserValue(string varName, bool defaultValue, bool Autodefault = false);

void saveStats(string filename, float BBS, float Diversity);

#endif