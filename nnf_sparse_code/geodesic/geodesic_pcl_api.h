#ifndef GEODESIC_PCL_H
#define GEODESIC_PCL_H

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <geodesic_algorithm_exact.h>
#include <geodesic_algorithm_dijkstra.h>
#include <geodesic_algorithm_subdivision.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <proj_namespaces.hpp>
#include <string.h>

enum GeoAlg {MMP,SUBDIVISION,DIJK};

class Geo_Estimator {
public:
	GeoAlg Type;
	Geo_Estimator() {};
	bool initialize_mesh(pcl::PolygonMeshPtr pclmesh);
	virtual void init_alg() = 0;
	//virtual void propogate(unsigned int source) = 0;
	virtual void propogate(unsigned int source,double Rmax) = 0;
	virtual double distance(unsigned int index) = 0;
	virtual std::vector<double> distances() = 0;
	virtual std::vector<double> distances(unsigned int source, double Rmax) = 0;
	virtual pcl::PointCloud<pcl::PointXYZI>::Ptr distance_cloud(unsigned int index)=0;
	virtual pcl::PointCloud<pcl::PointXYZI>::Ptr distance_cloud()= 0;
	virtual pcl::PointCloud<pcl::PointXYZI>::Ptr get_path(unsigned int index) = 0;
	virtual unsigned int find_focal() = 0;
	virtual Eigen::MatrixXf CreateDistanceMatrix() = 0;
	double GetMaxDistance() { return max_distance; }
	unsigned int GetIndex() { return max_index; }
	unsigned int GetSource() { return source; }
	virtual void extract_geodesic_patch(unsigned int source, double MaxR, std::vector<dis3::ind_r_s>& patch) = 0;
protected:
	geodesic::Mesh mesh;
	double max_distance;
	unsigned int max_index;
	unsigned int source;
	std::vector<double> distance_field;
};

template<class Alg> class MMP_Estimator : public Geo_Estimator {
public:
	MMP_Estimator() {};
	void init_alg();
	~MMP_Estimator() { delete algorithm; }
	//void propogate(unsigned int source);
	void propogate(unsigned int source, double Rmax = geodesic::GEODESIC_INF);
	Eigen::MatrixXf CreateDistanceMatrix();
	double distance(unsigned int index);
	std::vector<double> distances();
	std::vector<double> distances(unsigned int source, double Rmax = geodesic::GEODESIC_INF);
	pcl::PointCloud<pcl::PointXYZI>::Ptr distance_cloud(unsigned int index);
	pcl::PointCloud<pcl::PointXYZI>::Ptr get_path(unsigned int index);
	pcl::PointCloud<pcl::PointXYZI>::Ptr distance_cloud();
	void extract_geodesic_patch(unsigned int source, double MaxR, std::vector<dis3::ind_r_s>& patch);
	unsigned int find_focal();
	Alg* algorithm;
};

class ExactGeodesicEstimator : public MMP_Estimator<geodesic::GeodesicAlgorithmExact> {};

class Geo_Subdiv_Estimator : public MMP_Estimator<geodesic::GeodesicAlgorithmSubdivision>{
private:
	unsigned int subdivision_level;
public:
	Geo_Subdiv_Estimator() {};
	Geo_Subdiv_Estimator(unsigned int sub) : subdivision_level(sub){};
	void init_alg();
};

template<class Alg> void MMP_Estimator<Alg>::init_alg() {
	algorithm = new Alg(&mesh);
}

template<class Alg> Eigen::MatrixXf MMP_Estimator<Alg>::CreateDistanceMatrix() {
	Eigen::MatrixXf D;
	D.resize(mesh.vertices().size(), mesh.vertices().size());
	for (int i = 0; i < mesh.vertices().size(); i++) {
		std::vector<double> Dtemp = distances(i);
		for (int j = 0; j < mesh.vertices().size(); j++) {
			D(i, j) = float(Dtemp.at(j));
		}
	}
	return D;
}

template<class Alg> unsigned int MMP_Estimator<Alg>::find_focal() {
	float Max_Radius = INFINITY;
	unsigned int result = 0;
	for (int source = 0; source < mesh.vertices().size(); source++) {
		distances(source);
		if (max_distance < Max_Radius) {
			Max_Radius = max_distance;
			result = source;
		}
	}
	distances(result);
	return result;
}

//template<class Alg> void MMP_Estimator<Alg>::propogate(unsigned int source) {
//	std::vector<geodesic::SurfacePoint> sources;
//	sources.push_back(geodesic::SurfacePoint(&mesh.vertices()[source]));
//	algorithm->propagate(sources);
//}

template<class Alg> void MMP_Estimator<Alg>::propogate(unsigned int source,double Rmax) {
	std::vector<geodesic::SurfacePoint> sources;
	sources.push_back(geodesic::SurfacePoint(&mesh.vertices()[source]));
	algorithm->propagate(sources, Rmax);
}

template<class Alg> double MMP_Estimator<Alg>::distance(unsigned int index) {
	double result;
	geodesic::SurfacePoint p(&mesh.vertices()[index]);
	unsigned int best_source = algorithm->best_source(p, result);		//for a given surface point, find closets source and distance to this source
	return result;
}

template<class Alg> std::vector<double> MMP_Estimator<Alg>::distances() {
	max_distance = 0;
	for (int i = 0; i < distance_field.size(); i++) {
		geodesic::SurfacePoint p(&mesh.vertices()[i]);
		unsigned best_source = algorithm->best_source(p, distance_field.at(i));		//for a given surface point, find closets source and distance to this source
		if (distance_field.at(i) > max_distance){max_distance = distance_field.at(i); max_index = i; }//update what is the farthest point
	}
	return distance_field;
}

template<class Alg> std::vector<double> MMP_Estimator<Alg>::distances(unsigned int source,double Rmax) {
	propogate(source,Rmax);
	return distances();
}

template<class Alg> pcl::PointCloud<pcl::PointXYZI>::Ptr MMP_Estimator<Alg>::distance_cloud() {
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
	result->resize(mesh.vertices().size());
	for (int i = 0; i <result->size(); i++) {
		pcl::PointXYZI& p = result->at(i);
		p.x = mesh.vertices().at(i).x(); p.y = mesh.vertices().at(i).y(); p.z = mesh.vertices().at(i).z();
		p.intensity = distance_field.at(i);
	}
	return result;
}

template<class Alg> pcl::PointCloud<pcl::PointXYZI>::Ptr MMP_Estimator<Alg>::distance_cloud(unsigned int source) {
	propogate(source);
	return distance_cloud();
}

template<class Alg> pcl::PointCloud<pcl::PointXYZI>::Ptr MMP_Estimator<Alg>::get_path(unsigned int index) {
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
	geodesic::SurfacePoint p(&mesh.vertices()[index]);
	std::vector<geodesic::SurfacePoint> path;
	algorithm->trace_back(p, path);
	result->resize(path.size());
	for (int i = 0; i < path.size(); i++) {
		pcl::PointXYZI& p = result->at(i);
		std::vector<geodesic::vertex_pointer> ptr;
		mesh.closest_vertices(&path.at(i), &ptr);
		unsigned id = ptr.at(0)->id();
		p.x = path.at(i).x(); p.y = path.at(i).y(); p.z = path.at(i).z();

		p.intensity = distance_field.at(id);
	}
	return result;
}

template<class Alg> void MMP_Estimator<Alg>::extract_geodesic_patch(unsigned int source, double MaxR, std::vector<dis3::ind_r_s>& patch) {
	propogate(source, MaxR);
	for (int i = 0; i < distance_field.size(); i++) {
		geodesic::SurfacePoint p(&mesh.vertices()[i]);
		unsigned best_source = algorithm->best_source(p, distance_field.at(i));		//for a given surface point, find closets source and distance to this source
		if (distance_field.at(i) < MaxR) {
			patch.push_back(ind_r_s(i, distance_field.at(i)));
		};
	};
};

void Geo_Subdiv_Estimator::init_alg() {
	algorithm = new geodesic::GeodesicAlgorithmSubdivision(&mesh, subdivision_level);
}

bool Geo_Estimator::initialize_mesh(pcl::PolygonMeshPtr pclmesh) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromPCLPointCloud2(pclmesh->cloud, *cloud);
	std::vector<double> points(3*cloud->size());
	std::vector<unsigned> faces(3*pclmesh->polygons.size());
	distance_field.resize(cloud->size());
	for (int i = 0; i < cloud->size(); i++) {
		pcl::PointXYZ p = cloud->at(i);
		points.at(3 * i) = p.x;
		points.at(3 * i+1) = p.y;
		points.at(3 * i+2) = p.z;
	}

	for (int i = 0; i < pclmesh->polygons.size(); i++) {
		pcl::Vertices V = pclmesh->polygons.at(i);
		faces.at(3 * i) = V.vertices.at(0);
		faces.at(3 * i + 1) = V.vertices.at(1);
		faces.at(3 * i + 2) = V.vertices.at(2);
	}

	mesh.initialize_mesh_data(points, faces);
	return true;
}

void save_distance_matrix(std::string path, Eigen::MatrixXf D) {
	std::fstream Dmatfile(path, std::fstream::out);
	for (int i = 0; i < D.rows(); i++) {
		Dmatfile << D(i, 0);
		for (int j = 1; j < D.cols(); j++) {
			Dmatfile << "," << D(i,j);
		}
		Dmatfile << std::endl;
	}
	Dmatfile.close();
}
#endif
