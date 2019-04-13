#ifndef MISC_UTILS_H
#define MISC_UTILS_H
#include <string>
#include <iostream>     // std::cout
#include <sstream>      // std::ostringstream
#include <iomanip>
#include <vector> 
#include <algorithm>
#include <list>
#include <numeric>
#include <random>
#include <time.h>
#include <math.h>
#include <direct.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <proj_namespaces.hpp>
#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <fstream>


using namespace std;
using namespace dis3;


/*void save_distance_matrix_a(std::string path, double* D, int sz) {
	std::fstream Dmatfile(path, std::fstream::out);
	for (int i = 0; i < sz; i++) {
		for (int j = 1; j < sz; j++) {
			Dmatfile << "," << D[i*sz+j];
		}
		Dmatfile << std::endl;
	}
	Dmatfile.close();
}*/

/** \brief Get the vertice of the polygon closest to the barycentric coordinate
  * \param bar the barycentric coordinates
  * \param poly the polygon vertices
  * \param fraction the approximate fraction of the original indices we want to get back
  */
int barycentric_closest(barycentric_polygon_s bar, pcl::Vertices poly);

float stringetof(string in);

int stringetoi(string in);

void parse_3dis_params(char** argv, int argc, template_match_parms_s& p);

void create_output_directories(template_match_parms_s& p);

//check if a string contains a number
bool check_num(std::string const &in);

//gets only the name of the file from a path
string extract_filename(string s);

/** \brief Converts a float to string
  * \param f input value
  * \param nd number of digits after the decimal point
  */
string ftos(float f, int nd);

//formats timer values into Days/Hours/Minutes/Seconds
string format_time_d_h_m_s(time_t elapsed);


class TimeLog {
private:
	time_t base;//holds the reference(initial) time
	time_t elapsed;//holds the last recorded time
	string label;//holds the name of the timed event
	bool logged;//indicated if the time had been logged
public:
	TimeLog(string label);//Constructor - sets the label
	string LogToCSV(); //calculates the elapsed time and outputs to a string
	string printlabel();//returns the event name string
	void SetTime();//set the elapsed time
	void Reset();//resets the logger and sets a new base time
	string format_time_d_h_m_s();//formats timer values into Days/Hours/Minutes/Seconds
};

/** \brief Writes a vector of time log events to a csv file
    \param logger a vector contaiming all the events to be logged
	\param out_path contains the file path to be written
	*/
void Time_LogtoCSV(std::vector<TimeLog*> logger, string out_path);

struct LabeledStat {
	float stat;
	string Label;
};

//read a time log csv file into a LabeledStat vec
std::vector<LabeledStat> read_time_log(string filename);


/** \brief Get a random sample of the indices - changes the input vectors
* \param indices the list of indices to be sampled
* \param distances a vector containing distances corresponding to the indices
* \param fraction the approximate fraction of the original indices we want to get back
*/
void rand_sample_patch(std::vector<int>& indices, vector<float>& distances, float fraction);

void parse_Rmode(string in, template_match_parms_s& p);
void print4x4Matrix(const Eigen::Matrix4f & matrix);

template<typename M>
M load_csv(const std::string & path) {
	std::ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<float> values;
	unsigned int rows = 0;
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ',')) {
			values.push_back(std::stod(cell));
		}
		++rows;
	}
	return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size() / rows);
}

template<typename M> void save_vector_to_csv(std::vector<std::vector<M>> corrs, std::string path) {
	std::fstream outfile(path, std::fstream::out);
	uint32_t rows = corrs.size(), cols = corrs.at(0).size();
	for (int i = 0; i < rows; i++) {
		outfile << corrs.at(i).at(0);
		for (int j = 1; j < cols; j++) {
			outfile << "," << corrs.at(i).at(j);
		}
		outfile << endl;
	}
	outfile.close();
}

template<typename M> void save_single_vector_to_csv(std::vector<M> corrs, std::string path) {
	std::fstream outfile(path, std::fstream::out);
	uint32_t rows = corrs.size();
	for (int i = 0; i < rows; i++) {
		outfile  << corrs.at(i);
		outfile << endl;
	}
	outfile.close();
}

void save_feature_corrs_to_csv(std::vector<std::vector<dis3::ind_dis_s>> corrs, std::string path,std::vector<uint32_t> finds);

template<typename M>
std::vector<M> load_csv_to_vector(const std::string& path) {
	std::ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<M> values;
	std::stringstream lineStream(line);
	std::string cell;
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;

		while (std::getline(lineStream, cell, ',')) {
			values.push_back(std::stoi(cell));
		}
	};
	return values;
}

pcl::CorrespondencesPtr load_result_corrs_from_csv(const std::string & path);

std::vector<pcl::CorrespondencesPtr>  load_feature_corrs_from_csv(const std::string & path, bool throw_header);

std::vector<pcl::CorrespondencesPtr>  load_feature_corrs_from_corr(const std::string & path, bool throw_header);

vector<barycentric_polygon_s> vertex_to_bar(pcl::PolygonMeshPtr mesh);

void save_bar_corrs(vector<barycentric_polygon_s> result, std::string path);

vector<barycentric_polygon_s> corrs_to_bar_corrs(std::vector<std::vector<dis3::ind_dis_s>> corrs, vector<barycentric_polygon_s> map);

#endif

