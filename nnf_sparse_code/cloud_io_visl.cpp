#include <cloud_io_visl.hpp>

using namespace std;
using namespace dis3;

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

void saveTransform(string file_name, Eigen::Matrix4f& transform) {
	ofstream transformation_file;
	transformation_file.open(file_name);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cout << transform(j, i) << " ";
			transformation_file << transform(j, i) << " ";
		}
		transformation_file << endl;
		cout << endl;
	}
	cout << endl;
	transformation_file.close();
}


Eigen::Matrix4f readTransform(string file_name) {
	Eigen::Matrix4f result;
	ifstream transformation_file;
	transformation_file.open(file_name);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			transformation_file >> result(j, i);
		}
	}
	transformation_file.close();
	return result;
}

void printSection(string outputString, string variableString ) {
	cout << endl << "******************************************************************************" << endl;
	cout <<"\t"<< outputString << variableString << endl;
	cout << "******************************************************************************" << endl << endl;
}


float getUserValue(string varName, float defaultValue, bool Autodefault) {
	string::size_type sz;
	string input = "";
	if (!Autodefault) {
		cout << "Please enter " << varName << " value (" << defaultValue << "):";
		std::getline(cin, input);
	}
	else {
		cout << "Using Default " << varName << " value (" << defaultValue << ")";
	}
	if (!input.empty())
		return stof(input, &sz);
	else
		return defaultValue;
}

bool getUserValue(string varName, bool defaultValue, bool Autodefault) {
	string input = "";
	if (!Autodefault) {
		cout << "Please enter wether you want " << varName << "[y/n](" << defaultValue << "):";
		std::getline(cin, input);
	} else {
		cout << "Using default option for" << varName << "[y/n] : " << defaultValue ;
	}
	if (!input.empty()) {
		if (!(input.compare(0, 1, "y")))
			return true;
		else
			return false;
	}
	else
		return defaultValue;
}


int getUserValue(string varName, int defaultValue, bool Autodefault) {
	string::size_type sz;
	string input = "";
	if (!Autodefault) {
		cout << "Please enter " << varName << " value (" << defaultValue << "):";
		std::getline(cin, input);
	}
	else {
		cout << "Using Default " << varName << " value (" << defaultValue << ")";
	}
	if (!input.empty())
		return stof(input, &sz);
	else
		return defaultValue;
}

void saveStats(string filename, float BBS, float Diversity) {
	ofstream stats_file;
	stats_file.open(filename);
	stats_file << "Number of Best Buddies:" << BBS << std::endl << "Diversity Score:" << Diversity;
	stats_file.close();
}

void saveres(string filename, int ind) {
	ofstream stats_file;
	stats_file.open(filename);
	stats_file << "Center of result:" << ind << std::endl;
	stats_file.close();
}

vector<barycentric_polygon_s> load_gt_map_bar(string filename) {
	vector<barycentric_polygon_s> result;
	ifstream vec_file(filename);
	//faces = *(new vector<Vertices>);
	string line;
	int i = 0;
	if (vec_file.is_open()) {
		while (getline(vec_file, line)) {
			std::istringstream iss(line);
			std::vector<std::string> results((std::istream_iterator<std::string>(iss))
				,std::istream_iterator<std::string>());
			barycentric_polygon_s bar;
			bar.index = stringetof(results.at(0))-1;
			bar.coor[0] = stringetof(results.at(1));
			bar.coor[1] = stringetof(results.at(2));
			bar.coor[2] = stringetof(results.at(3));
			result.push_back(bar);
		}
	}
	return result;
}

vector<int> load_gt_map(string filename) {
	vector<int> result;
	ifstream vec_file(filename);
	//faces = *(new vector<Vertices>);
	string line;
	int i = 0;
	if (vec_file.is_open()) {
		while (getline(vec_file, line)) {
			result.push_back(stoi(line)-1);
		}
	}
	return result;
}

int loadres(string filename) {
	ifstream stats_file;
	string stat;
	stats_file.open(filename);
	std::getline(stats_file, stat);
	stringstream iss(stat);
	string SingleLine;
	getline(iss, SingleLine, ':');
	getline(iss, SingleLine, '\n');
	stats_file.close();
	return stoi(SingleLine);
}

void saveStats(string filename, float mean, int Zeros) {
	ofstream stats_file;
	stats_file.open(filename);
	stats_file << "Mean number of points for FPFH:" << mean << std::endl << "Number of Zeros" << Zeros;
	stats_file.close();
}


float load_score(string filename) {
	ifstream stats_file;
	string stat;
	stats_file.open(filename);
	std::getline(stats_file, stat);

	std::getline(stats_file, stat);
	cout << "Reading " << stat.substr(16, stat.size());
	return stof(stat.substr(16, stat.size()));
}

bool savePolygonMeshToFile(pcl::PolygonMeshPtr mesh, string filenameString) {
	if (mesh->cloud.width > 0) {
		cout << "writing mesh to:" << filenameString << endl;
		pcl::io::savePLYFileBinary(filenameString, *mesh);
		return true;
	}
	return false;
}

