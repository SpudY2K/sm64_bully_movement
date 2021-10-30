#include <algorithm>
#include <cmath>
#include "Constants.hpp"
#include "SpeedRangeSearch.hpp"
#include "Trig.hpp"

float get_ou_max_speed(double x_delta, double z_delta, double x0, double z0) {
	if (x_delta < 0) {
		x_delta = -x_delta;
		x0 = -x0;
	}

	if (z_delta < 0) {
		z_delta = -z_delta;
		z0 = -z0;
	}

	double max_speed = fmin((8192.0 - x0) / x_delta, (8192.0 - z0) / z_delta);

	return (float)max_speed;
}

void get_pu_speed_ranges(double x_delta, double z_delta, double x0, double z0, double min_speed, double max_speed, vector<pair<float, float>> &speed_ranges) {
	if (x_delta < 0) {
		x_delta = -x_delta;
		x0 = -x0;
	}

	if (z_delta < 0) {
		z_delta = -z_delta;
		z0 = -z0;
	}

	if (z_delta < x_delta) {
		swap(x_delta, z_delta);
		swap(x0, z0);
	}

	double first_pu_x_idx = ceil((x_delta*min_speed + x0 - 8192.0) / 65536.0);
	double last_pu_x_idx = floor((x_delta*max_speed + x0 + 8192.0) / 65536.0);

	for (double x = first_pu_x_idx; x <= last_pu_x_idx; x++) {
		double min_pu_x_speed = fmax(min_speed, (x*65536.0 - 8192.0 - x0) / x_delta);
		double max_pu_x_speed = fmin(max_speed, (x*65536.0 + 8192.0 - x0) / x_delta);

		double first_pu_z_idx = ceil((z_delta*min_pu_x_speed + z0 - 8192.0) / 65536.0);
		double last_pu_z_idx = floor((z_delta*max_pu_x_speed + z0 + 8192.0) / 65536.0);

		for (double z = first_pu_z_idx; z <= last_pu_z_idx; z++) {
			double min_pu_z_speed = fmax(min_pu_x_speed, (z*65536.0 - 8192.0 - z0) / z_delta);
			double max_pu_z_speed = fmin(max_pu_x_speed, (z*65536.0 + 8192.0 - z0) / z_delta);

			speed_ranges.push_back(make_pair((float)min_pu_z_speed, (float)max_pu_z_speed));
		}
	}
}

void get_floor_speed_ranges(double x_delta, double z_delta, double x0, double z0, double min_speed, double max_speed, const vector<vector<vector<float>>> &floors, vector<pair<float, float>> &speed_ranges) {
	double start_x = min_speed * x_delta + x0;
	double start_z = min_speed * z_delta + z0;

	double end_x = max_speed * x_delta + x0;
	double end_z = max_speed * z_delta + z0;

	double pu_x = 65536.0f*floor((start_x + 32768.0f) / 65536.0f);
	double pu_z = 65536.0f*floor((start_z + 32768.0f) / 65536.0f);

	for (int k = 0; k < floors.size(); k++) {
		bool side0 = ((end_x - start_x)*(pu_z + floors[k][0][1] - start_z) - (end_z - start_z)*(pu_x + floors[k][0][0] - start_x)) > 0;
		bool side1 = ((end_x - start_x)*(pu_z + floors[k][1][1] - start_z) - (end_z - start_z)*(pu_x + floors[k][1][0] - start_x)) > 0;
		bool side2 = ((end_x - start_x)*(pu_z + floors[k][2][1] - start_z) - (end_z - start_z)*(pu_x + floors[k][2][0] - start_x)) > 0;

		if (!(side0 && side1 && side2) && (side0 || side1 || side2)) {
			double speed1;
			double speed2;

			if (!(side0^side1)) {
				speed1 = ((x0 - floors[k][2][0] - pu_x)*(floors[k][2][1] - floors[k][0][1]) - (z0 - floors[k][2][1] - pu_z)*(floors[k][2][0] - floors[k][0][0])) / (z_delta * (floors[k][2][0] - floors[k][0][0]) - x_delta * (floors[k][2][1] - floors[k][0][1]));
				speed2 = ((x0 - floors[k][2][0] - pu_x)*(floors[k][2][1] - floors[k][1][1]) - (z0 - floors[k][2][1] - pu_z)*(floors[k][2][0] - floors[k][1][0])) / (z_delta * (floors[k][2][0] - floors[k][1][0]) - x_delta * (floors[k][2][1] - floors[k][1][1]));
			}
			else if (!(side0^side2)) {
				speed1 = ((x0 - floors[k][1][0] - pu_x)*(floors[k][1][1] - floors[k][0][1]) - (z0 - floors[k][1][1] - pu_z)*(floors[k][1][0] - floors[k][0][0])) / (z_delta * (floors[k][1][0] - floors[k][0][0]) - x_delta * (floors[k][1][1] - floors[k][0][1]));
				speed2 = ((x0 - floors[k][1][0] - pu_x)*(floors[k][1][1] - floors[k][2][1]) - (z0 - floors[k][1][1] - pu_z)*(floors[k][1][0] - floors[k][2][0])) / (z_delta * (floors[k][1][0] - floors[k][2][0]) - x_delta * (floors[k][1][1] - floors[k][2][1]));
			}
			else {
				speed1 = ((x0 - floors[k][0][0] - pu_x)*(floors[k][0][1] - floors[k][1][1]) - (z0 - floors[k][0][1] - pu_z)*(floors[k][0][0] - floors[k][1][0])) / (z_delta * (floors[k][0][0] - floors[k][1][0]) - x_delta * (floors[k][0][1] - floors[k][1][1]));
				speed2 = ((x0 - floors[k][0][0] - pu_x)*(floors[k][0][1] - floors[k][2][1]) - (z0 - floors[k][0][1] - pu_z)*(floors[k][0][0] - floors[k][2][0])) / (z_delta * (floors[k][0][0] - floors[k][2][0]) - x_delta * (floors[k][0][1] - floors[k][2][1]));
			}

			if (speed1 > speed2) {
				swap(speed1, speed2);
			}

			if (speed2 >= min_speed && speed1 <= max_speed) {
				speed1 = fmax(speed1, min_speed);
				speed2 = fmin(speed2, max_speed);

				speed_ranges.push_back(make_pair((float)speed1, (float)speed2));
			}
		}
	}

	sort(speed_ranges.begin(), speed_ranges.end());

	merge_speed_ranges(speed_ranges);
}


void get_wall_speed_ranges(double x_delta, double z_delta, double x0, double z0, double min_speed, double max_speed, int angle, const vector<vector<vector<float>>> &walls, vector<pair<float, float>> &speed_ranges) {
	double start_x = min_speed * x_delta + x0;
	double start_z = min_speed * z_delta + z0;

	double end_x = max_speed * x_delta + x0;
	double end_z = max_speed * z_delta + z0;

	double x_offset = bully_radius * gSineTable[angle >> 4];
	double z_offset = bully_radius * gCosineTable[angle >> 4];

	for (int k = 0; k < walls.size(); k++) {
		double speed1;
		double speed2;

		bool side0 = ((end_x - start_x)*(walls[k][0][1] + z_offset - start_z) - (end_z - start_z)*(walls[k][0][0] + x_offset - start_x)) > 0;
		bool side1 = ((end_x - start_x)*(walls[k][1][1] + z_offset - start_z) - (end_z - start_z)*(walls[k][1][0] + x_offset - start_x)) > 0;
		bool side2 = ((end_x - start_x)*(walls[k][0][1] - z_offset - start_z) - (end_z - start_z)*(walls[k][0][0] - x_offset - start_x)) > 0;
		bool side3 = ((end_x - start_x)*(walls[k][1][1] - z_offset - start_z) - (end_z - start_z)*(walls[k][1][0] - x_offset - start_x)) > 0;

		if (!(side0 && side1 && side2 && side3) && (side0 || side1 || side2 || side3)) {
			if (side0^side1) {
				speed1 = ((x0 - walls[k][0][0] - x_offset)*(walls[k][0][1] - walls[k][1][1]) - (z0 - walls[k][0][1] - z_offset)*(walls[k][0][0] - walls[k][1][0])) / (z_delta * (walls[k][0][0] - walls[k][1][0]) - x_delta * (walls[k][0][1] - walls[k][1][1]));

				if (side0^side2) {
					speed2 = (2.0*z_offset*(x0 - walls[k][0][0] - x_offset) - 2.0*x_offset*(z0 - walls[k][0][1] - z_offset)) / (2.0 * z_delta * x_offset - 2.0 * x_delta * z_offset);
				}
				else if (side1^side3) {
					speed2 = (2.0*z_offset*(x0 - walls[k][1][0] - x_offset) - 2.0*x_offset*(z0 - walls[k][1][1] - z_offset)) / (2.0 * z_delta * x_offset - 2.0 * x_delta * z_offset);
				}
				else {
					speed2 = ((x0 - walls[k][0][0] + x_offset)*(walls[k][0][1] - walls[k][1][1]) - (z0 - walls[k][0][1] + z_offset)*(walls[k][0][0] - walls[k][1][0])) / (z_delta * (walls[k][0][0] - walls[k][1][0]) - x_delta * (walls[k][0][1] - walls[k][1][1]));
				}
			}
			else if (side0^side2) {
				speed1 = (2.0*z_offset*(x0 - walls[k][0][0] - x_offset) - 2.0*x_offset*(z0 - walls[k][0][1] - z_offset)) / (2.0 * z_delta * x_offset - 2.0 * x_delta * z_offset);

				if (side1^side3) {
					speed2 = (2.0*z_offset*(x0 - walls[k][1][0] - x_offset) - 2.0*x_offset*(z0 - walls[k][1][1] - z_offset)) / (2.0 * z_delta * x_offset - 2.0 * x_delta * z_offset);
				}
				else {
					speed2 = ((x0 - walls[k][0][0] + x_offset)*(walls[k][0][1] - walls[k][1][1]) - (z0 - walls[k][0][1] + z_offset)*(walls[k][0][0] - walls[k][1][0])) / (z_delta * (walls[k][0][0] - walls[k][1][0]) - x_delta * (walls[k][0][1] - walls[k][1][1]));
				}
			}
			else {
				speed1 = (2.0*z_offset*(x0 - walls[k][1][0] - x_offset) - 2.0*x_offset*(z0 - walls[k][1][1] - z_offset)) / (2.0 * z_delta * x_offset - 2.0 * x_delta * z_offset);
				speed2 = ((x0 - walls[k][0][0] + x_offset)*(walls[k][0][1] - walls[k][1][1]) - (z0 - walls[k][0][1] + z_offset)*(walls[k][0][0] - walls[k][1][0])) / (z_delta * (walls[k][0][0] - walls[k][1][0]) - x_delta * (walls[k][0][1] - walls[k][1][1]));
			}

			if (speed1 > speed2) {
				swap(speed1, speed2);
			}

			if (speed2 >= min_speed && speed1 <= max_speed) {
				speed1 = fmax(speed1, min_speed);
				speed2 = fmin(speed2, max_speed);

				speed_ranges.push_back(make_pair((float)speed1, (float)speed2));
			}
		}
	}

	sort(speed_ranges.begin(), speed_ranges.end());

	merge_speed_ranges(speed_ranges);
}

void merge_speed_ranges(vector<pair<float, float>> &speed_ranges) {
	if (speed_ranges.size() > 1) {
		vector<pair<float, float>> merged_speed_ranges;

		pair<float, float> current_range = speed_ranges.front();

		for (int i = 1; i < speed_ranges.size(); i++) {
			if (current_range.second >= speed_ranges[i].first) {
				current_range.second = speed_ranges[i].second;
			}
			else {
				merged_speed_ranges.push_back(current_range);
				current_range = speed_ranges[i];
			}
		}

		merged_speed_ranges.push_back(current_range);

		speed_ranges.swap(merged_speed_ranges);
	}
}