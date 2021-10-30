#include <cmath>
#include <cstdint>

#include "BullyPath.hpp"
#include "SpeedRangeSearch.hpp"
#include "Trig.hpp"

const double max_dist = pow(2, 31) - 1;

float BullyPath::calculate_current_dist() {
	float x = 0.0;
	float z = 0.0;

	for (int i = 0; i < this->path_positions.back().size(); i++) {
		x += this->path_positions.back()[i] * gSineTable[this->vector_haus[i]];
		z += this->path_positions.back()[i] * gCosineTable[this->vector_haus[i]];
	}

	return sqrtf(x * x + z * z);
}

void BullyPath::update_speed_ranges(vector<float> position, int type, int angle_idx) {
	double x_delta = 0;
	double z_delta = 0;

	float ou_max_speed = -1.0;

	for (int i = 0; i < position.size(); i++) {
		x_delta += position[i] * gSineTable[this->vector_haus[i]];
		z_delta += position[i] * gCosineTable[this->vector_haus[i]];
	}

	double max_speed = max_dist / fmax(fabs(x_delta), fabs(z_delta));

	if (type == STATE_CLEAR) {
		vector<pair<float, float>> new_speed_ranges;

		float ou_max_speed = get_ou_max_speed(x_delta, z_delta, this->start_position[0], this->start_position[2]);

		for (int i = 0; i < this->speed_ranges.size(); i++) {
			if (this->speed_ranges[i].first > max_speed) {
				break;
			}

			vector<pair<float, float>> pu_speed_ranges;

			get_pu_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], this->speed_ranges[i].first, fminf(this->speed_ranges[i].second, max_speed), pu_speed_ranges);

			for (int j = 0; j < pu_speed_ranges.size(); j++) {
				vector<pair<float, float>> floor_wall_speed_ranges;

				get_floor_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], pu_speed_ranges[j].first, pu_speed_ranges[j].second, friction_floors, floor_wall_speed_ranges);

				for (int k = 0; k < steep_floors.size(); k++) {
					get_floor_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], pu_speed_ranges[j].first, pu_speed_ranges[j].second, steep_floors[k].second, floor_wall_speed_ranges);
				}

				if (pu_speed_ranges[j].first < ou_max_speed) {
					for (int k = 0; k < walls.size(); k++) {
						get_wall_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], pu_speed_ranges[j].first, fminf(pu_speed_ranges[j].second, ou_max_speed), walls[k].first, walls[k].second, floor_wall_speed_ranges);
					}
				}

				float range_start = pu_speed_ranges[j].first;

				for (int k = 0; k < floor_wall_speed_ranges.size(); k++) {
					float range_end = nextafterf(floor_wall_speed_ranges[k].first, -INFINITY);

					if (range_start <= range_end) {
						new_speed_ranges.push_back(make_pair(range_start, range_end));
					}

					range_start = nextafterf(floor_wall_speed_ranges[k].second, INFINITY);
				}

				float range_end = pu_speed_ranges[j].second;

				if (range_start <= range_end) {
					new_speed_ranges.push_back(make_pair(range_start, range_end));
				}
			}
		}

		this->speed_ranges.swap(new_speed_ranges);
	}
	else if (type == STATE_OOB) {
		vector<pair<float, float>> new_speed_ranges;

		for (int i = 0; i < this->speed_ranges.size(); i++) {
			if (this->speed_ranges[i].first > max_speed) {
				break;
			}

			vector<pair<float, float>> pu_speed_ranges;

			get_pu_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], this->speed_ranges[i].first, fminf(this->speed_ranges[i].second, max_speed), pu_speed_ranges);

			float range_start = this->speed_ranges[i].first;

			for (int j = 0; j < pu_speed_ranges.size(); j++) {
				float range_end = nextafterf(pu_speed_ranges[j].first, -INFINITY);

				if (range_start <= range_end) {
					new_speed_ranges.push_back(make_pair(range_start, range_end));
				}

				range_start = nextafterf(pu_speed_ranges[j].second, INFINITY);
			}

			float range_end = this->speed_ranges[i].second;

			if (range_start <= range_end) {
				new_speed_ranges.push_back(make_pair(range_start, range_end));
			}
		}

		this->speed_ranges.swap(new_speed_ranges);
	}
	else if (type == STATE_STEEP_FLOOR) {
		vector<pair<float, float>> new_speed_ranges;

		for (int i = 0; i < this->speed_ranges.size(); i++) {
			if (this->speed_ranges[i].first > max_speed) {
				break;
			}

			vector<pair<float, float>> pu_speed_ranges;

			get_pu_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], this->speed_ranges[i].first, fminf(this->speed_ranges[i].second, max_speed), pu_speed_ranges);

			for (int j = 0; j < pu_speed_ranges.size(); j++) {
				get_floor_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], pu_speed_ranges[j].first, pu_speed_ranges[j].second, steep_floors[angle_idx].second, new_speed_ranges);
			}
		}

		this->speed_ranges.swap(new_speed_ranges);
	}
	else if (type == STATE_WALL) {
		vector<pair<float, float>> new_speed_ranges;

		if (ou_max_speed < 0) {
			ou_max_speed = get_ou_max_speed(x_delta, z_delta, this->start_position[0], this->start_position[2]);
		}

		max_speed = fminf(max_speed, ou_max_speed);

		for (int i = 0; i < this->speed_ranges.size(); i++) {
			if (this->speed_ranges[i].first > max_speed) {
				break;
			}

			get_wall_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], this->speed_ranges[i].first, fminf(this->speed_ranges[i].second, max_speed), walls[angle_idx].first, walls[angle_idx].second, new_speed_ranges);
		}

		this->speed_ranges.swap(new_speed_ranges);
	}
	else if (type == STATE_FRICTION_FLOOR) {
		vector<pair<float, float>> new_speed_ranges;

		for (int i = 0; i < this->speed_ranges.size(); i++) {
			if (this->speed_ranges[i].first > max_speed) {
				break;
			}

			vector<pair<float, float>> pu_speed_ranges;

			get_pu_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], this->speed_ranges[i].first, fminf(this->speed_ranges[i].second, max_speed), pu_speed_ranges);

			for (int j = 0; j < pu_speed_ranges.size(); j++) {
				get_floor_speed_ranges(x_delta, z_delta, this->start_position[0], this->start_position[2], pu_speed_ranges[j].first, pu_speed_ranges[j].second, friction_floors, new_speed_ranges);
			}
		}

		this->speed_ranges.swap(new_speed_ranges);
	}
}