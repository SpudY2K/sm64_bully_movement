#pragma once

#include <vector>
#include <cstdint>

using namespace std;

#ifndef BULLY_PATH_H
#define BULLY_PATH_H

#define STATE_CLEAR 0
#define STATE_OOB 1
#define STATE_STEEP_FLOOR 2
#define STATE_FRICTION_FLOOR 3
#define STATE_WALL 4
#define STATE_FLOOR_BOUNCE 5
#define STATE_START 6

class BullyPath {
public:
	vector<vector<float>> path_positions = { { 0.0 } };

	vector<float> start_position;
	vector<int> vector_haus;
	vector<float> vector_lengths = { 1.0 };

	vector<vector<float>> checked_positions = { { 0.0 } };
	vector<int> checked_states = { STATE_START };

	vector<pair<float, float>> speed_ranges;

	int start_yaw;
	int current_yaw;
	float current_direction = 1.0;
	float current_length = 1.0;
	int falling_frames = 0;

	bool falling = true;

	BullyPath() {}

	BullyPath(vector<float> pos, int yaw, float min_speed, float max_speed) {
		start_position = pos;
		start_yaw = yaw;
		current_yaw = yaw;
		vector_haus.push_back((uint16_t)yaw >> 4);
		speed_ranges = { make_pair(min_speed, max_speed) };
	}

	void update_speed_ranges(vector<float> position, int type, int angle_idx);
	float calculate_current_dist();
};
#endif
